# -*- coding: utf-8 -*-
"""
Ventana principal de la aplicación de control (BrazoRobot).

La clase en sí solo contiene el constructor y los dos manejadores de
eventos de Qt que le son propios (closeEvent/keyPressEvent); todo el resto
de funcionalidad se reparte en mixins por área temática (mixins/*.py), que
BrazoRobot combina por herencia múltiple. Cada mixin es legible por
separado y solo depende de los módulos "puros" (kinematics, app_paths,
dialogs, widgets, routine_editor_widgets), nunca de otro mixin.
"""
import os
from collections import deque

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QSlider, QPushButton, QLineEdit, QTextEdit, QComboBox, QSizePolicy, QToolButton,
    QMessageBox, QScrollArea, QFrame, QGridLayout, QSplashScreen, QProgressBar,
    QTabWidget, QTableWidget, QTableWidgetItem, QButtonGroup, QRadioButton,
    QHeaderView, QAbstractItemView, QCheckBox, QMenu, QAction, QSplitter,
    QFileDialog, QDialog, QGraphicsScene, QGraphicsView, QGraphicsRectItem,
    QGraphicsEllipseItem, QGraphicsPolygonItem, QGraphicsPathItem, QGraphicsTextItem,
    QShortcut, QSpinBox, QDoubleSpinBox
)
from PyQt5.QtGui import QPixmap, QFont, QPen, QBrush, QColor, QPolygonF, QPainterPath, QPainter, QDrag, QKeySequence
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject, QPointF, QMimeData
from pyvistaqt import QtInteractor

from mixins.diagnostics import DiagnosticsMixin
from mixins.log import LogMixin
from mixins.routines import RoutinesMixin
from mixins.manual_control import ManualControlMixin
from mixins.visualization3d import Visualization3DMixin
from mixins.serial_comm import SerialCommMixin


class BrazoRobot(QMainWindow, DiagnosticsMixin, LogMixin, RoutinesMixin,
                  ManualControlMixin, Visualization3DMixin, SerialCommMixin):
    """Ventana principal: compone todos los mixins de funcionalidad.

    El orden de herencia no importa para el funcionamiento (cada mixin solo
    usa sus propios atributos/métodos y los que inicializa __init__), pero
    se mantiene agrupado por tema para que sea fácil encontrar dónde vive
    cada pestaña/panel de la interfaz.
    """

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Simulación brazo robótico STEP")
        self.setStyleSheet(self.dark_theme())
 
        self.angles = [0.0] * 6
        self.serial_port = None   # objeto serial.Serial activo

        # Ghost arm
        self.ghost_actors        = []
        self._ghost_visible      = False

        # Programa por bloques — motor de ejecución
        self._prog_running    = False
        self._prog_paused     = False   # True = congelada, resumible con Run
        self._prog_gen        = None
        self._prog_state      = 'needs_next'   # 'needs_next'|'moving'|'waiting'|'settling'
        self._prog_highlighted_widget = None    # BlockWidget resaltado como "en curso"
        self._prog_anim_step  = 0
        self._prog_anim_total = 0
        self._prog_anim_q_start = None
        self._prog_anim_q_end   = None
        # Ángulos objetivo (0-360, ya resueltos/esquivando zona prohibida)
        # del bloque de movimiento en curso, usados por _prog_settle_tick()
        # para comprobar la llegada real. Independiente de self.angles
        # porque, desde que el setpoint real se manda al empezar el bloque
        # (ver _prog_start_move), self.angles puede seguir cambiando por la
        # animación local del visor 3D mientras el robot real ya llegó.
        self._prog_settle_target = None
        self._prog_wait_ticks   = 0
        # Timeout de seguridad del estado 'settling' (segundos): si tras
        # mandar un setpoint el robot no confirma "llegada estable" antes de
        # este tiempo, se fuerza el avance al siguiente bloque avisando en el
        # log — ver _prog_tick(), estado 'settling'. Editable desde la
        # pestaña Rutinas (self._prog_timeout_spin).
        self._prog_settle_timeout_s = 30.0
        self._prog_timer = QTimer(self)
        self._prog_timer.setInterval(25)
        self._prog_timer.timeout.connect(self._prog_tick)

        # Tolerancia angular para considerar que un movimiento de rutina
        # "llegó" al objetivo (grados). Con 1.5° (el MOVEMENT_TOL del firmware,
        # pensado solo para la baliza) el error cartesiano rondaba 8-13 mm.
        # Con 0.5° bajó a 2-4 mm... pero SOLO en los puntos donde el PID puede
        # realmente asentarse por debajo de 0.5°; en los demás (esquina1,
        # esquina2, HOME) el robot nunca llega a estar tan quieto y cada
        # movimiento agota el timeout de 30 s esperando algo que el propio PID
        # no está configurado para dar (su zona muerta real, tol_deg, es mayor
        # que 0.5° para esas articulaciones/poses). 1.0° es el punto medio:
        # sigue mejorando bastante sobre 1.5° sin forzar timeout en casi todos
        # los movimientos. Si algún punto concreto sigue dando timeout con
        # esto, el límite ya no está aquí sino en tol_deg de ese PID (Config.
        # de nodos) — apretar más este valor solo quemará más tiempo sin
        # ganar precisión real. Editable desde la pestaña Rutinas
        # (self._prog_tol_spin).
        self._PROG_MOVEMENT_TOL_DEG = 1.0

        # Ventana de estabilidad (segundos) del estado 'settling': tiempo
        # mínimo seguido con max_delta<0.5° Y dentro de _PROG_MOVEMENT_TOL_DEG
        # antes de dar el bloque por "llegado" (ver _prog_tick()). Por
        # defecto 0.5 s = 20 ticks de 25 ms. Editable desde la pestaña
        # Rutinas (self._prog_stable_spin).
        self._prog_stable_window_s = 0.5

        # Destino (XYZ mm + nombre de variable si aplica) del último bloque de
        # movimiento lanzado por la rutina; usado por _prog_log_arrival() para
        # comparar contra dónde se detuvo realmente el robot.
        self._prog_target_xyz = None
        self._prog_target_var = None
        self._prog_current_vel_pct = 100.0  # % de velocidad del bloque move/move_ori en curso

        # Variables de posición nombradas { nombre: {x,y,z,traj} }
        self._position_vars = {}
        # Setpoints enviados (para gráfica PID)
        self._joint_setpoints = [0.0] * 6
        # Traza 3D de trayectoria
        self._traj_tracing = False
        self._traj_pts     = []
        self._traj_actor   = None
        self._preview_actor = None   # preview de rutina en 3D
        self._workspace_actor = None  # envolvente de trabajo 3D
        self._workspace_visible = False
        # Pila de subrutinas en ejecución (anti-recursión)
        self._prog_call_stack = set()
        # Log de errores
        self._log_widget = None   # QTextEdit, asignado en _build_log_tab
        # Historial de posiciones (últimas 20)
        self._pos_history = deque(maxlen=20)
        self._pos_hist_rows_layout = None  # asignado en _build_rutinas_tab

        # Animación ghost
        self._ghost_anim_q_start = None   # np.array de ángulos de inicio
        self._ghost_anim_q_end   = None   # np.array de ángulos objetivo
        self._ghost_anim_step    = 0
        self._ghost_anim_total   = 0
        self._ghost_anim_err_pos = 0.0
        self._ghost_anim_err_ori = 0.0
        self._ghost_timer        = QTimer(self)
        self._ghost_timer.timeout.connect(self._animate_ghost_step)

        # Signos visuales (calibrados para el 3D)
        self.rotation_signs_visual = [1, 1, -1, -1, 1, 1]
 
        # Signos MDH (calibrados con el panel de diagnóstico)
        self.rotation_signs_mdh = [1, 1, 1, 1, -1, 1]
 
        self.sliders               = []
        self.labels                = []
        self.real_angle_labels     = []
        self.real_angles_feedback  = [0.0] * 6   # ángulos reales recibidos del ESP32
        self._joint_offsets        = [0.0] * 6   # offsets de calibración (°)
        self._joint_raw_feedback   = [0.0] * 6   # lectura bruta antes de offset
        self.inputs                = []
        self.connection_indicators = []
        self.sign_buttons_mdh      = []
        self.enable_buttons        = []
        self._enable_states        = [False] * 6   # False = habilitado (motor activo)
        self._viz_real             = False
        self._free_move            = False
        self._dry_run_mode         = False   # modo simulacion: no envia nada al hardware
        self._teach_mode_active    = False   # modo Teach: capturar puntos a mano

        # Cinta transportadora (ESP32-C3 propio, ver central "cinta="/"cintadist=").
        # Estado recibido del central por Serial (ver _parse_cinta_estado);
        # cinta_objeto_detectado lo usa la condición "Pieza (cinta)" de las
        # rutinas (ver _eval_prog_cond). cinta_distancia_umbral_cm es lo
        # último que la propia app mandó con "cintadist=" (para mostrarlo en
        # la UI de configuración sin depender de que el hardware lo confirme).
        self.cinta_objeto_detectado    = False
        self.cinta_distancia_cm        = -1.0    # última distancia MEDIDA por el HC-SR04 (cm), -1 = sin lectura
        self.cinta_distancia_umbral_cm = 4.0      # umbral de detección configurado desde la app (cm)
        self._free_move_timer      = QTimer(self)
        self._free_move_timer.setInterval(150)
        self._free_move_timer.timeout.connect(self._free_move_tick)
        self._range_warning_active     = False
        self._range_warning_blink_timer = QTimer(self)
        self._range_warning_blink_timer.setInterval(400)
        self._range_warning_blink_timer.timeout.connect(self._range_warning_blink_tick)

        # Deshacer/Rehacer del editor de rutinas
        self._prog_undo_stack = []
        self._prog_redo_stack = []
        self._prog_undo_max   = 50

        # Perfiles de herramienta (TCP offset) -- carga el activo al iniciar
        self._tool_profiles       = {}
        self._active_tool_profile = 'Sin offset'
        self._load_tool_profiles()

 
        main_widget  = QWidget()
        self.setCentralWidget(main_widget)
        root_layout = QVBoxLayout(main_widget)
        root_layout.setSpacing(0)
        root_layout.setContentsMargins(4, 4, 4, 4)
 
        # ── Splitter vertical: arriba 3D+controles / abajo diagnóstico ───
        self._main_splitter = QSplitter(Qt.Vertical)
        self._main_splitter.setHandleWidth(7)
        self._main_splitter.setStyleSheet("""
            QSplitter::handle:vertical {
                background: qlineargradient(x1:0,y1:0,x2:0,y2:1,
                    stop:0 #23262b, stop:0.4 #f0a500, stop:0.6 #f0a500, stop:1 #23262b);
                border-radius: 3px; margin: 1px 40px;
            }
            QSplitter::handle:vertical:hover {
                background: qlineargradient(x1:0,y1:0,x2:0,y2:1,
                    stop:0 #23262b, stop:0.3 #ffd060, stop:0.7 #ffd060, stop:1 #23262b);
            }
        """)
        root_layout.addWidget(self._main_splitter)
 
        # ── Fila superior: 3D + controles (dentro del splitter) ──
        # Splitter horizontal: izquierda 3D / derecha Controles Manuales
        top_splitter = QSplitter(Qt.Horizontal)
        top_splitter.setHandleWidth(4)
        top_splitter.setStyleSheet("""
            QSplitter::handle {
                margin: 0px;
            }
            QSplitter::handle:horizontal {
                background: #2e3340;
                border-left:  1px solid #70e570;
                border-right: 1px solid #70e570;
                margin: 0px;
            }
            QSplitter::handle:horizontal:hover {
                background: #70e570;
                margin: 0px;
            }
        """)
        self._top_splitter = top_splitter

        left_frame  = QFrame()
        left_frame.setStyleSheet("QFrame { border: 2px solid #70e570; border-radius: 5px; }")
        left_layout = QVBoxLayout(left_frame)

        # Cabecera 3D con botón de colapsar panel derecho
        hdr_3d = QHBoxLayout()
        lbl_3d = QLabel("🖥️ VISUAL. 3D")
        lbl_3d.setStyleSheet("font-size:8px; font-weight:bold; color:#70e570; padding:1px;")
        lbl_3d.setAlignment(Qt.AlignCenter)
        self._ctrl_collapse_btn = QPushButton("▶")
        self._ctrl_collapse_btn.setToolTip("Plegar / desplegar Controles Manuales")
        self._ctrl_collapse_btn.setFixedSize(19, 14)
        self._ctrl_collapse_btn.setStyleSheet(
            "QPushButton { border:1px solid #70e570; color:#70e570; background:transparent;"
            " border-radius:4px; font-weight:bold; }"
            "QPushButton:hover { background:#70e570; color:#23262b; }"
        )
        self._ws_btn = QPushButton('🌐 Envolvente')
        self._ws_btn.setToolTip('Mostrar/ocultar envolvente de trabajo 3D')
        self._ws_btn.setCheckable(True)
        self._ws_btn.setStyleSheet(
            'QPushButton { border:1px solid #446688; color:#6699bb; background:transparent;'
            ' border-radius:4px; font-size:9px; padding:2px 6px; }'
            'QPushButton:checked { background:#224466; color:#88bbdd; border-color:#88bbdd; }'
            'QPushButton:hover { background:#1a3355; }')
        self._ws_btn.clicked.connect(self._toggle_workspace)

        self._viz_real_btn = QPushButton('👁 Vista App')
        self._viz_real_btn.setToolTip('Cambiar entre angulos de la app o reales del ESP32')
        self._viz_real_btn.setCheckable(True)
        self._viz_real_btn.setStyleSheet(
            'QPushButton { border:1px solid #886644; color:#bbaa77; background:transparent;'
            ' border-radius:4px; font-size:9px; padding:2px 6px; }'
            'QPushButton:checked { background:#443311; color:#ffcc66; border-color:#ffcc66; }'
            'QPushButton:hover { background:#332200; }')
        self._viz_real_btn.clicked.connect(self._toggle_viz_real)

        self._free_btn = QPushButton('🔓 Mover libre')
        self._free_btn.setToolTip('Deshabilita motores y sigue angulos reales en 3D')
        self._free_btn.setCheckable(True)
        self._free_btn.setStyleSheet(
            'QPushButton { border:1px solid #664488; color:#aa77cc; background:transparent;'
            ' border-radius:4px; font-size:9px; padding:2px 6px; }'
            'QPushButton:checked { background:#331144; color:#dd99ff; border-color:#dd99ff; }'
            'QPushButton:hover { background:#220033; }')
        self._free_btn.clicked.connect(self._toggle_free_move)

        self._tcp_btn = QPushButton('TCP')
        self._tcp_btn.setToolTip('Perfiles de herramienta (offset TCP: pinza, ventosa, etc.)')
        self._tcp_btn.setStyleSheet(
            'QPushButton { border:1px solid #26C6DA; color:#26C6DA; background:transparent;'
            ' border-radius:4px; font-size:9px; padding:2px 6px; }'
            'QPushButton:hover { background:#0d2a3d; }')
        self._tcp_btn.clicked.connect(self._open_tool_profile_dialog)

        self._reset_view_btn = QPushButton('🎥')
        self._reset_view_btn.setToolTip('Recuperar la vista de cámara con la que arranca la app')
        self._reset_view_btn.setStyleSheet(
            'QPushButton { border:1px solid #888888; color:#cccccc; background:transparent;'
            ' border-radius:4px; font-size:9px; padding:2px 6px; }'
            'QPushButton:hover { background:#333333; }')
        self._reset_view_btn.clicked.connect(self._reset_camera_view)

        # Botón de cierre visible: esta ventana arranca en modo kiosco (sin
        # bordes ni barra de título, ver main()) para la pantalla táctil de
        # 800x480, así que no hay "X" nativa — hace falta un botón propio.
        self._close_btn = QPushButton('✕')
        self._close_btn.setToolTip('Cerrar la aplicación')
        self._close_btn.setFixedSize(20, 16)
        self._close_btn.setStyleSheet(
            'QPushButton { border:1px solid #aa4444; color:#ff8888; background:transparent;'
            ' border-radius:4px; font-weight:bold; font-size:10px; }'
            'QPushButton:hover { background:#aa2222; color:white; }')
        self._close_btn.clicked.connect(self.close)

        hdr_3d.addWidget(lbl_3d, 1)
        hdr_3d.addWidget(self._viz_real_btn)
        hdr_3d.addWidget(self._free_btn)
        hdr_3d.addWidget(self._tcp_btn)
        hdr_3d.addWidget(self._ws_btn)
        hdr_3d.addWidget(self._reset_view_btn)
        hdr_3d.addWidget(self._ctrl_collapse_btn)
        hdr_3d.addWidget(self._close_btn)
        left_layout.addLayout(hdr_3d)

        self._range_warning_lbl = QLabel('')
        self._range_warning_lbl.setAlignment(Qt.AlignCenter)
        self._range_warning_lbl.setStyleSheet(
            'background:#cc2222; color:white; font-weight:bold; font-size:10px;'
            ' padding:4px; border-radius:3px;')
        self._range_warning_lbl.setVisible(False)
        left_layout.addWidget(self._range_warning_lbl)

        self.plotter_widget = QtInteractor()
        self.plotter_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        left_layout.addWidget(self.plotter_widget)

        right_frame  = QFrame()
        right_frame.setStyleSheet("QFrame { border: 2px solid #70e570; border-radius: 5px; }")
        right_layout = QVBoxLayout(right_frame)
        lbl_ctrl = QLabel("🎮 CONTROLES MANUALES")
        lbl_ctrl.setStyleSheet("font-size:9px; font-weight:bold; color:#70e570; padding:2px;")
        lbl_ctrl.setAlignment(Qt.AlignCenter)

        # Botón para ocultar/mostrar la vista 3D (simétrico al que oculta
        # este mismo panel desde la cabecera de la vista 3D) — en una
        # pantalla de 800x480 a veces interesa dar todo el ancho a los
        # controles y prescindir del render 3D.
        self._viz3d_collapse_btn = QPushButton("◀")
        self._viz3d_collapse_btn.setToolTip("Ocultar visualización 3D")
        self._viz3d_collapse_btn.setFixedSize(19, 14)
        self._viz3d_collapse_btn.setStyleSheet(
            "QPushButton { border:1px solid #70e570; color:#70e570; background:transparent;"
            " border-radius:4px; font-weight:bold; }"
            "QPushButton:hover { background:#70e570; color:#23262b; }"
        )
        hdr_ctrl = QHBoxLayout()
        hdr_ctrl.addWidget(self._viz3d_collapse_btn)
        hdr_ctrl.addWidget(lbl_ctrl, 1)
        right_layout.addLayout(hdr_ctrl)
        self._ctrl_frame = right_frame

        right_layout.addWidget(self._build_serial_panel())

        scroll_area   = QScrollArea(); scroll_area.setWidgetResizable(True)
        scroll_widget = QWidget()
        self.controls_layout = QVBoxLayout(scroll_widget)
        scroll_area.setWidget(scroll_widget)
        scroll_area.setStyleSheet("""
            QScrollArea { border:none; background:transparent; }
            QScrollBar:vertical { background:#23262b; width:14px; margin:16px 3px; border-radius:7px; }
            QScrollBar::handle:vertical { background:#70e570; border-radius:7px; min-height:20px; }
        """)
        right_layout.addWidget(scroll_area)

        # Lógica del botón colapsar / expandir
        self._ctrl_prev_sizes = [190, 590]
        def _toggle_ctrl_panel():
            if self._ctrl_frame.isVisible():
                self._ctrl_prev_sizes = self._top_splitter.sizes()
                self._ctrl_frame.setVisible(False)
                self._ctrl_collapse_btn.setText("◀")
                self._ctrl_collapse_btn.setToolTip("Mostrar Controles Manuales")
            else:
                self._ctrl_frame.setVisible(True)
                self._top_splitter.setSizes(self._ctrl_prev_sizes)
                self._ctrl_collapse_btn.setText("▶")
                self._ctrl_collapse_btn.setToolTip("Plegar Controles Manuales")
        self._ctrl_collapse_btn.clicked.connect(_toggle_ctrl_panel)

        # Lógica del botón ocultar / mostrar la vista 3D (mismo patrón que
        # el de arriba, pero sobre left_frame en vez de _ctrl_frame).
        self._viz3d_prev_sizes = [190, 590]
        def _toggle_viz3d_panel():
            if left_frame.isVisible():
                self._viz3d_prev_sizes = self._top_splitter.sizes()
                left_frame.setVisible(False)
                self._viz3d_collapse_btn.setText("▶")
                self._viz3d_collapse_btn.setToolTip("Mostrar visualización 3D")
            else:
                left_frame.setVisible(True)
                self._top_splitter.setSizes(self._viz3d_prev_sizes)
                self._viz3d_collapse_btn.setText("◀")
                self._viz3d_collapse_btn.setToolTip("Ocultar visualización 3D")
        self._viz3d_collapse_btn.clicked.connect(_toggle_viz3d_panel)

        top_splitter.addWidget(left_frame)
        top_splitter.addWidget(right_frame)
        # En la pantalla de 800x480 el panel de CONTROLES MANUALES (fila de
        # puerto serie, botones, etc.) necesita más ancho que la vista 3D
        # para no requerir scroll horizontal — al revés que en el monitor
        # de PC, donde la vista 3D es la protagonista.
        top_splitter.setSizes([190, 590])
        top_splitter.setStretchFactor(0, 1)
        top_splitter.setStretchFactor(1, 3)

        # ── Fila inferior: panel de diagnóstico ──────────
        self.diag_frame = self._build_diagnostic_panel()
        self._main_splitter.addWidget(top_splitter)
        self._main_splitter.addWidget(self.diag_frame)
        self._main_splitter.setSizes([650, 200])
        self.setup_manual_controls()
        self.setup_3d_visualization()
        self.update_xyz_display()
        self.update_3d_visualization()
        self._load_joint_offsets()   # cargar offsets de calibración si existen
 
    # ─────────────────────────────────────────
    #  Panel de diagnóstico / calibración
    # ─────────────────────────────────────────
 

    def closeEvent(self, event):
        self._disconnect_serial()
        super().closeEvent(event)

    def keyPressEvent(self, event):
        # Esta ventana arranca en modo kiosco (sin bordes ni barra de
        # título, ver main()) para la pantalla táctil de 800x480, así que
        # no hay botón "X" para cerrarla. Con un teclado conectado (o por
        # SSH con xdotool): Esc cierra la app, F11 alterna pantalla
        # completa / ventana normal para depurar en el propio panel táctil.
        if event.key() == Qt.Key_Escape:
            self.close()
        elif event.key() == Qt.Key_F11:
            if self.isFullScreen():
                self.showNormal()
            else:
                self.showFullScreen()
        else:
            super().keyPressEvent(event)

    # ─────────────────────────────────────────
    #  Envio de setpoints al ESP32
    # ─────────────────────────────────────────

    def dark_theme(self):
        return """
        QWidget { background:#23262b; color:#E0E0E0;
                  font-family:'Segoe UI',Arial; font-size:10px; }
        QPushButton { background-color:#31364a; border:1.2px solid #5dd095;
                      border-radius:8px; padding:5px 10px;
                      color:#70e570; font-weight:bold; }
        QPushButton:hover { background-color:#70e570; color:#23262b; }
        QLineEdit, QComboBox, QTextEdit { background:#2b2e38; border:1.2px solid #5dd095;
                      border-radius:5px; color:#e0e0e0; padding:4px; }
        QLabel { color:#93ffa8; font-weight:600; }
        QSlider::groove:horizontal { height:8px; background:#393e4c; border-radius:4px; }
        QSlider::handle:horizontal { background:#70e570; border-radius:8px;
                      height:16px; width:16px; margin:-4px 0; }
        QTextEdit { background:#181c1f; color:#97fa97;
                    font-family:'Consolas',monospace; border-radius:6px; }
        """
