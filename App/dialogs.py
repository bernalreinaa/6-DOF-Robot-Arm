# -*- coding: utf-8 -*-
"""
Diálogos independientes de la ventana principal: configuración de nodos
(PID/velocidad/zona prohibida) y de la cinta, perfiles de herramienta (TCP),
señal de progreso de arranque y el asistente de calibración de offsets.
"""
import os
import json
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

from kinematics import JOINT_FORBIDDEN_ZONES, set_forbidden_zone, ik_dls
from widgets import ResponseGraph

class NodeConfigDialog(QDialog):
    """Ventana independiente para configurar los parametros PID y de velocidad
    de cada articulacion del brazo robotico.

    Formato enviado al ESP32:
        init[i]kp;ki;kd;max_vel;cruise_vel;approach_vel;min_vel;tol_deg;slow_zone_deg;approach_zone_deg;\\n
    donde i = 0-based (0..5), sin '=' tras el corchete.
    """

    # (nombre visible, clave interna, valor por defecto)
    PARAMS = [
        ("Kp",                "kp",                5.0  ),
        ("Ki",                "ki",                0.0  ),
        ("Kd",                "kd",                0.1  ),
        ("Vel. maxima",       "max_vel",           15.0 ),
        ("Vel. crucero",      "cruise_vel",         8.0 ),
        ("Vel. aproximacion", "approach_vel",       3.0 ),
        ("Vel. minima",       "min_vel",            1.0 ),
        ("Tolerancia (deg)",  "tol_deg",            1.0 ),
        ("Zona lenta (deg)",  "slow_zone_deg",     10.0 ),
        ("Zona aprox. (deg)", "approach_zone_deg",  5.0 ),
        ("Límite inferior (deg)", "limit_inf_motor", -180.0),
        ("Límite superior (deg)", "limit_sup_motor",  180.0)
    ]

    def __init__(self, main_window):
        super().__init__(main_window)
        self._main = main_window
        self.setWindowTitle("Configuracion de nodos — ESP32")
        self.setMinimumWidth(300)
        self.setStyleSheet(self._theme())
        self._build_ui()
        # Auto-cargar valores de la articulación 1 al abrir
        QTimer.singleShot(150, self._reload_config)
        # Debounce para auto-envío
        self._debounce = QTimer(self)
        self._debounce.setSingleShot(True)
        self._debounce.setInterval(600)
        self._debounce.timeout.connect(self._auto_send)
        # Muestreo de datos para la gráfica (100 ms)
        self._sample_timer = QTimer(self)
        self._sample_timer.setInterval(100)
        self._sample_timer.timeout.connect(self._sample_pid_data)
        self._sample_timer.start()
        self.finished.connect(self._sample_timer.stop)
        # Conectar spinboxes al debounce
        for spin in self._spin_boxes.values():
            spin.valueChanged.connect(self._on_spin_changed)
        # Limpiar gráfica al cambiar articulación
        self.joint_combo.currentIndexChanged.connect(self._resp_graph.clear_data)

    # ── Construccion de la UI ──────────────────────────────────────────────

    def _build_ui(self):
        from PyQt5.QtWidgets import QDoubleSpinBox
        root = QVBoxLayout(self)
        root.setSpacing(6)
        root.setContentsMargins(8, 8, 8, 8)

        # Titulo
        title = QLabel("CONFIGURACION DE NODO / ARTICULACION")
        title.setStyleSheet(
            "font-size:12px; font-weight:bold; color:#ff9900; padding:4px;"
        )
        title.setAlignment(Qt.AlignCenter)
        root.addWidget(title)

        # Selector de articulacion
        sel_row = QHBoxLayout()
        lbl_sel = QLabel("Articulacion:")
        lbl_sel.setStyleSheet("color:#ff9900; font-weight:bold;")
        sel_row.addWidget(lbl_sel)
        self.joint_combo = QComboBox()
        for j in range(1, 7):
            self.joint_combo.addItem(f"Articulacion {j}  (nodo {j})")
        self.joint_combo.setFixedWidth(132)
        self.joint_combo.currentIndexChanged.connect(self._reload_config)
        sel_row.addWidget(self.joint_combo)
        sel_row.addStretch()
        root.addLayout(sel_row)

        # Separador
        sep = QFrame(); sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet("background:#ff9900; max-height:1px;")
        root.addWidget(sep)

        # Grid de parametros — 2 columnas
        grid = QGridLayout()
        grid.setHorizontalSpacing(24)
        grid.setVerticalSpacing(8)
        self._spin_boxes = {}

        for idx, (label_txt, key, default) in enumerate(self.PARAMS):
            col = (idx % 2) * 2   # columnas 0 o 2
            row = idx // 2

            lbl = QLabel(label_txt)
            lbl.setStyleSheet("color:#e0e0e0; font-size:9px; font-weight:normal;")

            spin = QDoubleSpinBox()
            spin.setDecimals(4)
            spin.setRange(-9999.0, 9999.0)
            spin.setSingleStep(0.1)
            spin.setValue(default)
            spin.setFixedWidth(78)
            spin.setStyleSheet(
                "QDoubleSpinBox { background:#2b2e38; border:1.2px solid #ff9900;"
                " border-radius:5px; color:#e0e0e0; padding:3px; }"
                "QDoubleSpinBox::up-button, QDoubleSpinBox::down-button"
                " { width:18px; background:#31364a; border-radius:3px; }"
            )
            if key in ('limit_inf_motor', 'limit_sup_motor'):
                tip = ('Zona prohibida para la cinemática inversa (IK): estos mismos '
                       'valores son los que descarta ik_dls() al buscar solución, y los '
                       'que "Leer desde ESP32"/"Enviar configuración" sincronizan '
                       'automáticamente con el firmware de este nodo. Iguales entre sí '
                       '= sin zona prohibida.')
                spin.setToolTip(tip)
                lbl.setToolTip(tip)
            self._spin_boxes[key] = spin
            grid.addWidget(lbl,  row, col)
            grid.addWidget(spin, row, col + 1)

        root.addLayout(grid)

        zona_hint = QLabel(
            '⚠ Los límites inferior/superior definen también la zona prohibida que '
            'usa la cinemática inversa (IK) del brazo — se sincronizan solos al leer '
            'o enviar la configuración de este nodo.')
        zona_hint.setWordWrap(True)
        zona_hint.setStyleSheet('color:#ffcc66; font-size:8.5px; font-style:italic;')
        root.addWidget(zona_hint)

        # Separador
        sep2 = QFrame(); sep2.setFrameShape(QFrame.HLine)
        sep2.setStyleSheet("background:#ff9900; max-height:1px;")
        root.addWidget(sep2)

        # Fila de botones + estado
        btn_row = QHBoxLayout()
        self._status_lbl = QLabel("")
        self._status_lbl.setStyleSheet("font-size:9px; font-weight:bold;")
        btn_row.addWidget(self._status_lbl)
        btn_row.addStretch()

        btn_reload = QPushButton("Leer desde ESP32")
        btn_reload.setStyleSheet(
            "border:1.5px solid #44bbff; color:#44bbff; font-weight:bold;"
            " border-radius:8px; padding:6px 16px;"
        )
        btn_reload.clicked.connect(self._reload_config)
        btn_row.addWidget(btn_reload)

        btn_send = QPushButton("Enviar configuracion")
        btn_send.setStyleSheet(
            "border:1.5px solid #ff9900; color:#ff9900; font-weight:bold;"
            " border-radius:8px; padding:6px 16px;"
        )
        btn_send.clicked.connect(self._send_config)
        btn_row.addWidget(btn_send)

        btn_zone = QPushButton("Aplicar zona a IK")
        btn_zone.setToolTip(
            'Actualiza la zona prohibida de esta articulación en la cinemática '
            'inversa con los valores de Límite inferior/superior de arriba, sin '
            'necesidad de conexión serie (útil para editar rutinas sin el robot '
            'conectado).')
        btn_zone.setStyleSheet(
            "border:1.5px solid #66cc88; color:#66cc88; font-weight:bold;"
            " border-radius:8px; padding:6px 16px;"
        )
        btn_zone.clicked.connect(self._apply_zone_only)
        btn_row.addWidget(btn_zone)

        self._auto_chk = QCheckBox("Auto-enviar")
        self._auto_chk.setStyleSheet("color:#ff9900; font-weight:bold;")
        self._auto_chk.setToolTip(
            "Envía automáticamente al ESP32 cuando cambias un valor")
        btn_row.addWidget(self._auto_chk)

        btn_close = QPushButton("Cerrar")
        btn_close.setStyleSheet(
            "border:1.5px solid #888; color:#888; border-radius:8px; padding:6px 14px;"
        )
        btn_close.clicked.connect(self.reject)
        btn_row.addWidget(btn_close)

        root.addLayout(btn_row)

        # ── Gráfica de respuesta ──────────────────────────────────────────
        sep3 = QFrame(); sep3.setFrameShape(QFrame.HLine)
        sep3.setStyleSheet("background:#333; max-height:1px;")
        root.addWidget(sep3)
        lbl_graph = QLabel("Respuesta articulación seleccionada")
        lbl_graph.setStyleSheet(
            "font-size:9px; color:#aaa; background:transparent;")
        root.addWidget(lbl_graph)
        self._resp_graph = ResponseGraph(self)
        root.addWidget(self._resp_graph)

    # ── Envio al ESP32 ─────────────────────────────────────────────────────

    def _send_config(self):
        sp = self._main.serial_port
        if not (sp and sp.is_open):
            self._set_status("Sin conexion serie activa", error=True)
            return

        joint_idx = self.joint_combo.currentIndex()   # 0-based

        # Orden exacto que espera procesarComandoInit en el ESP32
        orden = ["kp", "ki", "kd",
                 "max_vel", "cruise_vel", "approach_vel", "min_vel",
                 "tol_deg", "slow_zone_deg", "approach_zone_deg", "limit_inf_motor", "limit_sup_motor"]
        values = [self._spin_boxes[k].value() for k in orden]

        # Formato: init[i]val0;val1;...val9;\n  (i=0-based, sin '=' tras ']')
        params_str = ";".join(f"{v:.4f}" for v in values) + ";"
        cmd = f"init[{joint_idx + 1}]{params_str}\n"

        # Mantener la IK (JOINT_FORBIDDEN_ZONES) sincronizada con lo que se
        # manda al firmware — ver comentario junto a set_forbidden_zone().
        set_forbidden_zone(joint_idx,
                            self._spin_boxes['limit_inf_motor'].value(),
                            self._spin_boxes['limit_sup_motor'].value())

        try:
            sp.write(cmd.encode("ascii"))
            self._set_status(f"Enviado a Articulacion {joint_idx + 1}  OK", error=False)
            print(f"[NodeConfig] {cmd.strip()}")
        except Exception as e:
            self._set_status(f"Error: {e}", error=True)
            print(f"[NodeConfig] Error al enviar: {e}")

    def _apply_zone_only(self):
        """Aplica la zona prohibida a la IK sin necesidad de conexión serie
        (a diferencia de _send_config, que requiere el puerto abierto)."""
        joint_idx = self.joint_combo.currentIndex()
        lo = self._spin_boxes['limit_inf_motor'].value()
        hi = self._spin_boxes['limit_sup_motor'].value()
        set_forbidden_zone(joint_idx, lo, hi)
        if lo == hi:
            self._set_status(f"Articulación {joint_idx + 1}: sin zona prohibida (IK)", error=False)
        else:
            self._set_status(
                f"Articulación {joint_idx + 1}: zona prohibida IK = {lo:.1f}°–{hi:.1f}°",
                error=False)

    def _reload_config(self):
        """Solicita al ESP32 la configuracion actual de un nodo y rellena los campos.

        El ESP32 responde (entre otras lineas de angulos) con:
            reload[1]=3.0;0.2;0.0;2000.0;1000.0;600.0;20.0;0.5;5.0;15.0;
        Como el ESP32 sigue emitiendo angulos[...] cada ~100 ms, hay que leer
        en bucle hasta encontrar la linea que empiece por 'reload['.
        """
        sp = self._main.serial_port
        if not (sp and sp.is_open):
            self._set_status("Sin conexion serie activa", error=True)
            return

        joint_idx = self.joint_combo.currentIndex()   # 0-based en Python
        cmd = f"reload[{joint_idx + 1}]\n"            # 1-based para el ESP32

        # Orden exacto en que el ESP32 construye el mensaje
        orden = ["kp", "ki", "kd",
                 "max_vel", "cruise_vel", "approach_vel", "min_vel",
                 "tol_deg", "slow_zone_deg", "approach_zone_deg", "limit_inf_motor", "limit_sup_motor"]

        try:
            # 1) Pausar el timer de lectura continua para no competir por el buffer
            self._main._serial_rx_timer.stop()

            # 2) Enviar solicitud
            sp.write(cmd.encode("ascii"))
            self._set_status(
                f"Esperando respuesta de Articulacion {joint_idx + 1}...", error=False
            )
            print(f"[NodeReload TX] {cmd.strip()}")

            # 3) Leer líneas hasta encontrar la que empieza por 'reload['
            #    Máximo 30 intentos × timeout 1 s = 30 s de espera máxima
            reload_line = None
            for intento in range(30):
                raw  = sp.readline()
                line = raw.decode("ascii", errors="ignore").strip()
                print(f"[NodeReload RX #{intento}] '{line}'")
                if line.startswith("reload["):
                    reload_line = line
                    break

            if reload_line is None:
                self._set_status("Sin respuesta reload del ESP32 (timeout)", error=True)
                return

            # 4) Parsear formato: reload[i]=v0;v1;v2;...v9;
            params_str = reload_line.split("=", 1)[1]
            tokens = [t.strip() for t in params_str.split(";") if t.strip()]

            if len(tokens) < len(orden):
                self._set_status(
                    f"Respuesta incompleta ({len(tokens)}/{len(orden)} parametros)",
                    error=True
                )
                return

            # 5) Actualizar cada spin-box con su valor posicional
            for i, key in enumerate(orden):
                try:
                    self._spin_boxes[key].setValue(float(tokens[i]))
                except (ValueError, KeyError):
                    print(f"[NodeReload] Error al parsear '{key}' = '{tokens[i]}'")

            # Sincronizar la IK con los límites REALES que acaba de reportar
            # el firmware (ver comentario junto a set_forbidden_zone()).
            set_forbidden_zone(joint_idx,
                                self._spin_boxes['limit_inf_motor'].value(),
                                self._spin_boxes['limit_sup_motor'].value())

            self._set_status(
                f"Articulacion {joint_idx + 1} cargada correctamente  OK", error=False
            )

        except Exception as e:
            self._set_status(f"Error: {e}", error=True)
            print(f"[NodeReload] Error: {e}")

        finally:
            # 6) Reanudar siempre el timer, tanto si hubo éxito como si no
            self._main._serial_rx_timer.start()




    def _on_spin_changed(self):
        if self._auto_chk.isChecked():
            self._debounce.start()

    def _auto_send(self):
        if self._auto_chk.isChecked():
            self._send_config()

    def _sample_pid_data(self):
        idx = self.joint_combo.currentIndex()
        fb  = self._main.real_angles_feedback
        sp  = self._main._joint_setpoints
        if fb and sp and idx < len(fb) and idx < len(sp):
            self._resp_graph.push(sp[idx], fb[idx])

    def _set_status(self, msg, error=False):
        color = "#ff6b6b" if error else "#70e570"
        self._status_lbl.setStyleSheet(
            f"font-size:9px; font-weight:bold; color:{color};"
        )
        self._status_lbl.setText(msg)

    # ── Tema oscuro ────────────────────────────────────────────────────────

    def _theme(self):
        return """
        QDialog     { background:#23262b; color:#e0e0e0;
                      font-family:'Segoe UI',Arial; font-size:10px; }
        QLabel      { color:#93ffa8; font-weight:600; }
        QComboBox   { background:#2b2e38; border:1.2px solid #ff9900;
                      border-radius:5px; color:#e0e0e0; padding:4px; }
        QPushButton { background-color:#31364a; border:1.2px solid #5dd095;
                      border-radius:8px; padding:5px 10px;
                      color:#70e570; font-weight:bold; }
        QPushButton:hover { background-color:#70e570; color:#23262b; }
        QDoubleSpinBox { background:#2b2e38; border:1.2px solid #ff9900;
                         border-radius:5px; color:#e0e0e0; padding:3px; }
        """


class CintaConfigDialog(QDialog):
    """Ventana pequeña para configurar la cinta transportadora: distancia
    umbral de detección del HC-SR04 (enviada por Serial como "cintadist=N.N",
    reenviada por el central a la cinta por ESP-NOW) y una lectura en vivo
    del estado que reporta la propia cinta (objeto detectado / distancia
    medida, ver BrazoRobot._parse_cinta_estado()). El arranque/paro con
    velocidad se maneja desde el bloque 'Cinta transportadora' del editor de
    rutinas, no desde aquí -- esto es solo configuración + monitor."""

    def __init__(self, main_window):
        super().__init__(main_window)
        self._main = main_window
        self.setWindowTitle("Cinta transportadora")
        self.setMinimumWidth(260)
        self.setStyleSheet("""
            QDialog     { background:#23262b; color:#e0e0e0;
                          font-family:'Segoe UI',Arial; font-size:10px; }
            QLabel      { color:#93ffa8; font-weight:600; }
            QPushButton { background-color:#31364a; border:1.2px solid #00b894;
                          border-radius:8px; padding:5px 10px;
                          color:#00e0ac; font-weight:bold; }
            QPushButton:hover { background-color:#00b894; color:#23262b; }
            QDoubleSpinBox { background:#2b2e38; border:1.2px solid #00b894;
                             border-radius:5px; color:#e0e0e0; padding:3px; }
        """)
        from PyQt5.QtWidgets import QDoubleSpinBox
        root = QVBoxLayout(self)
        root.setSpacing(8)
        root.setContentsMargins(12, 12, 12, 12)

        title = QLabel("CINTA TRANSPORTADORA")
        title.setStyleSheet("font-size:12px; font-weight:bold; color:#00e0ac; padding:4px;")
        title.setAlignment(Qt.AlignCenter)
        root.addWidget(title)

        dist_row = QHBoxLayout()
        dist_row.addWidget(QLabel("Distancia de detección (cm):"))
        self.dist_spin = QDoubleSpinBox()
        self.dist_spin.setRange(1.0, 100.0)
        self.dist_spin.setSingleStep(0.5)
        self.dist_spin.setDecimals(1)
        self.dist_spin.setValue(getattr(self._main, 'cinta_distancia_umbral_cm', 4.0))
        self.dist_spin.setFixedWidth(74)
        self.dist_spin.setToolTip(
            'Si el HC-SR04 mide una pieza a esta distancia o menos, la cinta se '
            'para automáticamente (hasta que la pieza se aleje).')
        dist_row.addWidget(self.dist_spin)
        dist_row.addStretch()
        root.addLayout(dist_row)

        btn_send = QPushButton("Enviar distancia")
        btn_send.clicked.connect(self._enviar_distancia)
        root.addWidget(btn_send)

        # Estado en vivo, refrescado cada 300 ms desde lo último que
        # reportó el central por Serial (objetoCinta=/distCinta=).
        self.lbl_estado = QLabel("Objeto: —")
        self.lbl_estado.setStyleSheet("color:#e0e0e0; font-weight:normal;")
        root.addWidget(self.lbl_estado)
        self.lbl_dist_medida = QLabel("Distancia medida: —")
        self.lbl_dist_medida.setStyleSheet("color:#e0e0e0; font-weight:normal;")
        root.addWidget(self.lbl_dist_medida)

        self._refresh_timer = QTimer(self)
        self._refresh_timer.setInterval(300)
        self._refresh_timer.timeout.connect(self._refresh_estado)
        self._refresh_timer.start()
        self.finished.connect(self._refresh_timer.stop)
        self._refresh_estado()

    def _enviar_distancia(self):
        self._main.send_cinta_distancia(self.dist_spin.value())

    def _refresh_estado(self):
        det = getattr(self._main, 'cinta_objeto_detectado', False)
        d = getattr(self._main, 'cinta_distancia_cm', -1.0)
        self.lbl_estado.setText(f"Objeto: {'🟢 DETECTADO' if det else '⚪ no detectado'}")
        self.lbl_dist_medida.setText(
            "Distancia medida: —" if d < 0 else f"Distancia medida: {d:.1f} cm")


class ToolProfileDialog(QDialog):
    """Gestion de perfiles de herramienta (offset TCP) -- pinza, ventosa, etc."""

    def __init__(self, main_window):
        super().__init__(main_window)
        self._main = main_window
        self.setWindowTitle("Perfiles de Herramienta (TCP)")
        self.setMinimumWidth(216)
        self.setStyleSheet(
            "QDialog { background:#1c1f24; color:#ddd; }"
            "QLabel { color:#ccc; }"
            "QLineEdit { background:#0d1014; color:#e0e0e0; border:1px solid #3a3f4a;"
            " border-radius:3px; padding:3px; }"
            "QComboBox { background:#0d1014; color:#ddd; border:1px solid #3a3f4a;"
            " border-radius:3px; padding:3px; }"
            "QPushButton { border:1px solid #26C6DA; color:#26C6DA; background:transparent;"
            " border-radius:4px; padding:4px 10px; }"
            "QPushButton:hover { background:#0d2a3d; }"
        )
        vl = QVBoxLayout(self)

        hint = QLabel(
            "El offset se aplica desde el flange (extremo del robot) hasta la\n"
            "punta real de la herramienta. Util al cambiar pinza/ventosa/otra.")
        hint.setStyleSheet("color:#888; font-size:8px;")
        vl.addWidget(hint)

        row1 = QHBoxLayout()
        row1.addWidget(QLabel("Perfil:"))
        self._combo = QComboBox()
        self._combo.addItems(sorted(self._main._tool_profiles.keys()))
        idx = self._combo.findText(self._main._active_tool_profile)
        if idx >= 0:
            self._combo.setCurrentIndex(idx)
        row1.addWidget(self._combo, 1)
        btn_new = QPushButton("+ Nuevo")
        btn_del = QPushButton("X Eliminar")
        row1.addWidget(btn_new)
        row1.addWidget(btn_del)
        vl.addLayout(row1)

        grid = QGridLayout()
        self._fields = {}
        labels = [('dx', 'X offset (mm)'), ('dy', 'Y offset (mm)'), ('dz', 'Z offset (mm)'),
                  ('roll', 'Roll (grados)'), ('pitch', 'Pitch (grados)'), ('yaw', 'Yaw (grados)')]
        for i, (key, label) in enumerate(labels):
            grid.addWidget(QLabel(label), i, 0)
            e = QLineEdit('0')
            grid.addWidget(e, i, 1)
            self._fields[key] = e
        vl.addLayout(grid)

        self._combo.currentTextChanged.connect(self._load_fields)
        self._load_fields(self._combo.currentText())

        btn_new.clicked.connect(self._new_profile)
        btn_del.clicked.connect(self._delete_profile)

        row2 = QHBoxLayout()
        btn_apply = QPushButton("OK Aplicar y guardar")
        btn_apply.setStyleSheet(
            "border-color:#70e570; color:#70e570; font-weight:bold;")
        btn_apply.clicked.connect(self._apply_and_save)
        btn_close = QPushButton("Cerrar")
        btn_close.clicked.connect(self.accept)
        row2.addWidget(btn_apply)
        row2.addStretch()
        row2.addWidget(btn_close)
        vl.addLayout(row2)

    def _load_fields(self, name):
        p = self._main._tool_profiles.get(name, {})
        for key, e in self._fields.items():
            e.setText(str(p.get(key, 0)))

    def _new_profile(self):
        from PyQt5.QtWidgets import QInputDialog
        name, ok = QInputDialog.getText(self, "Nuevo perfil", "Nombre del perfil:")
        name = name.strip()
        if not ok or not name:
            return
        if name in self._main._tool_profiles:
            QMessageBox.warning(self, "Ya existe", "Ya hay un perfil con ese nombre.")
            return
        self._main._tool_profiles[name] = {'dx': 0, 'dy': 0, 'dz': 0,
                                            'roll': 0, 'pitch': 0, 'yaw': 0}
        self._combo.addItem(name)
        self._combo.setCurrentText(name)

    def _delete_profile(self):
        name = self._combo.currentText()
        if name == 'Sin offset':
            QMessageBox.warning(self, "No permitido",
                                 "El perfil 'Sin offset' no se puede eliminar.")
            return
        if name not in self._main._tool_profiles:
            return
        self._main._tool_profiles.pop(name, None)
        idx = self._combo.findText(name)
        if idx >= 0:
            self._combo.removeItem(idx)

    def _apply_and_save(self):
        name = self._combo.currentText()
        try:
            vals = {k: float(e.text() or 0) for k, e in self._fields.items()}
        except ValueError:
            QMessageBox.warning(self, "Valor invalido", "Revisa los campos numericos.")
            return
        self._main._tool_profiles[name] = vals
        self._main._active_tool_profile = name
        self._main._apply_active_tool_profile()
        self._main._save_tool_profiles()
        self._main.update_xyz_display()
        self._main.update_3d_visualization()
        QMessageBox.information(self, "Aplicado",
                                 f"Perfil '{name}' aplicado y guardado.")
class LoadingSignals(QObject):
    progress_updated = pyqtSignal(int)
    message_updated  = pyqtSignal(str)


class CalibrationWizard(QDialog):
    """Asistente paso a paso para calibrar el offset de cada articulación."""

    def __init__(self, robot):
        super().__init__(robot)
        self.robot = robot
        self.step  = 0          # 0-5 = joints, 6 = summary
        self.setWindowTitle('🔧 Asistente de Calibración')
        self.setMinimumWidth(288)
        self.setStyleSheet(robot.dark_theme())
        self._pending_offsets = list(robot._joint_offsets)
        self._build_ui()
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh_live)
        self._timer.start(150)
        self._refresh_step()

    def _build_ui(self):
        vlay = QVBoxLayout(self)
        vlay.setSpacing(6)

        # Progress bar
        from PyQt5.QtWidgets import QProgressBar
        self._progress = QProgressBar()
        self._progress.setRange(0, 6)
        self._progress.setValue(0)
        self._progress.setTextVisible(True)
        self._progress.setStyleSheet(
            'QProgressBar { border:1px solid #446644; border-radius:4px; background:#0d1a0d; color:#70e570; }'
            'QProgressBar::chunk { background:#336633; border-radius:3px; }')
        vlay.addWidget(self._progress)

        # Step title
        self._title_lbl = QLabel()
        self._title_lbl.setStyleSheet('font-size:11px; font-weight:bold; color:#88dd88;')
        vlay.addWidget(self._title_lbl)

        # Instructions
        self._instr_lbl = QLabel()
        self._instr_lbl.setWordWrap(True)
        self._instr_lbl.setStyleSheet('color:#aaccaa; font-size:9px;')
        vlay.addWidget(self._instr_lbl)

        # Live angle display
        self._live_lbl = QLabel('─')
        self._live_lbl.setStyleSheet(
            'font-size:17px; font-weight:bold; color:#ffcc44;'
            ' background:#0d1a0d; border:1px solid #336633;'
            ' border-radius:4px; padding:6px 12px;')
        self._live_lbl.setAlignment(Qt.AlignCenter)
        vlay.addWidget(self._live_lbl)

        # Current offset display
        self._offset_lbl = QLabel()
        self._offset_lbl.setStyleSheet('color:#556655; font-size:9px;')
        self._offset_lbl.setAlignment(Qt.AlignCenter)
        vlay.addWidget(self._offset_lbl)

        # Set zero button
        self._set_zero_btn = QPushButton('📍 Fijar como 0° (usar lectura actual)')
        self._set_zero_btn.setStyleSheet(
            'QPushButton { border:2px solid #44aa44; color:#44ff44;'
            ' background:#0d1a0d; border-radius:6px; padding:6px; font-weight:bold; }'
            'QPushButton:hover { background:#1a3a1a; }')
        self._set_zero_btn.clicked.connect(self._set_zero)
        vlay.addWidget(self._set_zero_btn)

        # Reset offset button
        btn_reset = QPushButton('↺ Resetear offset a 0')
        btn_reset.setStyleSheet(
            'QPushButton { border:1px solid #446644; color:#669966;'
            ' background:transparent; border-radius:4px; padding:4px; }'
            'QPushButton:hover { background:#1a2a1a; }')
        btn_reset.clicked.connect(self._reset_offset)
        vlay.addWidget(btn_reset)

        # Navigation
        nav = QHBoxLayout()
        self._prev_btn = QPushButton('◀ Anterior')
        self._prev_btn.clicked.connect(self._prev_step)
        self._next_btn = QPushButton('Siguiente ▶')
        self._next_btn.setStyleSheet(
            'QPushButton { border:1px solid #44aa44; color:#44cc44;'
            ' background:#0d1a0d; border-radius:5px; padding:4px 10px; }')
        self._next_btn.clicked.connect(self._next_step)
        self._save_btn = QPushButton('💾 Guardar y cerrar')
        self._save_btn.setStyleSheet(
            'QPushButton { border:2px solid #44aa44; color:#44ff44;'
            ' background:#0d1a0d; border-radius:5px; padding:4px 10px; font-weight:bold; }')
        self._save_btn.clicked.connect(self._save_and_close)
        self._save_btn.setVisible(False)
        nav.addWidget(self._prev_btn)
        nav.addStretch()
        nav.addWidget(self._next_btn)
        nav.addWidget(self._save_btn)
        vlay.addLayout(nav)

    def _refresh_step(self):
        self._progress.setValue(self.step)
        is_summary = (self.step == 6)
        self._set_zero_btn.setVisible(not is_summary)
        self._live_lbl.setVisible(not is_summary)
        self._next_btn.setVisible(not is_summary)
        self._save_btn.setVisible(is_summary)
        self._prev_btn.setEnabled(self.step > 0)

        if is_summary:
            self._title_lbl.setText('✅ Resumen de calibración')
            lines = []
            for i, off in enumerate(self._pending_offsets):
                lines.append(f'  J{i+1}:  offset = {off:+.2f}°')
            self._instr_lbl.setText(
                'Offsets a aplicar:\n' + '\n'.join(lines) +
                '\n\nPulsa "Guardar y cerrar" para aplicar y persistir.')
            self._offset_lbl.setText('')
        else:
            j = self.step + 1
            self._title_lbl.setText(f'Articulación {j}  (J{j})')
            self._instr_lbl.setText(
                f'1. Mueve físicamente J{j} a su posición de 0° (home mecánico).\n'
                f'2. Brazo conectado por serial para ver la lectura.\n'
                f'3. Pulsa "Fijar como 0" cuando el encoder esté en posición.')
            self._offset_lbl.setText(
                f'Offset actual J{j}: {self._pending_offsets[self.step]:+.2f}°')

    def _refresh_live(self):
        if self.step >= 6:
            return
        raw = self.robot._joint_raw_feedback[self.step]
        cal = raw - self._pending_offsets[self.step]
        self._live_lbl.setText(
            f'J{self.step+1}  raw: {raw:.2f}°   →   calibrado: {cal:.2f}°')

    def _set_zero(self):
        if self.step >= 6:
            return
        raw = self.robot._joint_raw_feedback[self.step]
        self._pending_offsets[self.step] = raw
        self._offset_lbl.setText(
            f'Offset J{self.step+1} fijado: {raw:+.2f}°  ✓')
        self._offset_lbl.setStyleSheet('color:#44ff44; font-size:9px;')

    def _reset_offset(self):
        if self.step >= 6:
            return
        self._pending_offsets[self.step] = 0.0
        self._offset_lbl.setText(f'Offset J{self.step+1}: 0.00° (reseteado)')
        self._offset_lbl.setStyleSheet('color:#556655; font-size:9px;')

    def _prev_step(self):
        if self.step > 0:
            self.step -= 1
            self._refresh_step()

    def _next_step(self):
        if self.step < 6:
            self.step += 1
            self._refresh_step()

    def _save_and_close(self):
        self._timer.stop()
        for i, off in enumerate(self._pending_offsets):
            self.robot._joint_offsets[i] = off
        self.robot._save_joint_offsets()
        self.accept()

    def closeEvent(self, event):
        self._timer.stop()
        super().closeEvent(event)
