# -*- coding: utf-8 -*-
"""Comunicación serie con el nodo Central: escaneo/conexión de puertos, envío de setpoints y comandos, y parseo de los reportes periódicos (ángulos, mando, cinta)."""
import os
import json
import numpy as np
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

from app_paths import SERIAL_AVAILABLE
try:
    import serial
    import serial.tools.list_ports
except ImportError:
    pass  # ya cubierto por SERIAL_AVAILABLE, ver app_paths.py
from kinematics import _wrap360
from dialogs import NodeConfigDialog, CintaConfigDialog


class SerialCommMixin:
    """Ver Comunicación serie con el nodo Central: escaneo/conexión de puertos, envío de setpoints y comandos, y parseo de los reportes periódicos (ángulos, mando, cinta)."""

    def _build_serial_panel(self):
        frame = QFrame()
        frame.setStyleSheet(
            "QFrame { border: 2px solid #4488ff; border-radius: 5px; }"
        )
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(4, 2, 4, 2)
        layout.setSpacing(2)

        title = QLabel("COMUNICACION SERIE — ESP32")
        title.setStyleSheet(
            "font-size:10px; font-weight:bold; color:#4488ff; padding:2px;"
        )
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        row = QHBoxLayout()
        row.setSpacing(3)

        self.serial_led = QLabel()
        self.serial_led.setFixedSize(8, 8)
        self.serial_led.setStyleSheet("background:#ff6b6b; border-radius:7px;")
        row.addWidget(self.serial_led)

        self.serial_status_lbl = QLabel("Desconectado")
        self.serial_status_lbl.setStyleSheet("color:#ff6b6b; font-weight:bold;")
        row.addWidget(self.serial_status_lbl)

        row.addSpacing(10)

        self.com_combo = QComboBox()
        self.com_combo.setFixedWidth(66)
        row.addWidget(self.com_combo)

        self.baud_combo = QComboBox()
        for br in ["9600", "19200", "38400", "57600", "115200", "230400"]:
            self.baud_combo.addItem(br)
        self.baud_combo.setCurrentText("115200")
        self.baud_combo.setFixedWidth(51)
        row.addWidget(self.baud_combo)

        btn_scan = QPushButton("Escanear")
        btn_scan.setStyleSheet("border-color:#4488ff; color:#4488ff;")
        btn_scan.clicked.connect(self.scan_com_ports)
        row.addWidget(btn_scan)

        self.btn_connect = QPushButton("Conectar")
        self.btn_connect.setStyleSheet("border-color:#4488ff; color:#4488ff;")
        self.btn_connect.clicked.connect(self.toggle_serial)
        row.addWidget(self.btn_connect)

        btn_cfg = QPushButton("Configurar nodos")
        btn_cfg.setStyleSheet("border-color:#ff9900; color:#ff9900;")
        btn_cfg.clicked.connect(self.open_node_config)
        row.addWidget(btn_cfg)

        row.addStretch()
        layout.addLayout(row)

        # ── Fila 2: Backup / Restaurar / Calibrar ────────────────────
        row2 = QHBoxLayout()
        row2.setSpacing(3)
        btn_backup = QPushButton('💾 Backup')
        btn_backup.setToolTip('Guardar configuración completa')
        btn_backup.setStyleSheet('border-color:#44cc88; color:#44cc88;')
        btn_backup.clicked.connect(self._backup_config)
        row2.addWidget(btn_backup)

        btn_restore = QPushButton('📂 Restaurar')
        btn_restore.setToolTip('Restaurar configuración desde backup')
        btn_restore.setStyleSheet('border-color:#44cc88; color:#44cc88;')
        btn_restore.clicked.connect(self._restore_config)
        row2.addWidget(btn_restore)

        btn_cal = QPushButton('🔧 Calibrar')
        btn_cal.setToolTip('Asistente de calibración de offsets por articulación')
        btn_cal.setStyleSheet('border-color:#cc8844; color:#cc8844;')
        btn_cal.clicked.connect(self._open_calibration_wizard)
        row2.addWidget(btn_cal)

        btn_rst = QPushButton("Reset todos a 0°")
        btn_rst.setStyleSheet("border-color:#f0a500; color:#f0a500;")
        btn_rst.clicked.connect(self.reset_all)
        row2.addWidget(btn_rst)

        btn_cinta = QPushButton('📦 Cinta')
        btn_cinta.setToolTip('Configurar distancia de detección de la cinta transportadora')
        btn_cinta.setStyleSheet('border-color:#00b894; color:#00b894;')
        btn_cinta.clicked.connect(self.open_cinta_config)
        row2.addWidget(btn_cinta)

        row2.addStretch()
        layout.addLayout(row2)

        self.scan_com_ports()
        return frame

    def open_node_config(self):
        dlg = NodeConfigDialog(self)
        dlg.exec_()

    def open_cinta_config(self):
        dlg = CintaConfigDialog(self)
        dlg.exec_()

    def scan_com_ports(self):
        self.com_combo.clear()
        if not SERIAL_AVAILABLE:
            self.com_combo.addItem("pyserial no instalado")
            return
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.com_combo.addItem(p.device)
        if self.com_combo.count() == 0:
            self.com_combo.addItem("Sin puertos")

    def toggle_serial(self):
        if self.serial_port and self.serial_port.is_open:
            self._disconnect_serial()
        else:
            self._connect_serial()

    def _connect_serial(self):
        if not SERIAL_AVAILABLE:
            QMessageBox.warning(
                self, "pyserial no instalado",
                "Instala pyserial:\n  pip install pyserial"
            )
            return
        port = self.com_combo.currentText()
        if not port or port in ("Sin puertos", "pyserial no instalado"):
            QMessageBox.warning(self, "Puerto invalido", "Selecciona un puerto COM valido.")
            return
        baud = int(self.baud_combo.currentText())
        try:
            self.serial_port = serial.Serial(port, baud, timeout=1)
            self._set_serial_status(True, port)
        except Exception as e:
            QMessageBox.critical(self, "Error al conectar", str(e))
            self._disconnect_serial()

    def _disconnect_serial(self):
        self._log_error("Conexión serial cerrada / perdida", "SERIAL")
        if self.serial_port:
            try:
                self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None
        self._set_serial_status(False)

    def _set_serial_status(self, connected, port=""):
        if connected:
            self.serial_led.setStyleSheet("background:#70e570; border-radius:7px;")
            self.serial_status_lbl.setStyleSheet("color:#70e570; font-weight:bold;")
            self.serial_status_lbl.setText(f"Conectado — {port}")
            self.btn_connect.setText("Desconectar")
        else:
            self.serial_led.setStyleSheet("background:#ff6b6b; border-radius:7px;")
            self.serial_status_lbl.setStyleSheet("color:#ff6b6b; font-weight:bold;")
            self.serial_status_lbl.setText("Desconectado")
            self.btn_connect.setText("Conectar")
    def send_setpoints(self, angles, vel_pct=100.0):
        self._joint_setpoints = [a % 360 for a in angles]  # para gráfica PID (0-360, espacio calibrado)
        if getattr(self, '_dry_run_mode', False):
            return  # modo simulación: no enviar nada al hardware
        if not (self.serial_port and self.serial_port.is_open):
            return
        # "angles" (self.angles) vive en espacio CALIBRADO: cinemática, IK y
        # pantalla usan angulo_calibrado = angulo_raw - _joint_offsets[i] (ver
        # _parse_angulos). El firmware, en cambio, solo conoce su propio
        # angulo_raw (lo que reporta como "angulo[i]=..." y contra lo que
        # compara el setpoint). Si aquí se manda el valor calibrado tal cual,
        # el brazo se para sistemáticamente _joint_offsets[i]° desplazado del
        # objetivo que se ve en pantalla — justo el motivo de que "no se
        # acerque del todo" a las coordenadas indicadas en cuanto hay una
        # calibración (offsets != 0) guardada. Se compensa sumando el offset
        # de vuelta antes de enviar (con offsets todos a 0 no cambia nada).
        # Formato: SP[1]=10.00;SP[2]=20.00;...SP[6]=60.00;V=75;\n
        # "V=NN" es el % de velocidad (10-100) para este movimiento — el
        # central lo reenvía a las 6 articulaciones junto al setpoint, y
        # cada una escala su perfil de velocidad por ese factor. Se manda
        # siempre explícito (incluso a 100) para no depender de que el
        # central conserve el valor de un envío anterior.
        parts = [f"SP[{i+1}]={(angles[i] + self._joint_offsets[i]) % 360:.2f}" for i in range(6)]
        vel_pct = max(10.0, min(100.0, float(vel_pct)))
        parts.append(f"V={vel_pct:.0f}")
        cmd = ";".join(parts) + ";\n"
        try:
            self.serial_port.write(cmd.encode("ascii"))
            print(cmd)
        except Exception as e:
            self._log_error(f"Serial send_setpoints: {e}", "SERIAL")
            self._disconnect_serial()

    def _read_serial_angles(self):
        """Lee líneas pendientes del ESP32 y actualiza las etiquetas de ángulo real."""
        if not (self.serial_port and self.serial_port.is_open):
            return
        try:
            while self.serial_port.in_waiting:
                raw = self.serial_port.readline()
                line = raw.decode("ascii", errors="ignore").strip()
                if not line:
                    continue
                # Formato: angulo[1]=10.50;angulo[2]=20.30;...angulo[6]=60.10;
                # Formato: spmando[1]=45.00;  (setpoint enviado por el mando fisico)
                print(f"[Serial RX] {line}")
                self._parse_angulos(line)
                self._parse_sp_mando(line)
                self._parse_cinta_estado(line)
        except Exception as e:
            print(f"[Serial RX] Error: {e}")
            self._disconnect_serial()

    def _parse_angulos(self, line):
        """Parsea 'angulo[i]=val;' y actualiza real_angle_labels (i es 1-based)."""
        import re
        for m in re.finditer(r"angulo\[(\d+)\]=([-\d.]+)", line):
            idx = int(m.group(1)) - 1   # 0-based
            if 0 <= idx < 6:
                val = float(m.group(2))
                self._joint_raw_feedback[idx] = val
                val_cal = val - self._joint_offsets[idx]
                self.real_angle_labels[idx].setText(f"Real: {val_cal:.2f}°")
                self.real_angles_feedback[idx] = val_cal   # para el motor de rutinas

    def _parse_sp_mando(self, line):
        """Parsea 'spmando[i]=val;' — setpoint que el mando fisico acaba de
        mandar directamente al central (sin pasar por la app). El central lo
        reenvia al PC en cuanto llega. Sin esto, la app solo se enteraba del
        ANGULO REAL (via _parse_angulos, cada ~100 ms), pero self.angles
        (el objetivo que usa "Vista App"/la cinemática por defecto) se quedaba
        con el valor viejo — parecía que el mando "no llegaba" a la app.
        Sincroniza self.angles + sliders/inputs/labels + la vista 3D, sin
        volver a enviar el setpoint (ya está aplicado en el hardware)."""
        import re
        actualizado = False
        for m in re.finditer(r"spmando\[(\d+)\]=([-\d.]+)", line):
            idx = int(m.group(1)) - 1   # 0-based
            if 0 <= idx < 6:
                val = _wrap360(float(m.group(2)))
                self.angles[idx] = val
                self.sliders[idx].blockSignals(True)
                self.sliders[idx].setValue(int(val))
                self.sliders[idx].blockSignals(False)
                self.inputs[idx].setText(f"{val:.2f}")
                self.labels[idx].setText(f"Articulación {idx+1}: {val:.2f}°")
                # También el setpoint usado por la gráfica de ajuste PID
                # (Setpoint vs Feedback), que si no, se quedaba con el último
                # valor mandado desde la propia app y no reflejaba lo que
                # acaba de marcar el mando físico.
                if idx < len(self._joint_setpoints):
                    self._joint_setpoints[idx] = val
                actualizado = True
        if actualizado:
            self.update_xyz_display()
            self.update_3d_visualization()

    def _parse_cinta_estado(self, line):
        """Parsea 'objetoCinta=0|1;' y 'distCinta=val;' — estado de la cinta
        transportadora que el central reenvía desde la propia cinta (ver
        struct_message_cinta_estado en ambos firmwares). objeto_detectado lo
        usa la condición "Pieza (cinta)" de las rutinas (_eval_prog_cond)."""
        import re
        m = re.search(r"objetoCinta=([01]);", line)
        if m:
            self.cinta_objeto_detectado = (m.group(1) == '1')
        m = re.search(r"distCinta=(-?[\d.]+);", line)
        if m:
            try:
                self.cinta_distancia_cm = float(m.group(1))
            except ValueError:
                pass

    def send_reset(self, index):
        """Envía reset[i]=0.00 al ESP32 (index 0-based, igual que el firmware)."""
        if not (self.serial_port and self.serial_port.is_open):
            return
        cmd = f"reset[{index}]=0.00\n"
        try:
            self.serial_port.write(cmd.encode("ascii"))
        except Exception as e:
            print(f"[Serial] Error al enviar reset: {e}")
            self._disconnect_serial()

    def send_cinta_cmd(self, arranque, vel_pct=None):
        """Arranca/para la cinta transportadora: 'cinta=1;vel=NN' o 'cinta=0'.
        vel_pct None al parar (el central conserva la última velocidad
        conocida; no hace falta reenviarla)."""
        if self._dry_run_mode or not (self.serial_port and self.serial_port.is_open):
            return
        if arranque:
            v = max(0.0, min(100.0, float(vel_pct if vel_pct is not None else 50)))
            cmd = f"cinta=1;vel={v:.0f}\n"
        else:
            cmd = "cinta=0\n"
        try:
            self.serial_port.write(cmd.encode("ascii"))
        except Exception as e:
            print(f"[Serial] Error al enviar comando de cinta: {e}")
            self._disconnect_serial()

    def send_cinta_distancia(self, distancia_cm):
        """Cambia la distancia umbral de detección de la cinta ('cintadist=N.N'),
        reenviada por el central al ESP32-C3 de la cinta."""
        self.cinta_distancia_umbral_cm = float(distancia_cm)
        if self._dry_run_mode or not (self.serial_port and self.serial_port.is_open):
            return
        cmd = f"cintadist={float(distancia_cm):.1f}\n"
        try:
            self.serial_port.write(cmd.encode("ascii"))
        except Exception as e:
            print(f"[Serial] Error al enviar distancia de la cinta: {e}")
            self._disconnect_serial()
