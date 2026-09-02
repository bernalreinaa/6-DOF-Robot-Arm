# -*- coding: utf-8 -*-
"""Controles manuales: sliders/incrementos por articulación, jog cartesiano y de orientación, home y movimiento a una posición XYZ por cinemática inversa."""
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

from kinematics import T_MDH_TO_FUSION, cinematica_directa, rpy_from_matrix, ik_dls


class ManualControlMixin:
    """Ver Controles manuales: sliders/incrementos por articulación, jog cartesiano y de orientación, home y movimiento a una posición XYZ por cinemática inversa."""

    def setup_manual_controls(self):
        # ── Fila 1: posición XYZ + botón IK + ghost ──────────────────────
        xyz_layout = QHBoxLayout()
        self.x_input = QLineEdit(); self.x_input.setPlaceholderText("X [mm]"); self.x_input.setFixedWidth(42)
        self.y_input = QLineEdit(); self.y_input.setPlaceholderText("Y [mm]"); self.y_input.setFixedWidth(42)
        self.z_input = QLineEdit(); self.z_input.setPlaceholderText("Z [mm]"); self.z_input.setFixedWidth(42)
        xyz_layout.addWidget(QLabel("Posición:"))
        xyz_layout.addWidget(self.x_input)
        xyz_layout.addWidget(self.y_input)
        xyz_layout.addWidget(self.z_input)
        self.move_xyz_btn = QPushButton("Mover a XYZ")
        self.move_xyz_btn.clicked.connect(self.go_to_xyz)
        xyz_layout.addWidget(self.move_xyz_btn)

        self.ghost_btn = QPushButton("🎬 Animar Ghost")
        self.ghost_btn.setStyleSheet(
            "border:1.5px solid #3399ff; color:#3399ff; font-weight:bold;"
            " border-radius:6px; padding:4px 8px;"
        )
        self.ghost_btn.clicked.connect(self.show_ghost)
        xyz_layout.addWidget(self.ghost_btn)

        # Label de estado IK
        self.ik_status_label = QLabel("")
        self.ik_status_label.setStyleSheet("font-size:9px; font-weight:bold;")
        xyz_layout.addWidget(self.ik_status_label)
        xyz_layout.addStretch()
        self.controls_layout.addLayout(xyz_layout)

        # ── Fila 2: orientación RPY (opcional) ───────────────────────────
        from PyQt5.QtWidgets import QCheckBox
        ori_layout = QHBoxLayout()
        self.roll_input  = QLineEdit(); self.roll_input.setPlaceholderText("Roll [°]");  self.roll_input.setFixedWidth(42)
        self.pitch_input = QLineEdit(); self.pitch_input.setPlaceholderText("Pitch [°]"); self.pitch_input.setFixedWidth(42)
        self.yaw_input   = QLineEdit(); self.yaw_input.setPlaceholderText("Yaw [°]");   self.yaw_input.setFixedWidth(42)
        self.ori_check   = QCheckBox("Controlar orientación")
        self.ori_check.setChecked(False)
        self.ori_check.setStyleSheet("color:#93ffa8; font-size:9px;")
        def _toggle_ori(state):
            enabled = bool(state)
            self.roll_input.setEnabled(enabled)
            self.pitch_input.setEnabled(enabled)
            self.yaw_input.setEnabled(enabled)
            for w in (self.roll_input, self.pitch_input, self.yaw_input):
                w.setStyleSheet("" if enabled else "color:#666666;")
        self.ori_check.stateChanged.connect(_toggle_ori)
        _toggle_ori(False)
        ori_layout.addWidget(QLabel("Orientación:"))
        ori_layout.addWidget(self.roll_input)
        ori_layout.addWidget(self.pitch_input)
        ori_layout.addWidget(self.yaw_input)
        ori_layout.addWidget(self.ori_check)
        ori_layout.addStretch()
        self.controls_layout.addLayout(ori_layout)

        # -- Jog Cartesiano --
        jog_frame = QFrame()
        jog_frame.setStyleSheet(
            'QFrame { border:1px solid #556677; border-radius:4px;'
            ' background:#0a0e14; padding:2px; }')
        jog_outer = QHBoxLayout(jog_frame)
        jog_outer.setContentsMargins(3, 2, 3, 2)
        jog_outer.setSpacing(2)
        jog_lbl = QLabel('JOG')
        jog_lbl.setStyleSheet('color:#778899; font-size:8px; font-weight:bold;'
                              ' background:transparent; border:none;')
        jog_outer.addWidget(jog_lbl)
        _JB = (
            'QPushButton { background:#0d1a2e; color:#88aacc;'
            ' border:1px solid #2a4a6a; border-radius:3px;'
            ' font-weight:bold; font-size:9px; min-width:36px; padding:2px 4px; }'
            'QPushButton:hover { background:#1a2a40; color:#aaccff; }'
            'QPushButton:pressed { background:#223355; }')
        for axis, sign, label in [
                ('X', +1, '+X'), ('X', -1, '−X'),
                ('Y', +1, '+Y'), ('Y', -1, '−Y'),
                ('Z', +1, '+Z'), ('Z', -1, '−Z')]:
            btn = QPushButton(label)
            btn.setStyleSheet(_JB)
            btn.clicked.connect(
                lambda _c=False, a=axis, s=sign: self._jog_cartesian(a, s))
            jog_outer.addWidget(btn)
        jog_outer.addSpacing(8)
        step_lbl = QLabel('paso:')
        step_lbl.setStyleSheet('color:#556677; font-size:8px;'
                               ' background:transparent; border:none;')
        jog_outer.addWidget(step_lbl)
        self._jog_step_combo = QComboBox()
        for v in ['1', '5', '10', '25', '50']:
            self._jog_step_combo.addItem(f'{v} mm', int(v))
        self._jog_step_combo.setCurrentIndex(1)
        self._jog_step_combo.setFixedWidth(39)
        self._jog_step_combo.setStyleSheet(
            'QComboBox { background:#0d1a2e; color:#88aacc;'
            ' border:1px solid #2a4a6a; border-radius:3px;'
            ' font-size:9px; padding:1px 4px; }'
            'QComboBox::drop-down { border:none; }'
            'QComboBox QAbstractItemView { background:#0d1a2e; color:#aaccff; }')
        jog_outer.addWidget(self._jog_step_combo)
        self._jog_status = QLabel('')
        self._jog_status.setStyleSheet(
            'color:#556677; font-size:8px; background:transparent; border:none;')
        jog_outer.addWidget(self._jog_status)
        jog_outer.addStretch()
        self.controls_layout.addWidget(jog_frame)

        # -- Jog RPY --
        jogr_frame = QFrame()
        jogr_frame.setStyleSheet(
            'QFrame { border:1px solid #554466; border-radius:4px;'
            ' background:#0e0a14; padding:2px; }')
        jogr_outer = QHBoxLayout(jogr_frame)
        jogr_outer.setContentsMargins(3, 2, 3, 2)
        jogr_outer.setSpacing(2)
        jogr_lbl = QLabel('RPY')
        jogr_lbl.setStyleSheet('color:#886699; font-size:8px; font-weight:bold;'
                               ' background:transparent; border:none;')
        jogr_outer.addWidget(jogr_lbl)
        _RB = (
            'QPushButton { background:#150d2e; color:#cc88ff;'
            ' border:1px solid #553388; border-radius:3px;'
            ' font-weight:bold; font-size:9px; min-width:38px; padding:2px 4px; }'
            'QPushButton:hover { background:#251040; color:#ddaaff; }'
            'QPushButton:pressed { background:#331555; }')
        for raxis, rsign, rlabel in [
                ('R', +1, '+R'), ('R', -1, '−R'),
                ('P', +1, '+P'), ('P', -1, '−P'),
                ('Y', +1, '+Yaw'), ('Y', -1, '−Yaw')]:
            rbtn = QPushButton(rlabel)
            rbtn.setStyleSheet(_RB)
            rbtn.clicked.connect(
                lambda _c=False, a=raxis, s=rsign: self._jog_rpy(a, s))
            jogr_outer.addWidget(rbtn)
        jogr_outer.addSpacing(8)
        rstep_lbl = QLabel('paso:')
        rstep_lbl.setStyleSheet('color:#443355; font-size:8px;'
                                ' background:transparent; border:none;')
        jogr_outer.addWidget(rstep_lbl)
        self._jog_rpy_combo = QComboBox()
        for v in ['1', '5', '10', '15', '30']:
            self._jog_rpy_combo.addItem(f'{v}°', int(v))
        self._jog_rpy_combo.setCurrentIndex(1)
        self._jog_rpy_combo.setFixedWidth(33)
        self._jog_rpy_combo.setStyleSheet(
            'QComboBox { background:#150d2e; color:#cc88ff;'
            ' border:1px solid #553388; border-radius:3px;'
            ' font-size:9px; padding:1px 4px; }'
            'QComboBox::drop-down { border:none; }'
            'QComboBox QAbstractItemView { background:#150d2e; color:#cc88ff; }')
        jogr_outer.addWidget(self._jog_rpy_combo)
        self._jog_rpy_status = QLabel('')
        self._jog_rpy_status.setStyleSheet(
            'color:#443355; font-size:8px; background:transparent; border:none;')
        jogr_outer.addWidget(self._jog_rpy_status)
        jogr_outer.addStretch()
        self.controls_layout.addWidget(jogr_frame)

        # ── Fila 3: controles por articulación ───────────────────────────
        lbl = QLabel("CONTROLES POR ARTICULACIÓN")
        lbl.setStyleSheet("font-size:10px; font-weight:bold; color:#70e570; margin-top:8px;")
        self.controls_layout.addWidget(lbl)

        for i in range(6):
            group = QFrame()
            group.setStyleSheet("QFrame { border:1px solid #5dd095; border-radius:5px; margin:4px; }")
            artic_layout = QVBoxLayout(group)

            hdr = QHBoxLayout()
            indicator = QLabel(); indicator.setFixedSize(8, 8)
            indicator.setStyleSheet("background-color:#ff6b6b; border-radius:7px;")
            lbl_j = QLabel(f"Articulación {i+1}: 0.00°")
            lbl_j.setStyleSheet("color:#93ffa8; font-weight:600; font-size:10px;")
            lbl_real = QLabel("Real: —")
            lbl_real.setStyleSheet("color:#ffcc44; font-weight:600; font-size:10px; margin-left:12px;")
            hdr.addWidget(indicator); hdr.addWidget(lbl_j); hdr.addWidget(lbl_real); hdr.addStretch()

            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(-360); slider.setMaximum(360); slider.setValue(0)

            ctrl_row = QHBoxLayout()
            inp = QLineEdit("0.0"); inp.setFixedWidth(48)
            btn_rst = QPushButton("Reset")
            btn_rst.clicked.connect(lambda _, idx=i: self.reset_articulation(idx))

            btn_ena = QPushButton("✓ Habilitado")
            btn_ena.setCheckable(True)
            btn_ena.setChecked(False)   # False = habilitado (motor activo)
            btn_ena.setFixedWidth(66)
            btn_ena.setStyleSheet(
                "QPushButton { background:#1a4a1a; border:1px solid #70e570;"
                " color:#70e570; border-radius:5px; padding:3px 6px; font-weight:bold; }"
                "QPushButton:checked { background:#4a1a1a; border:1px solid #ff6b6b;"
                " color:#ff6b6b; }"
            )
            def _toggle_enable(checked, idx=i, b=btn_ena):
                self._enable_states[idx] = checked
                b.setText("✗ Deshabilitado" if checked else "✓ Habilitado")
                if self.serial_port and self.serial_port.is_open:
                    cmd = f"enable[{idx}]={'0' if checked else '1'}\n"
                    try:
                        self.serial_port.write(cmd.encode("ascii"))
                    except Exception:
                        pass
            btn_ena.toggled.connect(_toggle_enable)
            self.enable_buttons.append(btn_ena)

            ctrl_row.addWidget(inp); ctrl_row.addWidget(btn_rst)
            ctrl_row.addWidget(btn_ena); ctrl_row.addStretch()

            btn_grid = QGridLayout()
            for col, delta in enumerate([0.5, 1, 5, 10]):
                b_p = QPushButton(f"+{delta}°")
                b_m = QPushButton(f"-{delta}°")
                b_p.clicked.connect(lambda _, idx=i, d=delta:  self.manual_move(idx,  d))
                b_m.clicked.connect(lambda _, idx=i, d=-delta: self.manual_move(idx,  d))
                btn_grid.addWidget(b_p, 0, col)
                btn_grid.addWidget(b_m, 1, col)

            artic_layout.addLayout(hdr)
            artic_layout.addWidget(slider)
            artic_layout.addLayout(ctrl_row)
            artic_layout.addLayout(btn_grid)

            self.sliders.append(slider)
            self.labels.append(lbl_j)
            self.real_angle_labels.append(lbl_real)
            self.inputs.append(inp)
            self.connection_indicators.append(indicator)

            slider.valueChanged.connect(self.make_rotator(i))
            slider.valueChanged.connect(lambda val, j=i: self.inputs[j].setText(f"{val % 360:.1f}"))
            inp.editingFinished.connect(self.make_input_handler(i))

            self.controls_layout.addWidget(group)

        # ── Botón IR A HOME ──────────────────────────────────────────
        home_frame = QFrame()
        home_frame.setStyleSheet(
            "QFrame { border:2px solid #8BC34A; border-radius:8px; margin:6px 4px; }")
        home_layout = QHBoxLayout(home_frame)
        home_layout.setContentsMargins(6, 3, 6, 3)
        btn_home = QPushButton("🏠  Ir a Home  (todas las articulaciones a 0°)")
        btn_home.setStyleSheet(
            "QPushButton { background:#0d1a0d; border:none; color:#8BC34A;"
            " font-size:11px; font-weight:bold; padding:8px 16px; border-radius:6px; }"
            "QPushButton:hover { background:#1a3a1a; }"
            "QPushButton:pressed { background:#0a100a; }")
        btn_home.clicked.connect(self._go_home)
        home_layout.addWidget(btn_home)
        self.controls_layout.addWidget(home_frame)

        self.controls_layout.addStretch()

        # Timer de lectura de ángulos reales desde ESP32 (110 ms)
        self._serial_rx_timer = QTimer(self)
        self._serial_rx_timer.setInterval(110)
        self._serial_rx_timer.timeout.connect(self._read_serial_angles)
        self._serial_rx_timer.start()

    def make_rotator(self, index):
        def rotar(angle):
            self.angles[index] = angle
            self.labels[index].setText(f"Articulación {index+1}: {angle % 360:.1f}°")
            self.send_setpoints(self.angles)
            self.update_xyz_display()
            self.update_3d_visualization()
        return rotar
 
    def make_input_handler(self, index):
        def handle_input():
            try:
                angle = float(self.inputs[index].text())
                if 0 <= angle <= 360:
                    self.sliders[index].blockSignals(True)
                    self.sliders[index].setValue(int(angle))
                    self.sliders[index].blockSignals(False)
                    self.angles[index] = angle
                    self.send_setpoints(self.angles)
                    self.update_xyz_display()
                    self.update_3d_visualization()
            except ValueError:
                pass
        return handle_input
 
    def manual_move(self, index, delta):
        new_angle = self.angles[index] + delta
        if not (-360 <= new_angle <= 360): return
        self.angles[index] = new_angle
        self.sliders[index].blockSignals(True)
        self.sliders[index].setValue(int(new_angle))
        self.sliders[index].blockSignals(False)
        self.inputs[index].setText(f"{new_angle % 360:.2f}")
        self.labels[index].setText(f"Articulación {index+1}: {new_angle % 360:.1f}°")
        self.send_setpoints(self.angles)
        self.update_xyz_display()
        self.update_3d_visualization()
 
    def reset_articulation(self, index):
        self.angles[index] = 0
        self.sliders[index].blockSignals(True)
        self.sliders[index].setValue(0)
        self.sliders[index].blockSignals(False)
        self.inputs[index].setText("0.0")
        self.labels[index].setText(f"Articulación {index+1}: 0°")
        self.send_reset(index)          # reset[i]=0.00 al ESP32
        self.update_xyz_display()
        self.update_3d_visualization()
 
    def reset_all(self):
        for i in range(6): self.reset_articulation(i)

    def _go_home(self):
        """Envía todas las articulaciones a 0° moviendo los motores (no reset encoder)."""
        self.angles = [0.0] * 6
        for i in range(6):
            self.sliders[i].blockSignals(True)
            self.sliders[i].setValue(0)
            self.sliders[i].blockSignals(False)
            self.inputs[i].setText("0.00")
            self.labels[i].setText(f"Articulación {i+1}: 0.00°")
        self.send_setpoints(self.angles)
        self.update_xyz_display()
        self.update_3d_visualization()
 
    # ─────────────────────────────────────────
    #  Cinemática Inversa (IK)
    # ─────────────────────────────────────────
 

    def go_to_xyz(self):
        try:
            x = float(self.x_input.text())
            y = float(self.y_input.text())
            z = float(self.z_input.text())
        except ValueError:
            self._set_ik_status("❌ X,Y,Z inválidos", error=True); return

        # Leer orientación si el checkbox está activo
        rpy_deg = None
        if self.ori_check.isChecked():
            try:
                rpy_deg = [float(self.roll_input.text()),
                           float(self.pitch_input.text()),
                           float(self.yaw_input.text())]
            except ValueError:
                self._set_ik_status("❌ Roll/Pitch/Yaw inválidos", error=True); return

        # Feedback visual durante el cálculo
        self.move_xyz_btn.setText("Calculando…")
        self.move_xyz_btn.setEnabled(False)
        self._set_ik_status("⏳ Calculando IK…", error=False)
        QApplication.processEvents()

        q_deg, err_pos_mm, err_ori_deg, ok = ik_dls(
            [x, y, z],
            self.angles,
            self.rotation_signs_mdh,
            target_rpy_deg=rpy_deg
        )

        self.move_xyz_btn.setText("Mover a XYZ")
        self.move_xyz_btn.setEnabled(True)

        if ok:
            for i in range(6):
                self.angles[i] = q_deg[i]
                self.sliders[i].blockSignals(True)
                self.sliders[i].setValue(int(q_deg[i]))
                self.sliders[i].blockSignals(False)
                self.inputs[i].setText(f"{q_deg[i]:.2f}")
                self.labels[i].setText(f"Articulación {i+1}: {q_deg[i]:.2f}°")
            self.update_xyz_display()
            self.update_3d_visualization()
            self.send_setpoints(self.angles)
            self._pos_history_add()   # registrar posición manual
            if rpy_deg is not None:
                self._set_ik_status(f"✅ pos={err_pos_mm:.1f}mm ori={err_ori_deg:.1f}°", error=False)
                print(f"✅ IK 6D convergida  pos={err_pos_mm:.2f}mm  ori={err_ori_deg:.2f}°  q={np.round(q_deg,2)}")
            else:
                self._set_ik_status(f"✅ pos={err_pos_mm:.1f}mm", error=False)
                print(f"✅ IK convergida  pos={err_pos_mm:.2f}mm  q={np.round(q_deg,2)}")
        else:
            if rpy_deg is not None:
                self._set_ik_status(f"❌ no convergió (pos={err_pos_mm:.0f}mm ori={err_ori_deg:.0f}°)", error=True)
            else:
                self._set_ik_status(f"❌ no convergió ({err_pos_mm:.0f}mm)", error=True)
            print(f"❌ IK no convergida  pos={err_pos_mm:.2f}mm  ori={err_ori_deg:.2f}°")

    def _jog_rpy(self, axis, sign):
        """Rota la orientación del TCP en el eje RPY indicado, manteniendo posición."""
        try:
            step_deg = self._jog_rpy_combo.currentData()
        except Exception:
            step_deg = 5
        thetas = np.deg2rad(np.array(self.angles, dtype=float))
        T = T_MDH_TO_FUSION @ cinematica_directa(thetas, self.rotation_signs_mdh)
        x, y, z = T[0,3]*1000, T[1,3]*1000, T[2,3]*1000
        rpy = np.degrees(rpy_from_matrix(T[:3,:3]))
        delta = float(sign) * float(step_deg)
        if   axis == 'R': rpy[0] += delta
        elif axis == 'P': rpy[1] += delta
        else:             rpy[2] += delta
        q_deg, err_mm, err_ori, ok = ik_dls(
            [x, y, z], self.angles, self.rotation_signs_mdh,
            target_rpy_deg=rpy.tolist())
        if ok:
            for i in range(6):
                self.angles[i] = q_deg[i]
                self.sliders[i].blockSignals(True)
                self.sliders[i].setValue(int(q_deg[i]))
                self.sliders[i].blockSignals(False)
                self.inputs[i].setText(f'{q_deg[i]:.2f}')
                self.labels[i].setText(f'Articulación {i+1}: {q_deg[i]:.2f}°')
            self.update_xyz_display()
            self.update_3d_visualization()
            self.send_setpoints(self.angles)
            self._pos_history_add()
            axis_name = {'R':'Roll','P':'Pitch','Y':'Yaw'}[axis]
            self._jog_rpy_status.setText(
                f'{delta:+.0f}° {axis_name} → ori={err_ori:.1f}°')
            self._jog_rpy_status.setStyleSheet(
                'color:#cc88ff; font-size:8px; background:transparent; border:none;')
        else:
            self._jog_rpy_status.setText(f'⚠ IK sin sol. ({err_mm:.0f}mm {err_ori:.0f}°)')
            self._jog_rpy_status.setStyleSheet(
                'color:#ff6b6b; font-size:8px; background:transparent; border:none;')

    def _jog_cartesian(self, axis, sign):

        """Mueve el TCP un paso en el eje cartesiano indicado."""
        try:
            step = self._jog_step_combo.currentData()
        except Exception:
            step = 5
        thetas = np.deg2rad(np.array(self.angles, dtype=float))
        T = T_MDH_TO_FUSION @ cinematica_directa(thetas, self.rotation_signs_mdh)
        x, y, z = T[0,3]*1000, T[1,3]*1000, T[2,3]*1000
        delta = float(sign) * float(step)
        if   axis == 'X': x += delta
        elif axis == 'Y': y += delta
        else:             z += delta
        q_deg, err_mm, _, ok = ik_dls([x, y, z], self.angles, self.rotation_signs_mdh)
        if ok:
            for i in range(6):
                self.angles[i] = q_deg[i]
                self.sliders[i].blockSignals(True)
                self.sliders[i].setValue(int(q_deg[i]))
                self.sliders[i].blockSignals(False)
                self.inputs[i].setText(f'{q_deg[i]:.2f}')
                self.labels[i].setText(f'Articulación {i+1}: {q_deg[i]:.2f}°')
            self.update_xyz_display()
            self.update_3d_visualization()
            self.send_setpoints(self.angles)
            self._pos_history_add()
            sign_str = f'{sign*step:+.0f}'
            self._jog_status.setText(
                f'{sign_str}mm {axis} → ({x:.0f},{y:.0f},{z:.0f})')
            self._jog_status.setStyleSheet(
                'color:#70e570; font-size:8px; background:transparent; border:none;')
        else:
            self._jog_status.setText(f'⚠ IK sin sol. ({err_mm:.0f}mm)')
            self._jog_status.setStyleSheet(
                'color:#ff6b6b; font-size:8px; background:transparent; border:none;')

    def _set_ik_status(self, msg, error=False):
        color = "#ff6b6b" if error else "#70e570"
        self.ik_status_label.setStyleSheet(f"font-size:9px; font-weight:bold; color:{color};")
        self.ik_status_label.setText(msg)
 
    # ─────────────────────────────────────────
    #  Visualización 3D
    # ─────────────────────────────────────────
 
