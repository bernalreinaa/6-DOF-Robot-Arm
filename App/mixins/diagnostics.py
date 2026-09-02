# -*- coding: utf-8 -*-
"""Panel de diagnóstico/calibración (comparación FK vs Fusion 360), histórico de posiciones, backup/restauración de configuración y asistente de calibración de offsets."""
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

from app_paths import BACKUPS_DIR
from kinematics import T_MDH_TO_FUSION, cinematica_directa
from dialogs import CalibrationWizard


class DiagnosticsMixin:
    """Ver Panel de diagnóstico/calibración (comparación FK vs Fusion 360), histórico de posiciones, backup/restauración de configuración y asistente de calibración de offsets."""

    def _build_diagnostic_panel(self):
        """Panel colapsable con dos pestañas: Diagnóstico y Rutinas."""
        outer = QFrame()
        outer.setStyleSheet("QFrame { border: 2px solid #f0a500; border-radius: 5px; }")
        outer_vbox = QVBoxLayout(outer)
        outer_vbox.setContentsMargins(2, 1, 2, 2)
        outer_vbox.setSpacing(0)

        # ── Header (siempre visible, pulsar para plegar/desplegar) ────────
        header = QWidget()
        header.setCursor(Qt.PointingHandCursor)
        h_row = QHBoxLayout(header)
        h_row.setContentsMargins(2, 1, 2, 1)

        self._diag_toggle = QToolButton()
        self._diag_toggle.setArrowType(Qt.DownArrow)
        self._diag_toggle.setStyleSheet("border:none; color:#f0a500;")
        h_row.addWidget(self._diag_toggle)

        lbl_h = QLabel("🔬  DIAGNÓSTICO  &  🤖  RUTINAS")
        lbl_h.setStyleSheet("font-size:10px; font-weight:bold; color:#f0a500;")
        h_row.addWidget(lbl_h)
        h_row.addStretch()

        # Botón de pantalla completa: oculta la vista 3D y los controles
        # manuales para que este panel (Diagnóstico/Rutinas/Log) ocupe toda
        # la ventana — útil en la pantalla táctil de 800x480, donde este
        # panel suele necesitar todo el espacio posible.
        self._diag_fullscreen_btn = QPushButton('⛶')
        self._diag_fullscreen_btn.setToolTip('Pantalla completa (ocultar vista 3D y controles)')
        self._diag_fullscreen_btn.setFixedSize(19, 14)
        self._diag_fullscreen_btn.setStyleSheet(
            "QPushButton { border:1px solid #f0a500; color:#f0a500; background:transparent;"
            " border-radius:4px; font-weight:bold; }"
            "QPushButton:hover { background:#f0a500; color:#23262b; }"
        )
        self._diag_prev_main_sizes = [650, 200]
        def _toggle_diag_fullscreen():
            if self._top_splitter.isVisible():
                self._diag_prev_main_sizes = self._main_splitter.sizes()
                self._top_splitter.setVisible(False)
                self._diag_fullscreen_btn.setText('🗗')
                self._diag_fullscreen_btn.setToolTip('Restaurar vista normal')
            else:
                self._top_splitter.setVisible(True)
                self._main_splitter.setSizes(self._diag_prev_main_sizes)
                self._diag_fullscreen_btn.setText('⛶')
                self._diag_fullscreen_btn.setToolTip('Pantalla completa (ocultar vista 3D y controles)')
        self._diag_fullscreen_btn.clicked.connect(_toggle_diag_fullscreen)
        h_row.addWidget(self._diag_fullscreen_btn)

        outer_vbox.addWidget(header)

        # ── Contenido plegable ────────────────────────────────────────────
        self._diag_content = QWidget()
        content_vbox = QVBoxLayout(self._diag_content)
        content_vbox.setContentsMargins(0, 1, 0, 0)

        tabs = QTabWidget()
        tabs.setStyleSheet("""
            QTabWidget::pane  { border: none; }
            QTabBar::tab      { color:#cccccc; padding:4px 14px; font-size:9px; }
            QTabBar::tab:selected { color:#f0a500; font-weight:bold;
                                    border-bottom:2px solid #f0a500; }
        """)
        tabs.addTab(self._build_diagnostico_tab(), "🔬  Diagnóstico")
        tabs.addTab(self._build_rutinas_tab(),     "🤖  Rutinas")
        tabs.addTab(self._build_log_tab(),         "📋  Log")
        content_vbox.addWidget(tabs)
        outer_vbox.addWidget(self._diag_content)

        # Toggle — colapsa a solo la cabecera (~32 px)
        def _toggle():
            vis = self._diag_content.isVisible()
            self._diag_content.setVisible(not vis)
            self._diag_toggle.setArrowType(Qt.RightArrow if vis else Qt.DownArrow)
            if vis:   # acabamos de colapsar → fijar altura mínima
                outer.setFixedHeight(header.sizeHint().height() + 10)
            else:     # acabamos de expandir → liberar
                outer.setMinimumHeight(1)
                outer.setMaximumHeight(16777215)

        self._diag_toggle.clicked.connect(_toggle)
        header.mousePressEvent = lambda _e: _toggle()

        return outer

    # ── Pestaña 1: contenido diagnóstico original ────────────────────────
    def _build_diagnostico_tab(self):
        w = QWidget()
        layout = QVBoxLayout(w)
        layout.setContentsMargins(2, 2, 2, 2)
        layout.setSpacing(2)

        instr = QLabel(
            "1) Pon todos los joints en 0°.  "
            "2) Mueve UN joint al ángulo de prueba.  "
            "3) Mide X,Y,Z en Fusion 360 y escríbelos.  "
            "4) Pulsa «Diagnosticar»."
        )
        instr.setStyleSheet("color:#cccccc; font-size:9px;")
        instr.setWordWrap(True)
        instr.setMaximumHeight(11)
        layout.addWidget(instr)

        row1 = QHBoxLayout()
        row1.addWidget(QLabel("Joint:"))
        self.diag_joint_combo = QComboBox()
        for j in range(1, 7):
            self.diag_joint_combo.addItem(f"Joint {j}")
        self.diag_joint_combo.setFixedWidth(48)
        row1.addWidget(self.diag_joint_combo)

        row1.addWidget(QLabel("Ángulo:"))
        self.diag_angle_input = QLineEdit("40")
        self.diag_angle_input.setFixedWidth(33)
        row1.addWidget(self.diag_angle_input)
        row1.addWidget(QLabel("°"))

        row1.addSpacing(16)
        for lbl_txt, attr in [("Fusion X:", "diag_fx"), ("Y:", "diag_fy"), ("Z:", "diag_fz")]:
            row1.addWidget(QLabel(lbl_txt))
            le = QLineEdit(); le.setFixedWidth(45)
            setattr(self, attr, le)
            row1.addWidget(le)

        btn_diag = QPushButton("Diagnosticar")
        btn_diag.setStyleSheet("border-color:#f0a500; color:#f0a500;")
        btn_diag.clicked.connect(self.run_diagnostico)
        row1.addWidget(btn_diag)

        btn_rst = QPushButton("Reset todos a 0°")
        btn_rst.setStyleSheet("border-color:#f0a500; color:#f0a500;")
        btn_rst.clicked.connect(self.reset_all)
        row1.addWidget(btn_rst)
        row1.addStretch()
        layout.addLayout(row1)

        row2 = QHBoxLayout()
        row2.addWidget(QLabel("Signos MDH:"))
        for i in range(6):
            is_neg = (self.rotation_signs_mdh[i] == -1)
            btn = QPushButton(f"J{i+1}: {'-1' if is_neg else '+1'}")
            btn.setFixedWidth(42); btn.setCheckable(True); btn.setChecked(is_neg)
            btn.setStyleSheet(self._sign_btn_style(is_neg))
            btn.clicked.connect(lambda checked, idx=i: self._toggle_mdh_sign(idx))
            self.sign_buttons_mdh.append(btn)
            row2.addWidget(btn)
        row2.addStretch()
        layout.addLayout(row2)

        self.diag_log = QTextEdit()
        self.diag_log.setReadOnly(True)
        self.diag_log.setFixedHeight(60)
        self.diag_log.setStyleSheet(
            "background:#181c1f; color:#f0a500; font-family:'Consolas',monospace;"
            " font-size:9px; border-radius:4px;"
        )
        self.diag_log.setPlaceholderText("Los resultados del diagnóstico aparecerán aquí…")
        layout.addWidget(self.diag_log)
        layout.addStretch()
        return w

    # ── Pestaña 2: Rutinas ────────────────────────────────────────────────
    def _pos_history_add(self, angles_override=None):
        """Captura la posición FK actual y la añade al historial.
        angles_override permite loguear el DESTINO de un movimiento de
        rutina en curso en vez de self.angles (que, con la animación 3D
        corriendo en paralelo al envío real del setpoint, todavía puede
        no haber llegado al valor final — ver _prog_start_move)."""
        import datetime
        try:
            if angles_override is not None:
                angles_src = angles_override
            else:
                angles_src = self.real_angles_feedback if self._free_move else self.angles
            thetas = np.deg2rad(np.array(angles_src, dtype=float))
            T = T_MDH_TO_FUSION @ cinematica_directa(thetas, self.rotation_signs_mdh)
            x, y, z = T[0,3]*1000, T[1,3]*1000, T[2,3]*1000
            ts = datetime.datetime.now().strftime('%H:%M:%S')
            self._pos_history.appendleft({'x': x, 'y': y, 'z': z,
                                          'traj': 'Articular', 'ts': ts,
                                          'angles': [float(a) for a in angles_src]})
            self._rebuild_pos_hist_rows()
        except Exception:
            pass

    def _rebuild_pos_hist_rows(self):
        """Reconstruye las filas del panel de historial."""
        lay = self._pos_hist_rows_layout
        if lay is None:
            return
        # Limpiar filas existentes
        for i in reversed(range(lay.count())):
            item = lay.itemAt(i)
            if item and item.widget():
                item.widget().deleteLater()
                lay.removeItem(item)
        _STY = ('background:#0a0e14; color:#ccc; border:1px solid #1e2530;'
                ' border-radius:3px; padding:1px 4px; font-size:9px;')
        for entry in list(self._pos_history):
            row = QWidget()
            row.setStyleSheet('background:transparent; border:none;')
            rl = QHBoxLayout(row)
            rl.setContentsMargins(1, 1, 1, 1)
            rl.setSpacing(2)
            ts_lbl = QLabel(entry['ts'])
            ts_lbl.setStyleSheet('color:#556677; font-size:8px; background:transparent; border:none;')
            ts_lbl.setFixedWidth(31)
            coord_lbl = QLabel(
                f"X <b>{entry['x']:.1f}</b>  "
                f"Y <b>{entry['y']:.1f}</b>  "
                f"Z <b>{entry['z']:.1f}</b>")
            coord_lbl.setStyleSheet('color:#aaccdd; font-size:9px; background:transparent; border:none;')
            btn_var = QPushButton('🔖 Var')
            btn_var.setFixedHeight(11)
            btn_var.setStyleSheet(
                'QPushButton { border:1px solid #336699; color:#5599cc; background:#0d1a2e;'
                ' border-radius:3px; font-size:8px; padding:0 5px; }'
                'QPushButton:hover { background:#1a2a40; }')
            btn_blk = QPushButton('➕ Bloque')
            btn_blk.setFixedHeight(11)
            btn_blk.setStyleSheet(
                'QPushButton { border:1px solid #336633; color:#55aa55; background:#0d1a0d;'
                ' border-radius:3px; font-size:8px; padding:0 5px; }'
                'QPushButton:hover { background:#1a2a1a; }')
            def _save_var(_checked=False, e=entry):
                name = f"Pos_{e['ts'].replace(':','')}"
                self._add_position_var(name, e['x'], e['y'], e['z'], e['traj'],
                                        angles=e.get('angles'))
            def _insert_block(_checked=False, e=entry):
                block = self._prog_container.add_block('move')
                block._mx.setText(f"{e['x']:.1f}")
                block._my.setText(f"{e['y']:.1f}")
                block._mz.setText(f"{e['z']:.1f}")
                idx = block._mtraj.findText(e['traj'])
                if idx >= 0: block._mtraj.setCurrentIndex(idx)
            btn_var.clicked.connect(_save_var)
            btn_blk.clicked.connect(_insert_block)
            rl.addWidget(ts_lbl)
            rl.addWidget(coord_lbl, 1)
            rl.addWidget(btn_var)
            rl.addWidget(btn_blk)
            lay.addWidget(row)
        # Update count label
        if hasattr(self, '_pos_hist_count_lbl'):
            self._pos_hist_count_lbl.setText(f'({len(self._pos_history)})')

    def _backup_config(self):
        """Guarda toda la configuración en un JSON con timestamp."""
        import datetime
        ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        fname = os.path.join(BACKUPS_DIR, f'config_{ts}.json')
        data = {
            'version': 1,
            'timestamp': ts,
            'position_vars': dict(self._position_vars),
            'serial_port': self.com_combo.currentText(),
            'baud_rate': self.baud_combo.currentText(),
            'rotation_signs_mdh': list(self.rotation_signs_mdh),
            'angles': list(self.angles),
        }
        try:
            with open(fname, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            self._log_error(f'Backup guardado: {os.path.basename(fname)}', 'INFO')
            QMessageBox.information(self, 'Backup', f'Configuración guardada en:\n{fname}')
        except Exception as e:
            self._log_error(f'Backup error: {e}', 'ERROR')
            QMessageBox.critical(self, 'Error en backup', str(e))

    def _restore_config(self):
        """Carga configuración desde un archivo de backup JSON."""
        fname, _ = QFileDialog.getOpenFileName(
            self, 'Restaurar configuración', BACKUPS_DIR,
            'Backups JSON (*.json);;Todos los archivos (*)')
        if not fname:
            return
        try:
            with open(fname, 'r', encoding='utf-8') as f:
                data = json.load(f)
            # 1 — Variables de posición
            if 'position_vars' in data:
                self._position_vars = data['position_vars']
                self._rebuild_var_rows()
                self._refresh_all_var_combos()
            # 2 — Puerto serie y baudios
            if 'serial_port' in data:
                port = data['serial_port']
                idx = self.com_combo.findText(port)
                if idx >= 0:
                    self.com_combo.setCurrentIndex(idx)
                else:
                    self.com_combo.addItem(port)
                    self.com_combo.setCurrentText(port)
            if 'baud_rate' in data:
                self.baud_combo.setCurrentText(str(data['baud_rate']))
            # 3 — Signos MDH (calibración)
            if 'rotation_signs_mdh' in data:
                signs = data['rotation_signs_mdh']
                for i, s in enumerate(signs[:6]):
                    self.rotation_signs_mdh[i] = int(s)
                    if i < len(self.sign_buttons_mdh):
                        is_neg = (int(s) == -1)
                        self.sign_buttons_mdh[i].setChecked(is_neg)
                        self.sign_buttons_mdh[i].setText(f"J{i+1}: {'-1' if is_neg else '+1'}")
                        self.sign_buttons_mdh[i].setStyleSheet(
                            self._sign_btn_style(is_neg))
            # 4 — Ángulos actuales
            if 'angles' in data:
                saved = data['angles']
                for i, a in enumerate(saved[:6]):
                    self.angles[i] = float(a)
                    if i < len(self.sliders):
                        self.sliders[i].blockSignals(True)
                        self.sliders[i].setValue(int(float(a)))
                        self.sliders[i].blockSignals(False)
                    if i < len(self.inputs):
                        self.inputs[i].setText(f'{float(a):.1f}')
                    if i < len(self.labels):
                        self.labels[i].setText(f'Articulación {i+1}: {float(a):.1f}°')
                self.update_xyz_display()
                self.update_3d_visualization()
            self._log_error(f'Config restaurada: {os.path.basename(fname)}', 'INFO')
            QMessageBox.information(self, 'Restaurar',
                f'Configuración restaurada desde:\n{os.path.basename(fname)}')
        except Exception as e:
            self._log_error(f'Restore error: {e}', 'ERROR')
            QMessageBox.critical(self, 'Error al restaurar', str(e))
    def _sign_btn_style(self, is_negative):
        color = "#ff6b6b" if is_negative else "#70e570"
        return f"border:1.5px solid {color}; color:{color}; border-radius:6px; font-weight:bold;"
 
    def _toggle_mdh_sign(self, idx):
        is_neg = self.sign_buttons_mdh[idx].isChecked()
        self.rotation_signs_mdh[idx] = -1 if is_neg else 1
        self.sign_buttons_mdh[idx].setText(f"J{idx+1}: {'-1' if is_neg else '+1'}")
        self.sign_buttons_mdh[idx].setStyleSheet(self._sign_btn_style(is_neg))
        self.update_xyz_display()
        self.diag_log.append(
            f"[Signo MDH J{idx+1}] → {self.rotation_signs_mdh[idx]:+d}   "
            f"signos actuales: {self.rotation_signs_mdh}"
        )
 
    def run_diagnostico(self):
        joint_idx = self.diag_joint_combo.currentIndex()
        try:
            test_angle = float(self.diag_angle_input.text())
        except ValueError:
            self.diag_log.append("❌ Ángulo inválido."); return
 
        test_deg   = [0.0] * 6
        test_deg[joint_idx] = test_angle
        thetas_rad = np.deg2rad(test_deg)
 
        T_raw    = T_MDH_TO_FUSION @ cinematica_directa(thetas_rad, rotation_signs_mdh=None)
        T_signed = T_MDH_TO_FUSION @ cinematica_directa(thetas_rad, rotation_signs_mdh=self.rotation_signs_mdh)
        x_r, y_r, z_r = T_raw[0,3]*1e3,    T_raw[1,3]*1e3,    T_raw[2,3]*1e3
        x_s, y_s, z_s = T_signed[0,3]*1e3, T_signed[1,3]*1e3, T_signed[2,3]*1e3
 
        self.diag_log.append(f"\n{'='*60}")
        self.diag_log.append(f"Joint {joint_idx+1}  |  {test_angle}°  |  signos={self.rotation_signs_mdh}")
        self.diag_log.append(f"  FK (sin signos): X={x_r:.2f}  Y={y_r:.2f}  Z={z_r:.2f}")
        self.diag_log.append(f"  FK (con signos): X={x_s:.2f}  Y={y_s:.2f}  Z={z_s:.2f}")
 
        fx_t, fy_t, fz_t = self.diag_fx.text().strip(), self.diag_fy.text().strip(), self.diag_fz.text().strip()
        if fx_t and fy_t and fz_t:
            try:
                fx, fy, fz = float(fx_t), float(fy_t), float(fz_t)
                dx, dy, dz = x_s-fx, y_s-fy, z_s-fz
                self.diag_log.append(f"  Fusion 360    : X={fx:.2f}  Y={fy:.2f}  Z={fz:.2f}")
                self.diag_log.append(f"  Error         : ΔX={dx:.2f}  ΔY={dy:.2f}  ΔZ={dz:.2f}")
                sugg = []
                for va, er, vf, ax in [(x_s,dx,fx,'X'),(y_s,dy,fy,'Y'),(z_s,dz,fz,'Z')]:
                    if abs(vf) > 1 and abs(er) > abs(vf) * 0.3:
                        if abs(va + vf) < abs(er):
                            sugg.append(f"{ax}: error={er:.1f}mm → flipea signo J{joint_idx+1}")
                        else:
                            sugg.append(f"{ax}: error={er:.1f}mm → revisar parámetros MDH")
                if sugg:
                    for s in sugg: self.diag_log.append(f"  ⚠️  {s}")
                else:
                    self.diag_log.append("  ✅  Error aceptable")
            except ValueError:
                self.diag_log.append("  ⚠️  Valores Fusion inválidos")
        else:
            self.diag_log.append("  ℹ️  Sin valores Fusion (solo FK)")
 
        for i in range(6):
            self.angles[i] = test_deg[i]
            self.sliders[i].blockSignals(True)
            self.sliders[i].setValue(int(test_deg[i]))
            self.sliders[i].blockSignals(False)
            self.inputs[i].setText(f"{test_deg[i]:.2f}")
            self.labels[i].setText(f"Articulación {i+1}: {test_deg[i]:.2f}°")
        self.update_xyz_display()
        self.update_3d_visualization()
 
    # ─────────────────────────────────────────
    #  Controles manuales
    # ─────────────────────────────────────────
 
    def _open_calibration_wizard(self):
        dlg = CalibrationWizard(self)
        dlg.exec_()

    def _save_joint_offsets(self):
        """Persiste los offsets en backups/calibration.json."""
        cal_path = os.path.join(BACKUPS_DIR, 'calibration.json')
        try:
            with open(cal_path, 'w', encoding='utf-8') as f:
                json.dump({'joint_offsets': self._joint_offsets}, f, indent=2)
            self._log_error(f'Offsets guardados: {self._joint_offsets}', 'INFO')
        except Exception as e:
            self._log_error(f'Error guardando offsets: {e}', 'ERROR')

    def _load_joint_offsets(self):
        """Carga offsets desde backups/calibration.json si existe."""
        cal_path = os.path.join(BACKUPS_DIR, 'calibration.json')
        if not os.path.exists(cal_path):
            return
        try:
            with open(cal_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            offs = data.get('joint_offsets', [0.0]*6)
            for i, o in enumerate(offs[:6]):
                self._joint_offsets[i] = float(o)
            self._log_error(f'Offsets cargados: {self._joint_offsets}', 'INFO')
        except Exception as e:
            self._log_error(f'Error cargando offsets: {e}', 'ERROR')

    # ─────────────────────────────────────────
    #  Tema oscuro
    # ─────────────────────────────────────────
