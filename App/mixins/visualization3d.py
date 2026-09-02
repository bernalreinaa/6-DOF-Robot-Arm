# -*- coding: utf-8 -*-
"""Visualización 3D del brazo (PyVista/VTK): actores del modelo, ghost, envolvente de trabajo, cámara y aviso de proximidad a zona prohibida."""
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

from app_paths import BASE_DIR
from kinematics import (numpy_to_vtk_matrix, translate, rotate_x, rotate_y, rotate_z,
                         T_MDH_TO_FUSION, cinematica_directa, rpy_from_matrix,
                         JOINT_FORBIDDEN_ZONES, _angle_is_forbidden, ik_dls)
import pyvista as pv
import vtk


class Visualization3DMixin:
    """Ver Visualización 3D del brazo (PyVista/VTK): actores del modelo, ghost, envolvente de trabajo, cámara y aviso de proximidad a zona prohibida."""

    def setup_3d_visualization(self):
        self.actors = []
        colores = ["green", "green", "green", "green", "green", "green"]
 
        self.rotation_origins = [
            (0.0,       0.0,      0.0    ),
            (6.40,      0.0,     20.76   ),
            (6.40,     -4.3535,  43.4599 ),
            (10.3237,  -0.296,   48.5599 ),
            (-18.3263, -2.504,   48.5599 ),
            (-18.3263, -0.546,   45.5099 ),
        ]
        self.rotation_axes = ['Z', 'Y', 'Y', 'X', 'Y', 'Z']
 
        piece_home_positions = [
            (0.0,       0.0,      0.0    ),
            (0.0,       0.0,      0.0    ),
            (6.40,     -0.81,    20.76   ),
            (6.40,     -4.3535,  43.4599 ),
            (10.3237,  -0.296,   48.5599 ),
            (-18.3263, -2.504,   48.5599 ),
            (0.0,       0.0,      0.0    ),
        ]
 
        for i in range(7):
            try:
                mesh = pv.read(os.path.join(BASE_DIR, f"P{i+1}.obj"))
                mesh.translate(piece_home_positions[i], inplace=True)
                actor = self.plotter_widget.add_mesh(
                    mesh, color=colores[i % len(colores)], name=f"pieza_{i+1}"
                )
                self.actors.append(actor)
            except Exception as e:
                print(f"Error cargando P{i+1}.obj: {e}")
                self.actors.append(None)
 
        # ── Ghost arm (mismos OBJ, azul semi-transparente, oculto por defecto) ──
        for i in range(7):
            try:
                mesh = pv.read(os.path.join(BASE_DIR, f"P{i+1}.obj"))
                mesh.translate(piece_home_positions[i], inplace=True)
                actor = self.plotter_widget.add_mesh(
                    mesh,
                    color="#3399ff",
                    opacity=0.28,
                    name=f"ghost_{i+1}"
                )
                actor.SetVisibility(False)
                self.ghost_actors.append(actor)
            except Exception as e:
                print(f"Error cargando ghost P{i+1}.obj: {e}")
                self.ghost_actors.append(None)

        # Plano de referencia
        L = 50
        self.plotter_widget.add_mesh(pv.Plane(i_size=2*L, j_size=2*L), color="#333333", opacity=0.9)
        self.plotter_widget.add_axes()
        self.plotter_widget.set_background("white")
        self.plotter_widget.view_vector((-1, -1, 0.7))
 

    def update_3d_visualization(self):
        _angles = self.real_angles_feedback if self._viz_real else self.angles
        signed_deg = [s * a for s, a in zip(self.rotation_signs_visual, _angles)]
 
        for actor in self.actors[1:]:
            if actor is not None: actor.SetUserMatrix(None)
 
        transform = np.identity(4)
        for i, actor in enumerate(self.actors[1:], start=1):
            if actor is None: continue
            origin    = np.array(self.rotation_origins[i - 1])
            axis      = self.rotation_axes[i - 1]
            angle_rad = np.radians(signed_deg[i - 1])
            T_fwd = translate(origin); T_inv = translate(-origin)
            if   axis == 'X': R = rotate_x(angle_rad)
            elif axis == 'Y': R = rotate_y(angle_rad)
            else:             R = rotate_z(angle_rad)
            local = T_fwd @ R @ T_inv
            transform = transform @ local
            actor.SetUserMatrix(numpy_to_vtk_matrix(transform))
 
        self.plotter_widget.render()
 
    def _reset_camera_view(self):
        """Recupera exactamente la vista de cámara con la que arranca la
        app (mismo view_vector que se usa al cargar las piezas del brazo)."""
        self.plotter_widget.view_vector((-1, -1, 0.7))
        self.plotter_widget.render()

    def _toggle_viz_real(self, checked):
        self._viz_real = checked
        self._viz_real_btn.setText('👁 Vista Real' if checked else '👁 Vista App')
        self.update_3d_visualization()

    def _toggle_free_move(self, checked):
        self._free_move = checked
        self._free_btn.setText('🔒 Mover libre ON' if checked else '🔓 Mover libre')
        if self.serial_port and self.serial_port.is_open:
            for idx in range(6):
                cmd = 'enable[{}]={}\n'.format(idx, '0' if checked else '1')
                try:
                    self.serial_port.write(cmd.encode('ascii'))
                except Exception:
                    pass
        for idx, btn in enumerate(self.enable_buttons):
            btn.blockSignals(True)
            btn.setChecked(checked)
            btn.setText('✗ Deshabilitado' if checked else '✓ Habilitado')
            self._enable_states[idx] = checked
            btn.blockSignals(False)
        if checked:
            self._viz_real = True
            self._viz_real_btn.setChecked(True)
            self._viz_real_btn.setText('👁 Vista Real')
            self._free_move_timer.start()
        else:
            self._free_move_timer.stop()
            self._range_warning_active = False
            self._range_warning_blink_timer.stop()
            self._range_warning_lbl.setVisible(False)

    def _free_move_tick(self):
        self.update_3d_visualization()
        self._update_xyz_from_angles(self.real_angles_feedback)
        self._check_range_warning()

    def _check_range_warning(self):
        """Parpadea un aviso rojo si, en movimiento libre, alguna
        articulación real cae en su zona prohibida. Solo visual: no
        bloquea ni envía nada, únicamente avisa."""
        offenders = []
        for idx in range(6):
            zone = JOINT_FORBIDDEN_ZONES.get(idx)
            if zone is not None and _angle_is_forbidden(self.real_angles_feedback[idx], zone):
                offenders.append(idx + 1)

        if offenders:
            nombres = ', '.join(f'Art.{i}' for i in offenders)
            self._range_warning_lbl.setText(f'\u26a0 FUERA DE RANGO PERMITIDO: {nombres}')
            if not self._range_warning_active:
                self._range_warning_active = True
                self._range_warning_lbl.setVisible(True)
                self._range_warning_blink_timer.start()
        else:
            if self._range_warning_active:
                self._range_warning_active = False
                self._range_warning_blink_timer.stop()
                self._range_warning_lbl.setVisible(False)

    def _range_warning_blink_tick(self):
        if not self._range_warning_active:
            self._range_warning_lbl.setVisible(False)
            return
        self._range_warning_lbl.setVisible(not self._range_warning_lbl.isVisible())

    def _update_xyz_from_angles(self, angles):
        try:
            thetas = np.deg2rad(np.array(angles, dtype=float))
            T = T_MDH_TO_FUSION @ cinematica_directa(thetas, rotation_signs_mdh=self.rotation_signs_mdh)
            self.x_input.setText(f"{T[0,3]*1000:.2f}")
            self.y_input.setText(f"{T[1,3]*1000:.2f}")
            self.z_input.setText(f"{T[2,3]*1000:.2f}")
            rpy = rpy_from_matrix(T[:3, :3])
            self.roll_input.setText(f"{np.degrees(rpy[0]):.2f}")
            self.pitch_input.setText(f"{np.degrees(rpy[1]):.2f}")
            self.yaw_input.setText(f"{np.degrees(rpy[2]):.2f}")
        except Exception:
            pass

    def update_xyz_display(self):
        thetas = np.deg2rad(np.array(self.angles, dtype=float))
        T = T_MDH_TO_FUSION @ cinematica_directa(thetas, rotation_signs_mdh=self.rotation_signs_mdh)
        self.x_input.setText(f"{T[0,3]*1000:.2f}")
        self.y_input.setText(f"{T[1,3]*1000:.2f}")
        self.z_input.setText(f"{T[2,3]*1000:.2f}")
        # Actualizar también los campos de orientación con la RPY actual de la FK
        rpy = rpy_from_matrix(T[:3, :3])
        self.roll_input.setText(f"{np.degrees(rpy[0]):.2f}")
        self.pitch_input.setText(f"{np.degrees(rpy[1]):.2f}")
        self.yaw_input.setText(f"{np.degrees(rpy[2]):.2f}")

    def update_ghost_visualization(self, ghost_angles):
        """Aplica la misma cadena cinemática al brazo fantasma sin mover el real."""
        signed_deg = [s * a for s, a in zip(self.rotation_signs_visual, ghost_angles)]

        for actor in self.ghost_actors[1:]:
            if actor is not None:
                actor.SetUserMatrix(None)

        transform = np.identity(4)
        for i, actor in enumerate(self.ghost_actors[1:], start=1):
            if actor is None:
                continue
            origin    = np.array(self.rotation_origins[i - 1])
            axis      = self.rotation_axes[i - 1]
            angle_rad = np.radians(signed_deg[i - 1])
            T_fwd = translate(origin)
            T_inv = translate(-origin)
            if   axis == 'X': R = rotate_x(angle_rad)
            elif axis == 'Y': R = rotate_y(angle_rad)
            else:             R = rotate_z(angle_rad)
            local     = T_fwd @ R @ T_inv
            transform = transform @ local
            actor.SetUserMatrix(numpy_to_vtk_matrix(transform))

        self.plotter_widget.render()


    def _clear_ghost(self):
        """Oculta el brazo fantasma, para la animación y resetea el estado."""
        self._ghost_timer.stop()
        for actor in self.ghost_actors:
            if actor is not None:
                actor.SetVisibility(False)
        self._ghost_visible = False
        if hasattr(self, 'ghost_btn'):
            self.ghost_btn.setText("🎬 Animar Ghost")
        self.plotter_widget.render()

    def show_ghost(self):
        """
        Calcula la IK y anima el brazo fantasma desde la posición actual hasta
        el target en pasos de ~3° por articulación (25 ms/frame).
        Pulsar de nuevo detiene/oculta el ghost.
        """
        # Si la animación está corriendo → detener y dejar ghost donde está
        if self._ghost_timer.isActive():
            self._ghost_timer.stop()
            self.ghost_btn.setText("✕ Ocultar Ghost")
            self._set_ik_status("Animación detenida", error=False)
            return

        # Si el ghost está quieto → ocultarlo
        if self._ghost_visible:
            self._clear_ghost()
            self._set_ik_status("Ghost oculto", error=False)
            return

        # ── Leer target ───────────────────────────────────────────────────
        try:
            x = float(self.x_input.text())
            y = float(self.y_input.text())
            z = float(self.z_input.text())
        except ValueError:
            self._set_ik_status("❌ X,Y,Z inválidos", error=True); return

        rpy_deg = None
        if self.ori_check.isChecked():
            try:
                rpy_deg = [float(self.roll_input.text()),
                           float(self.pitch_input.text()),
                           float(self.yaw_input.text())]
            except ValueError:
                self._set_ik_status("❌ Roll/Pitch/Yaw inválidos", error=True); return

        # ── Calcular IK ───────────────────────────────────────────────────
        self.ghost_btn.setText("Calculando…")
        self.ghost_btn.setEnabled(False)
        self._set_ik_status("⏳ Calculando IK…", error=False)
        QApplication.processEvents()

        q_deg, err_pos_mm, err_ori_deg, ok = ik_dls(
            [x, y, z],
            self.angles,
            self.rotation_signs_mdh,
            target_rpy_deg=rpy_deg
        )

        self.ghost_btn.setEnabled(True)

        if not ok:
            self.ghost_btn.setText("🎬 Animar Ghost")
            if rpy_deg is not None:
                self._set_ik_status(
                    f"❌ IK no convergió (pos={err_pos_mm:.0f}mm ori={err_ori_deg:.0f}°)",
                    error=True)
            else:
                self._set_ik_status(
                    f"❌ IK no convergió ({err_pos_mm:.0f}mm)", error=True)
            return

        # ── Preparar animación ────────────────────────────────────────────
        self._ghost_anim_q_start = np.array(self.angles, dtype=float)
        self._ghost_anim_q_end   = np.array(q_deg,       dtype=float)
        self._ghost_anim_err_pos = err_pos_mm
        self._ghost_anim_err_ori = err_ori_deg

        # Número de pasos: recorrido máximo en grados / 3° por paso (mín. 1)
        max_delta = np.max(np.abs(self._ghost_anim_q_end - self._ghost_anim_q_start))
        self._ghost_anim_total = max(int(np.ceil(max_delta / 3.0)), 1)
        self._ghost_anim_step  = 0

        # Mostrar ghost en posición inicial (misma que brazo real)
        for actor in self.ghost_actors:
            if actor is not None:
                actor.SetVisibility(True)
        self.update_ghost_visualization(self.angles)
        self._ghost_visible = True

        self.ghost_btn.setText("⏹ Detener")
        self._set_ik_status(
            f"🎬 Animando… {self._ghost_anim_total} pasos", error=False)

        # Arrancar timer: 25 ms/frame ≈ 40 fps
        self._ghost_timer.start(25)

    def _animate_ghost_step(self):
        """Avanza un paso de la animación del ghost (llamado por QTimer)."""
        self._ghost_anim_step += 1
        t = min(self._ghost_anim_step / self._ghost_anim_total, 1.0)

        # Interpolación lineal entre posición actual y objetivo
        ghost_angles = (self._ghost_anim_q_start
                        + t * (self._ghost_anim_q_end - self._ghost_anim_q_start))
        self.update_ghost_visualization(ghost_angles.tolist())

        if self._ghost_anim_step >= self._ghost_anim_total:
            self._ghost_timer.stop()
            self.ghost_btn.setText("✕ Ocultar Ghost")
            if self._ghost_anim_err_ori > 0:
                self._set_ik_status(
                    f"🎬 Ghost listo: pos={self._ghost_anim_err_pos:.1f}mm"
                    f"  ori={self._ghost_anim_err_ori:.1f}°",
                    error=False)
            else:
                self._set_ik_status(
                    f"🎬 Ghost listo: pos={self._ghost_anim_err_pos:.1f}mm",
                    error=False)
