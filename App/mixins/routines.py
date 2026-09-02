# -*- coding: utf-8 -*-
"""Editor visual de rutinas por bloques (variables de posición, trayectorias, deshacer/rehacer, guardar/cargar, exportar a Python, previsualización y envolvente de trabajo) y el motor de ejecución (run/pause/resume/cancel y la máquina de estados de _prog_tick)."""
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

from app_paths import BASE_DIR, RUTINAS_DIR, TOOLS_FILE
from kinematics import (T_MDH_TO_FUSION, cinematica_directa, rpy_from_matrix, set_tool_offset,
                         JOINT_FORBIDDEN_ZONES, _wrap360, _angular_diff, _q_respects_joint_limits,
                         _resolve_anim_q_end, _describe_ik_failure, ik_dls)
from routine_editor_widgets import FlowchartDialog, _block_restore, BlockContainer
from dialogs import ToolProfileDialog
import pyvista as pv


class RoutinesMixin:
    """Ver Editor visual de rutinas por bloques (variables de posición, trayectorias, deshacer/rehacer, guardar/cargar, exportar a Python, previsualización y envolvente de trabajo) y el motor de ejecución (run/pause/resume/cancel y la máquina de estados de _prog_tick)."""

    def _export_python(self):
        """Convierte los bloques de la rutina actual a un script Python."""
        import datetime
        blocks = self._prog_container.get_data()
        if not blocks:
            QMessageBox.warning(self, 'Exportar Python', 'No hay bloques en la rutina.')
            return
        default_path = os.path.join(BASE_DIR, 'rutina_exportada.py')
        fname, _ = QFileDialog.getSaveFileName(
            self, 'Exportar rutina como Python', default_path,
            'Python (*.py);;Todos los archivos (*)')
        if not fname:
            return

        ts = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        lines = [
            '# Rutina exportada desde Brazo Robótico 6 GDL',
            f'# Exportado: {ts}',
            '',
            'import time',
            '',
        ]

        # Variables de posición
        if self._position_vars:
            lines.append('# ── Variables de posición ─────────────────────────')
            for vname, vd in self._position_vars.items():
                lines.append(
                    f"{vname} = ({vd['x']:.1f}, {vd['y']:.1f}, {vd['z']:.1f})"
                    f"  # traj: {vd.get('traj','Articular')}")
                angs = vd.get('angles')
                if angs:
                    vals = ', '.join(f'{a:.2f}' for a in angs)
                    lines.append(f"{vname}_angulos = ({vals})  # J1..J6, sin pasar por IK")
            lines.append('')

        # Stub helpers
        lines += [
            '# ── Funciones de control (implementa según tu hardware) ──────',
            'def mover_a(x, y, z, traj="Articular", vel_pct=100):',
            '    """Mover brazo a XYZ (mm). vel_pct: % de velocidad (10-100) para este movimiento."""',
            '    print(f"MOVE {traj} @ {vel_pct}%: X={x:.1f} Y={y:.1f} Z={z:.1f}")',
            '',
            'def mover_a_ori(x, y, z, roll, pitch, yaw, traj="Articular", vel_pct=100):',
            '    """Mover brazo a XYZ (mm) con orientación RPY (°) de la herramienta."""',
            '    print(f"MOVE_ORI {traj} @ {vel_pct}%: X={x:.1f} Y={y:.1f} Z={z:.1f} "',
            '          f"R={roll:.1f} P={pitch:.1f} Yaw={yaw:.1f}")',
            '',
            'def mover_articulaciones(j1, j2, j3, j4, j5, j6, traj="Articular", vel_pct=100):',
            '    """Mover directo a estos 6 ángulos (°), sin pasar por cinemática inversa."""',
            '    print(f"MOVE_JOINTS {traj} @ {vel_pct}%: "',
            '          f"J1={j1:.2f} J2={j2:.2f} J3={j3:.2f} J4={j4:.2f} J5={j5:.2f} J6={j6:.2f}")',
            '',
            'def pausa(segundos):',
            '    time.sleep(segundos)',
            '',
            'def ir_a_home():',
            '    print("HOME")',
            '',
            'def bomba_vacio(estado):',
            '    print(f"VACÍO: {\'ON\' if estado else \'OFF\'}")',
            '',
            'def cinta_transportadora(arranque, vel_pct=50):',
            '    print(f"CINTA: {\'ARRANQUE\' if arranque else \'PARO\'} @ {vel_pct}%")',
            '',
            '',
        ]

        def _gen(blks, depth=1):
            pad = '    ' * depth
            out = []
            for b in blks:
                t = b.get('type', '')
                if t == 'move':
                    traj = b.get('traj', 'Articular')
                    vel_pct = b.get('vel_pct', 100)
                    if 'var' in b:
                        vname = b['var']
                        vd = self._position_vars.get(vname, {})
                        if b.get('var_mode') == 'angles' and vd.get('angles'):
                            out.append(f"{pad}mover_articulaciones(*{vname}_angulos, traj='{traj}', vel_pct={vel_pct})")
                        else:
                            out.append(f"{pad}mover_a(*{vname}, traj='{traj}', vel_pct={vel_pct})")
                    else:
                        x, y, z = b.get('x', 0), b.get('y', 0), b.get('z', 200)
                        out.append(f"{pad}mover_a({x:.1f}, {y:.1f}, {z:.1f}, traj='{traj}', vel_pct={vel_pct})")
                elif t == 'move_ori':
                    traj = b.get('traj', 'Articular')
                    roll, pitch, yaw = b.get('roll', 0), b.get('pitch', 0), b.get('yaw', 0)
                    vel_pct = b.get('vel_pct', 100)
                    if 'var' in b:
                        vname = b['var']
                        vd = self._position_vars.get(vname, {})
                        if b.get('var_mode') == 'angles' and vd.get('angles'):
                            # Modo ángulos: R/P/Yw no intervienen (ver _update_ori_enabled).
                            out.append(f"{pad}mover_articulaciones(*{vname}_angulos, traj='{traj}', vel_pct={vel_pct})")
                        else:
                            out.append(
                                f"{pad}mover_a_ori(*{vname}, "
                                f"{roll:.1f}, {pitch:.1f}, {yaw:.1f}, traj='{traj}', vel_pct={vel_pct})")
                    else:
                        x, y, z = b.get('x', 0), b.get('y', 0), b.get('z', 200)
                        out.append(
                            f"{pad}mover_a_ori({x:.1f}, {y:.1f}, {z:.1f}, "
                            f"{roll:.1f}, {pitch:.1f}, {yaw:.1f}, traj='{traj}', vel_pct={vel_pct})")
                elif t == 'wait':
                    out.append(f"{pad}pausa({b.get('seconds', 1.0)})")
                elif t == 'home':
                    out.append(f"{pad}ir_a_home()")
                elif t == 'vacuum':
                    out.append(
                        f"{pad}bomba_vacio({'True' if b.get('state') else 'False'})")
                elif t == 'cinta':
                    out.append(
                        f"{pad}cinta_transportadora({'True' if b.get('state') else 'False'}, "
                        f"vel_pct={b.get('vel_pct', 50)})")
                elif t == 'for':
                    out.append(f"{pad}for _ in range({b.get('n', 3)}):")
                    body = _gen(b.get('body', []), depth + 1)
                    out.extend(body if body else [f"{pad}    pass"])
                elif t == 'while_true':
                    out.append(f"{pad}while True:")
                    body = _gen(b.get('body', []), depth + 1)
                    out.extend(body if body else [f"{pad}    pass"])
                elif t in ('while_cond', 'if', 'if_else'):
                    j = b.get('cond_joint', 'J1')
                    op = b.get('cond_op', '>')
                    val = b.get('cond_val', 0)
                    kw = 'while' if t == 'while_cond' else 'if'
                    if j == 'Pieza (cinta)':
                        cond_py = 'objeto_detectado' if val >= 0.5 else 'not objeto_detectado'
                        out.append(f"{pad}{kw} {cond_py}:  # Pieza (cinta) {'detectada' if val >= 0.5 else 'no detectada'}")
                    else:
                        try:
                            jidx = int(j[1]) - 1
                        except (IndexError, ValueError):
                            jidx = 0
                        out.append(f"{pad}{kw} angulos[{jidx}] {op} {val}:"
                                   f"  # {j} {op} {val}")
                    body = _gen(b.get('body', []), depth + 1)
                    out.extend(body if body else [f"{pad}    pass"])
                    if t == 'if_else':
                        out.append(f"{pad}else:")
                        ebody = _gen(b.get('else_body', []), depth + 1)
                        out.extend(ebody if ebody else [f"{pad}    pass"])
                elif t == 'subroutine':
                    sname = b.get('name', 'desconocida')
                    safe = sname.replace(' ', '_').replace('-', '_')
                    out.append(f"{pad}{safe}()  # subrutina: {sname}")
            return out

        lines.append('# ── Rutina principal ──────────────────────────────────')
        lines.append('def rutina():')
        body_lines = _gen(blocks)
        lines.extend(body_lines if body_lines else ['    pass'])
        lines += ['', '',
                  'if __name__ == "__main__":',
                  '    sys.exit(main())',
                  '    angulos = [0.0] * 6  # ángulos actuales en grados',
                  '    objeto_detectado = False  # True/False según el sensor de la cinta',
                  '    rutina()']

        code = '\n'.join(lines) + '\n'
        try:
            with open(fname, 'w', encoding='utf-8') as f:
                f.write(code)
            self._log_error(f'Exportado: {os.path.basename(fname)}', 'INFO')
            QMessageBox.information(self, 'Exportado',
                                    f'Rutina exportada a:\n{fname}')
        except Exception as e:
            self._log_error(f'Export error: {e}', 'ERROR')
            QMessageBox.critical(self, 'Error al exportar', str(e))

    def _collect_preview_pts(self, blocks, q_start, position_vars, call_stack, pts, max_pts=300):
        """Recorre bloques y acumula puntos XYZ (sin mover el brazo real)."""
        q = list(q_start)
        for block in blocks:
            if len(pts) >= max_pts:
                return q
            t = block.get('type', '')
            if t == 'move':
                q_new = None
                if 'var' in block:
                    vd = position_vars.get(block['var'])
                    if not vd:
                        continue
                    if block.get('var_mode') == 'angles' and vd.get('angles'):
                        # Sin IK: el destino ya son estos 6 ángulos.
                        q_new = list(vd['angles'])
                    else:
                        x, y, z = vd.get('x',0), vd.get('y',0), vd.get('z',200)
                else:
                    x, y, z = block.get('x',0), block.get('y',0), block.get('z',200)
                if q_new is None:
                    q_new, _, _, ok = ik_dls([x, y, z], q, self.rotation_signs_mdh)
                    if not ok:
                        continue
                q = list(q_new)
                thetas = np.deg2rad(np.array(q, dtype=float))
                T = T_MDH_TO_FUSION @ cinematica_directa(thetas, self.rotation_signs_mdh)
                # *100 (no *1000): la malla del brazo y rotation_origins
                # estan en cm: el overlay 3D debe usar la misma escala.
                pts.append([T[0,3]*100, T[1,3]*100, T[2,3]*100])
            elif t == 'home':
                q = [0.0]*6
                thetas = np.deg2rad(np.array(q, dtype=float))
                T = T_MDH_TO_FUSION @ cinematica_directa(thetas, self.rotation_signs_mdh)
                pts.append([T[0,3]*100, T[1,3]*100, T[2,3]*100])
            elif t == 'for':
                for _ in range(min(block.get('n',1), 5)):
                    if len(pts) >= max_pts: break
                    q = self._collect_preview_pts(
                        block.get('body',[]), q, position_vars, call_stack, pts, max_pts)
            elif t in ('while_true', 'while_cond'):
                # preview solo 1 iteración
                q = self._collect_preview_pts(
                    block.get('body',[]), q, position_vars, call_stack, pts, max_pts)
            elif t in ('if', 'if_else'):
                q = self._collect_preview_pts(
                    block.get('body',[]), q, position_vars, call_stack, pts, max_pts)
                if t == 'if_else':
                    q = self._collect_preview_pts(
                        block.get('else_body',[]), q, position_vars, call_stack, pts, max_pts)
            elif t == 'subroutine':
                sub_name = block.get('name','')
                if sub_name and sub_name not in call_stack:
                    sub_path = os.path.join(RUTINAS_DIR, sub_name + '.json')
                    try:
                        with open(sub_path,'r',encoding='utf-8') as _f:
                            sub_data = json.load(_f)
                        merged_vars = dict(position_vars)
                        if isinstance(sub_data, dict):
                            merged_vars.update(sub_data.get('variables',{}))
                            sub_blocks = sub_data.get('blocks', [])
                        else:
                            sub_blocks = sub_data
                        call_stack.add(sub_name)
                        q = self._collect_preview_pts(
                            sub_blocks, q, merged_vars, call_stack, pts, max_pts)
                        call_stack.discard(sub_name)
                    except Exception:
                        pass
        return q

    def _preview_routine(self):
        """Dibuja en el visor 3D la trayectoria prevista sin mover el brazo."""
        blocks = self._prog_container.get_data()
        if not blocks:
            QMessageBox.information(self, 'Preview', 'No hay bloques en la rutina.')
            return
        pts = []
        thetas0 = np.deg2rad(np.array(self.angles, dtype=float))
        T0 = T_MDH_TO_FUSION @ cinematica_directa(thetas0, self.rotation_signs_mdh)
        pts.append([T0[0,3]*100, T0[1,3]*100, T0[2,3]*100])
        self._collect_preview_pts(
            blocks, list(self.angles), dict(self._position_vars), set(), pts)
        print(f'[Preview] {len(pts)} puntos: {pts}')
        if len(pts) < 2:
            self._log_error('Preview: sin puntos de movimiento válidos', 'WARN')
            QMessageBox.information(self, 'Preview',
                'No se encontraron bloques "Mover a" con IK válida.')
            return
        # Limpiar preview anterior
        for attr in ('_preview_actor', '_preview_spheres_actor'):
            actor = getattr(self, attr, None)
            if actor is not None:
                try:
                    self.plotter_widget.remove_actor(actor)
                except Exception:
                    pass
                setattr(self, attr, None)
        # Dibujar polilínea — mismo patrón que _traj_update_actor
        arr = np.array(pts, dtype=float)
        n = len(arr)
        poly = pv.PolyData()
        poly.points = arr
        cells = np.concatenate([[n], np.arange(n)]).astype(np.int_)
        poly.lines = cells
        # Línea
        try:
            line_mesh = pv.lines_from_points(arr)
        except Exception:
            line_mesh = poly
        self._preview_actor = self.plotter_widget.add_mesh(
            line_mesh, color='#ffff00', line_width=6)
        # Esferas en waypoints
        self._preview_spheres = []
        for pt in arr:
            sp = pv.Sphere(radius=6.0, center=pt.tolist())
            a = self.plotter_widget.add_mesh(sp, color='#ffff00', opacity=0.9)
            self._preview_spheres.append(a)
        self.plotter_widget.render()
        print(f'[Preview] actor={self._preview_actor}  pts={arr[:3]}')
        self._log_error(f'Preview: {n} waypoints en visor 3D', 'INFO')

    def _toggle_workspace(self, checked):
        if checked:
            self._ws_btn.setText('⏳ Calculando…')
            self._ws_btn.setEnabled(False)
            QApplication.processEvents()
            self._compute_workspace()
            self._ws_btn.setEnabled(True)
            self._ws_btn.setText('🌐 Envolvente ON')
        else:
            if self._workspace_actor is not None:
                try:
                    self.plotter_widget.remove_actor(self._workspace_actor)
                    self.plotter_widget.render()
                except Exception:
                    pass
                self._workspace_actor = None
            self._ws_btn.setText('🌐 Envolvente')

    def _compute_workspace(self):
        """Muestrea J1–J3 en pasos de 15° y dibuja la nube de puntos alcanzables.
        Descarta cualquier combinación que caiga en la zona prohibida de
        alguna articulación (JOINT_FORBIDDEN_ZONES) -- sin este filtro la
        envolvente mostraba puntos geométricamente alcanzables pero que el
        robot real nunca puede visitar porque el firmware/la IK los rechazan,
        dando una forma de esfera "completa" engañosa."""
        import itertools
        step = 15
        j1_range = range(-180, 181, step)
        j2_range = range(-120, 121, step)
        j3_range = range(-120, 121, step)
        pts = []
        skipped = 0
        for a1, a2, a3 in itertools.product(j1_range, j2_range, j3_range):
            q_deg = [a1, a2, a3, 0.0, 0.0, 0.0]
            if not _q_respects_joint_limits(q_deg):
                skipped += 1
                continue
            # update_3d_visualization() gira cada pieza con
            # rotation_signs_visual, no con rotation_signs_mdh (apartado 3.5.1
            # de la memoria: son convenios independientes). Si aquí se pasa
            # q_deg tal cual a cinematica_directa(), la nube de puntos sale
            # calculada con el signo de calibración cinemática y no coincide
            # con el sentido en que realmente gira el brazo en el visor 3D
            # (para J3 los dos signos son opuestos), dando una envolvente en
            # espejo. Se corrige aplicando aquí el mismo signo visual antes
            # de convertir a radianes, para que la envolvente gire igual que
            # el modelo 3D que se ve en pantalla.
            q_deg_visual = [v * s for v, s in zip(q_deg, self.rotation_signs_visual)]
            q = np.deg2rad(q_deg_visual)
            try:
                T = T_MDH_TO_FUSION @ cinematica_directa(
                    q, self.rotation_signs_mdh)
                pts.append([T[0,3]*100, T[1,3]*100, T[2,3]*100])
            except Exception:
                pass
        if len(pts) < 4:
            self._log_error('Envolvente: no hay puntos', 'WARN')
            return
        arr = np.array(pts, dtype=float)
        try:
            # Limpiar actor previo
            if self._workspace_actor is not None:
                try:
                    self.plotter_widget.remove_actor(self._workspace_actor)
                except Exception:
                    pass
            cloud = pv.PolyData(arr)
            # Convex hull via Delaunay 3D → superficie exterior
            vol = cloud.delaunay_3d()
            hull = vol.extract_surface()
            self._workspace_actor = self.plotter_widget.add_mesh(
                hull,
                color='#3399ff',
                opacity=0.12,
                show_edges=False)
            self.plotter_widget.render()
            self._log_error(
                f'Envolvente: {len(pts)} puntos ({skipped} descartados por zona '
                f'prohibida), hull {hull.n_points} vértices', 'INFO')
        except Exception as e:
            self._log_error(f'Envolvente error: {e}', 'ERROR')
    def _build_rutinas_tab(self):
        w = QWidget()
        outer = QVBoxLayout(w)
        outer.setContentsMargins(2, 2, 2, 2)
        outer.setSpacing(2)

        # ── Barra superior ──────────────────────────────────────────────────
        tb = QHBoxLayout()
        btn_add_curr = QPushButton('📍 Añadir posición actual')
        btn_clear    = QPushButton('🗑 Limpiar todo')
        btn_save     = QPushButton('💾 Guardar rutina')
        btn_load     = QPushButton('📂 Abrir rutina')
        btn_add_curr.setStyleSheet(
            'border-color:#70e570; color:#70e570; padding:3px 8px;')
        btn_clear.setStyleSheet(
            'border-color:#ff6b6b; color:#ff6b6b; padding:3px 8px;')
        btn_save.setStyleSheet(
            'border-color:#5588ff; color:#5588ff; padding:3px 8px;')
        btn_load.setStyleSheet(
            'border-color:#ffaa33; color:#ffaa33; padding:3px 8px;')
        btn_diag     = QPushButton('Diagrama')
        btn_diag.setStyleSheet(
            'border-color:#cc88ff; color:#cc88ff; padding:3px 8px;')
        btn_add_curr.clicked.connect(self._prog_add_current_pos)
        btn_clear.clicked.connect(self._prog_clear)
        btn_save.clicked.connect(self._save_rutina)
        btn_load.clicked.connect(self._load_rutina)
        btn_diag.clicked.connect(self._show_flowchart)
        tb.addWidget(btn_add_curr)
        tb.addWidget(btn_clear)
        tb.addStretch()
        tb.addWidget(btn_save)
        tb.addWidget(btn_load)
        tb.addWidget(btn_diag)
        btn_preview = QPushButton('👁 Preview')
        btn_preview.setToolTip('Vista previa de la trayectoria en 3D')
        btn_preview.setStyleSheet(
            'border-color:#00cccc; color:#00cccc; padding:3px 8px;')
        btn_preview.clicked.connect(self._preview_routine)
        tb.addWidget(btn_preview)
        btn_export_py = QPushButton('🐍 Exportar')
        btn_export_py.setToolTip('Exportar rutina como script Python')
        btn_export_py.setStyleSheet(
            'border-color:#aaccff; color:#aaccff; padding:3px 8px;')
        btn_export_py.clicked.connect(self._export_python)
        tb.addWidget(btn_export_py)
        outer.addLayout(tb)

        # -- Fila: Simulacion / Teach mode / Deshacer-Rehacer --------------------
        tb2 = QHBoxLayout()
        self._dry_run_btn = QPushButton('Simulacion: OFF')
        self._dry_run_btn.setCheckable(True)
        self._dry_run_btn.setToolTip(
            'Modo simulacion: ejecuta la rutina solo en el visor 3D, '
            'sin enviar nada al hardware (util para probar antes de mover el brazo real)')
        self._dry_run_btn.setStyleSheet(
            'QPushButton { border:1px solid #888; color:#aaa; padding:3px 8px; border-radius:4px; }'
            'QPushButton:checked { border-color:#ffaa33; color:#ffaa33; background:#2a1f0d; }')
        self._dry_run_btn.clicked.connect(self._toggle_dry_run)
        tb2.addWidget(self._dry_run_btn)

        self._teach_btn = QPushButton('Teach: OFF')
        self._teach_btn.setCheckable(True)
        self._teach_btn.setToolTip(
            'Modo Teach: activa Mover libre y permite capturar la pose actual '
            'como un bloque de rutina con el boton "Capturar punto"')
        self._teach_btn.setStyleSheet(
            'QPushButton { border:1px solid #888; color:#aaa; padding:3px 8px; border-radius:4px; }'
            'QPushButton:checked { border-color:#dd99ff; color:#dd99ff; background:#220033; }')
        self._teach_btn.clicked.connect(self._toggle_teach_mode)
        tb2.addWidget(self._teach_btn)

        self._teach_capture_btn = QPushButton('Capturar punto')
        self._teach_capture_btn.setEnabled(False)
        self._teach_capture_btn.setStyleSheet(
            'border-color:#dd99ff; color:#dd99ff; padding:3px 8px;')
        self._teach_capture_btn.clicked.connect(self._teach_capture_point)
        tb2.addWidget(self._teach_capture_btn)

        tb2.addStretch()

        self._undo_btn = QPushButton('Deshacer')
        self._redo_btn = QPushButton('Rehacer')
        self._undo_btn.setEnabled(False)
        self._redo_btn.setEnabled(False)
        self._undo_btn.setStyleSheet('border-color:#5588ff; color:#5588ff; padding:3px 8px;')
        self._redo_btn.setStyleSheet('border-color:#5588ff; color:#5588ff; padding:3px 8px;')
        self._undo_btn.clicked.connect(self._prog_undo)
        self._redo_btn.clicked.connect(self._prog_redo)
        tb2.addWidget(self._undo_btn)
        tb2.addWidget(self._redo_btn)
        QShortcut(QKeySequence('Ctrl+Z'), self).activated.connect(self._prog_undo)
        QShortcut(QKeySequence('Ctrl+Shift+Z'), self).activated.connect(self._prog_redo)
        outer.addLayout(tb2)

        # ── Panel de Historial de Posiciones ────────────────────────────────
        hist_frame = QFrame()
        hist_frame.setStyleSheet(
            'QFrame { background:#0d1118; border:1px solid #1e2a1e;'
            ' border-radius:4px; }')
        hist_outer = QVBoxLayout(hist_frame)
        hist_outer.setContentsMargins(3, 2, 3, 2)
        hist_outer.setSpacing(2)

        hist_hdr = QHBoxLayout()
        hist_hdr.setSpacing(2)
        self._hist_collapse_btn = QToolButton()
        self._hist_collapse_btn.setArrowType(Qt.RightArrow)
        self._hist_collapse_btn.setStyleSheet(
            'QToolButton { border:none; color:#557755; background:transparent; }')
        self._hist_collapse_btn.setFixedSize(10, 10)
        hist_hdr.addWidget(self._hist_collapse_btn)
        hist_hdr_lbl = QLabel('🕓  HISTORIAL DE POSICIONES')
        hist_hdr_lbl.setStyleSheet(
            'color:#558855; font-size:9px; font-weight:bold;'
            ' background:transparent; border:none;')
        hist_hdr.addWidget(hist_hdr_lbl)
        hist_hdr.addStretch()
        self._pos_hist_count_lbl = QLabel('(0)')
        self._pos_hist_count_lbl.setStyleSheet(
            'color:#334433; font-size:8px; background:transparent; border:none;')
        hist_hdr.addWidget(self._pos_hist_count_lbl)
        btn_hist_clear = QPushButton('🗑')
        btn_hist_clear.setFixedSize(12, 11)
        btn_hist_clear.setToolTip('Limpiar historial')
        btn_hist_clear.setStyleSheet(
            'QPushButton { background:#0d1a0d; color:#557755; border:1px solid #2a4a2a;'
            ' border-radius:3px; font-size:9px; padding:0; }'
            'QPushButton:hover { background:#1a3a1a; }')
        btn_hist_clear.clicked.connect(lambda: (
            self._pos_history.clear(), self._rebuild_pos_hist_rows()))
        hist_hdr.addWidget(btn_hist_clear)
        hist_outer.addLayout(hist_hdr)

        self._hist_rows_widget = QWidget()
        self._hist_rows_widget.setStyleSheet('background:transparent; border:none;')
        self._pos_hist_rows_layout = QVBoxLayout(self._hist_rows_widget)
        self._pos_hist_rows_layout.setContentsMargins(0, 1, 0, 1)
        self._pos_hist_rows_layout.setSpacing(1)

        # Scroll vertical: con muchas entradas, esto evita que el panel
        # empuje el resto de la pestaña hacia abajo — se queda a una altura
        # fija y el listado se desplaza dentro. Altura reducida respecto a
        # la versión de escritorio: pantalla kiosco de solo 480 px de alto.
        self._hist_scroll = QScrollArea()
        self._hist_scroll.setWidgetResizable(True)
        self._hist_scroll.setWidget(self._hist_rows_widget)
        self._hist_scroll.setMaximumHeight(90)
        self._hist_scroll.setStyleSheet(
            'QScrollArea { background:transparent; border:none; }'
            'QScrollBar:vertical { background:#0d1118; width:8px; }'
            'QScrollBar::handle:vertical { background:#2a4a2a; border-radius:4px; }')
        self._hist_scroll.setVisible(False)
        hist_outer.addWidget(self._hist_scroll)

        def _toggle_hist():
            vis = self._hist_scroll.isVisible()
            self._hist_scroll.setVisible(not vis)
            self._hist_collapse_btn.setArrowType(
                Qt.DownArrow if not vis else Qt.RightArrow)
        self._hist_collapse_btn.clicked.connect(_toggle_hist)
        hist_hdr_lbl.mousePressEvent = lambda e: _toggle_hist()

        outer.addWidget(hist_frame)

        # ── Panel de Variables de Posición ────────────────────────────────
        var_frame = QFrame()
        var_frame.setStyleSheet(
            'QFrame { background:#0d1118; border:1px solid #2a3345;'
            ' border-radius:4px; }')
        var_outer = QVBoxLayout(var_frame)
        var_outer.setContentsMargins(3, 2, 3, 2)
        var_outer.setSpacing(2)

        # Cabecera del panel
        var_hdr = QHBoxLayout()
        var_hdr.setSpacing(2)
        self._var_collapse_btn = QToolButton()
        self._var_collapse_btn.setArrowType(Qt.RightArrow)
        self._var_collapse_btn.setStyleSheet(
            'QToolButton { border:none; color:#6688aa; background:transparent; }')
        self._var_collapse_btn.setFixedSize(10, 10)
        var_hdr.addWidget(self._var_collapse_btn)
        var_hdr_lbl = QLabel('🔖  VARIABLES DE POSICIÓN')
        var_hdr_lbl.setStyleSheet(
            'color:#7799bb; font-size:9px; font-weight:bold;'
            ' background:transparent; border:none;')
        var_hdr.addWidget(var_hdr_lbl)
        var_hdr.addStretch()
        self._var_count_lbl = QLabel('(0)')
        self._var_count_lbl.setStyleSheet(
            'color:#556677; font-size:8px; background:transparent; border:none;')
        var_hdr.addWidget(self._var_count_lbl)
        btn_new_var = QPushButton('+ Nueva')
        btn_new_var.setFixedHeight(11)
        btn_new_var.setStyleSheet(
            'QPushButton { background:#0d1a2e; color:#5599cc; border:1px solid #336699;'
            ' border-radius:3px; font-size:8px; font-weight:bold; padding:1px 5px; }'
            'QPushButton:hover { background:#1a2a40; color:#88bbff; }')
        btn_new_var.clicked.connect(self._add_position_var)
        var_hdr.addWidget(btn_new_var)
        var_outer.addLayout(var_hdr)

        # Contenedor de filas de variables (colapsable)
        self._var_rows_widget = QWidget()
        self._var_rows_widget.setStyleSheet('background:transparent; border:none;')
        self._var_rows_layout = QVBoxLayout(self._var_rows_widget)
        self._var_rows_layout.setContentsMargins(0, 1, 0, 1)
        self._var_rows_layout.setSpacing(1)

        # Scroll vertical (ver comentario análogo en el historial de arriba).
        self._var_scroll = QScrollArea()
        self._var_scroll.setWidgetResizable(True)
        self._var_scroll.setWidget(self._var_rows_widget)
        self._var_scroll.setMaximumHeight(110)
        self._var_scroll.setStyleSheet(
            'QScrollArea { background:transparent; border:none; }'
            'QScrollBar:vertical { background:#0d1118; width:8px; }'
            'QScrollBar::handle:vertical { background:#336699; border-radius:4px; }')
        self._var_scroll.setVisible(False)
        var_outer.addWidget(self._var_scroll)

        def _toggle_var_panel():
            vis = self._var_scroll.isVisible()
            self._var_scroll.setVisible(not vis)
            self._var_collapse_btn.setArrowType(
                Qt.DownArrow if not vis else Qt.RightArrow)
        self._var_collapse_btn.clicked.connect(_toggle_var_panel)
        var_hdr_lbl.mousePressEvent = lambda e: _toggle_var_panel()

        outer.addWidget(var_frame)

        # ── Área de bloques (scroll) ────────────────────────────────────────
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet(
            'QScrollArea { background:#0f1115; border:1px solid #333; }'
            'QScrollBar:vertical { background:#1a1d21; width:8px; }'
            'QScrollBar::handle:vertical { background:#444; border-radius:4px; }')
        self._prog_container = BlockContainer(self)
        scroll.setWidget(self._prog_container)
        scroll.setMinimumHeight(168)
        outer.addWidget(scroll, 1)

        # ── Run / Stop / Cancelar ────────────────────────────────────────────
        run_row = QHBoxLayout()
        self._prog_run_btn    = QPushButton('▶  Run')
        self._prog_stop_btn   = QPushButton('⏸  Stop')
        self._prog_cancel_btn = QPushButton('⛔ Cancelar')
        self._prog_run_btn.setStyleSheet(
            'border:2px solid #70e570; color:#70e570; font-weight:bold;'
            ' padding:4px 14px; border-radius:6px;')
        self._prog_stop_btn.setStyleSheet(
            'border:2px solid #ffcc44; color:#ffcc44; font-weight:bold;'
            ' padding:4px 14px; border-radius:6px;')
        self._prog_stop_btn.setToolTip(
            'Congela la rutina donde está (movimiento o retardo en curso\n'
            'se pausan tal cual) — pulsa "Run" para reanudar exactamente\n'
            'desde donde se quedó.')
        self._prog_stop_btn.setEnabled(False)
        self._prog_cancel_btn.setStyleSheet(
            'border:2px solid #ff6b6b; color:#ff6b6b; font-weight:bold;'
            ' padding:4px 14px; border-radius:6px;')
        self._prog_cancel_btn.setToolTip(
            'Aborta la rutina por completo (esté corriendo o en pausa).\n'
            'El robot se queda parado donde esté hasta que lo muevas\n'
            'manualmente (mando o aplicación).')
        self._prog_cancel_btn.setEnabled(False)
        self._prog_status = QLabel('Detenido')
        self._prog_status.setStyleSheet(
            'font-size:9px; font-weight:bold; color:#aaaaaa;')
        self._prog_run_btn.clicked.connect(self._run_program)
        self._prog_stop_btn.clicked.connect(self._pause_program)
        self._prog_cancel_btn.clicked.connect(self._cancel_program)
        self._traj_btn = QPushButton('⚫ Traza')
        self._traj_btn.setCheckable(True)
        self._traj_btn.setToolTip('Grabar traza del extremo durante la rutina')
        self._traj_btn.setStyleSheet(
            'QPushButton { border:1px solid #555; color:#888; font-weight:bold;'
            ' padding:4px 10px; border-radius:6px; }'
            'QPushButton:checked { border-color:#ff4488; color:#ff4488; }'
            'QPushButton:hover { background:#1a1a2a; }')
        self._traj_btn.clicked.connect(self._traj_toggle)

        timeout_lbl = QLabel('Timeout llegada:')
        timeout_lbl.setStyleSheet('color:#888; font-size:9px; background:transparent;')
        timeout_lbl.setToolTip(
            'Tiempo máximo que espera cada bloque de movimiento a que el robot\n'
            'confirme que llegó (estable, dentro de tolerancia) antes de avisar\n'
            'en el log y seguir de todas formas con el siguiente bloque.')
        self._prog_timeout_spin = QDoubleSpinBox()
        self._prog_timeout_spin.setRange(1.0, 300.0)
        self._prog_timeout_spin.setDecimals(0)
        self._prog_timeout_spin.setSuffix(' s')
        self._prog_timeout_spin.setValue(self._prog_settle_timeout_s)
        self._prog_timeout_spin.setFixedWidth(54)
        self._prog_timeout_spin.setToolTip(timeout_lbl.toolTip())
        self._prog_timeout_spin.setStyleSheet(
            'QDoubleSpinBox { background:#0d1014; color:#e0e0e0; border:1px solid #444;'
            ' border-radius:3px; padding:1px 3px; font-size:9px; }')
        self._prog_timeout_spin.valueChanged.connect(self._on_prog_timeout_changed)

        run_row.addWidget(self._prog_run_btn)
        run_row.addWidget(self._prog_stop_btn)
        run_row.addWidget(self._prog_cancel_btn)
        run_row.addWidget(self._prog_status)
        run_row.addStretch()
        run_row.addWidget(timeout_lbl)
        run_row.addWidget(self._prog_timeout_spin)
        run_row.addWidget(self._traj_btn)
        outer.addLayout(run_row)

        arrival_row = QHBoxLayout()
        tol_lbl = QLabel('Tolerancia llegada:')
        tol_lbl.setStyleSheet('color:#888; font-size:9px; background:transparent;')
        tol_lbl.setToolTip(
            'Error angular máximo (respecto al objetivo) permitido para\n'
            'considerar que el robot ya llegó al punto del bloque actual.')
        self._prog_tol_spin = QDoubleSpinBox()
        self._prog_tol_spin.setRange(0.1, 5.0)
        self._prog_tol_spin.setDecimals(1)
        self._prog_tol_spin.setSingleStep(0.1)
        self._prog_tol_spin.setSuffix('°')
        self._prog_tol_spin.setValue(self._PROG_MOVEMENT_TOL_DEG)
        self._prog_tol_spin.setFixedWidth(54)
        self._prog_tol_spin.setToolTip(tol_lbl.toolTip())
        self._prog_tol_spin.setStyleSheet(
            'QDoubleSpinBox { background:#0d1014; color:#e0e0e0; border:1px solid #444;'
            ' border-radius:3px; padding:1px 3px; font-size:9px; }')
        self._prog_tol_spin.valueChanged.connect(self._on_prog_tol_changed)

        stable_lbl = QLabel('Ventana estable:')
        stable_lbl.setStyleSheet('color:#888; font-size:9px; background:transparent;')
        stable_lbl.setToolTip(
            'Tiempo seguido que el robot debe permanecer quieto y dentro de\n'
            'tolerancia antes de dar el bloque por llegado y pasar al siguiente.\n'
            'Más bajo = transiciones más rápidas pero más riesgo de avanzar\n'
            'con el robot aún acercándose lentamente al punto.')
        self._prog_stable_spin = QDoubleSpinBox()
        self._prog_stable_spin.setRange(0.1, 3.0)
        self._prog_stable_spin.setDecimals(2)
        self._prog_stable_spin.setSingleStep(0.05)
        self._prog_stable_spin.setSuffix(' s')
        self._prog_stable_spin.setValue(self._prog_stable_window_s)
        self._prog_stable_spin.setFixedWidth(54)
        self._prog_stable_spin.setToolTip(stable_lbl.toolTip())
        self._prog_stable_spin.setStyleSheet(
            'QDoubleSpinBox { background:#0d1014; color:#e0e0e0; border:1px solid #444;'
            ' border-radius:3px; padding:1px 3px; font-size:9px; }')
        self._prog_stable_spin.valueChanged.connect(self._on_prog_stable_changed)

        arrival_row.addWidget(tol_lbl)
        arrival_row.addWidget(self._prog_tol_spin)
        arrival_row.addSpacing(10)
        arrival_row.addWidget(stable_lbl)
        arrival_row.addWidget(self._prog_stable_spin)
        arrival_row.addStretch()
        outer.addLayout(arrival_row)

        return w

    def _refresh_prog_size(self):
        if hasattr(self, '_prog_container'):
            self._prog_container.adjustSize()
            self._prog_container.updateGeometry()

    def _prog_add_current_pos(self):
        """Añade un bloque 'Mover a' con la posición XYZ actual (nivel raíz)."""
        self._prog_container._add_current_pos()

    def _show_flowchart(self):
        """Abre el diálogo de diagrama de flujo para la rutina actual."""
        blocks = self._prog_container.get_data()
        if not blocks:
            QMessageBox.information(self, 'Sin bloques',
                                    'Anade bloques a la rutina para ver el diagrama.')
            return
        dlg = FlowchartDialog(blocks, self)
        dlg.exec_()

    # ── Variables de posición ────────────────────────────────────────────

    def _add_position_var(self, name='', x=0.0, y=0.0, z=0.0, traj='Articular',
                           angles=None, vel_pct=100.0):
        """Añade una nueva variable de posición y crea su fila en el panel.
        'angles' (6 grados articulares) es opcional: solo lo rellena el botón
        de captura (📍), que guarda a la vez XYZ y los ángulos de ESA misma
        pose — así el modo "ángulos directos" de Mover a puede reproducirla
        exactamente, sin pasar por IK. 'vel_pct' es la velocidad (10-100%) a
        la que se moverá el robot al usar los botones 🎯/🦾 de esta fila."""
        # Nombre único por defecto
        if not name:
            i = len(self._position_vars) + 1
            name = f'Pos_{i}'
            while name in self._position_vars:
                i += 1
                name = f'Pos_{i}'
        self._position_vars[name] = {
            'x': x, 'y': y, 'z': z, 'traj': traj,
            'angles': list(angles) if angles is not None else None,
            'vel_pct': vel_pct,
        }
        self._var_scroll.setVisible(True)
        self._var_collapse_btn.setArrowType(Qt.DownArrow)
        self._rebuild_var_rows()
        self._refresh_all_var_combos()

    def _delete_position_var(self, name):
        """Elimina una variable por nombre."""
        self._position_vars.pop(name, None)
        self._rebuild_var_rows()
        self._refresh_all_var_combos()

    def _rebuild_var_rows(self):
        """Reconstruye todas las filas del panel desde _position_vars."""
        # Limpiar filas existentes
        for i in reversed(range(self._var_rows_layout.count())):
            item = self._var_rows_layout.itemAt(i)
            if item and item.widget():
                item.widget().deleteLater()
                self._var_rows_layout.removeItem(item)

        _INP = ('background:#0a0e14; color:#e0e0e0; border:1px solid #2a3040;'
                ' border-radius:3px; padding:1px 3px; font-size:9px;')
        _CMB = ('QComboBox { background:#0a0e14; color:#ccc; border:1px solid #2a3040;'
                ' border-radius:3px; font-size:9px; padding:1px 2px; }'
                'QComboBox::drop-down { width:12px; border:none; }'
                'QComboBox QAbstractItemView { background:#1c1f24; color:#ddd;'
                ' selection-background-color:#2a3a50; border:1px solid #444; }')

        for var_name, vdata in list(self._position_vars.items()):
            row_w = QWidget()
            row_w.setStyleSheet('background:transparent; border:none;')
            outer_lay = QVBoxLayout(row_w)
            outer_lay.setContentsMargins(0, 0, 0, 0)
            outer_lay.setSpacing(0)

            top_row_w = QWidget()
            top_row_w.setStyleSheet('background:transparent; border:none;')
            row_lay = QHBoxLayout(top_row_w)
            row_lay.setContentsMargins(1, 1, 1, 1)
            row_lay.setSpacing(2)

            name_edit = QLineEdit(var_name)
            name_edit.setFixedWidth(54)
            name_edit.setStyleSheet(_INP)
            name_edit.setPlaceholderText('Nombre')

            x_edit = QLineEdit(f"{vdata.get('x', 0):.1f}")
            y_edit = QLineEdit(f"{vdata.get('y', 0):.1f}")
            z_edit = QLineEdit(f"{vdata.get('z', 0):.1f}")
            for w in (x_edit, y_edit, z_edit):
                w.setFixedWidth(31)
                w.setStyleSheet(_INP)

            traj_cmb = QComboBox()
            traj_cmb.addItems(['Articular', 'Lineal', 'Directo', 'Spline'])
            traj_cmb.setFixedWidth(43)
            traj_cmb.setStyleSheet(_CMB)
            idx = traj_cmb.findText(vdata.get('traj', 'Articular'))
            if idx >= 0:
                traj_cmb.setCurrentIndex(idx)

            def _on_name_changed(new_name, old_name=var_name, ne=name_edit):
                new_name = new_name.strip()
                if not new_name or new_name == old_name:
                    return
                if new_name in self._position_vars:
                    ne.setText(old_name)
                    return
                vd = self._position_vars.pop(old_name, {})
                self._position_vars[new_name] = vd
                self._rebuild_var_rows()
                self._refresh_all_var_combos()
            name_edit.editingFinished.connect(
                lambda ne=name_edit, on=var_name: _on_name_changed(ne.text(), on))

            # Indicador: ¿esta variable tiene ángulos guardados? (necesarios
            # para el modo "Ángulos (directo)" del bloque Mover a). Se define
            # ANTES de _on_coord_changed/_do_capture y se les pasa como
            # argumento por defecto (early-bound) — si se referenciara suelta
            # dentro de esas funciones, al estar todas definidas dentro del
            # mismo bucle "for", todas las filas acabarían compartiendo la
            # función de la ÚLTIMA fila (el clásico problema de cierres en
            # un bucle con "late binding").
            ang_lbl = QLabel()
            ang_lbl.setFixedWidth(10)
            ang_lbl.setCursor(Qt.PointingHandCursor)

            # Sub-fila (oculta por defecto) que muestra los 6 ángulos
            # articulares guardados junto a esta variable. Se despliega al
            # pulsar el indicador 🦾 y también se ve reflejada en el tooltip
            # del propio indicador, para poder "verlos" tanto sin abrir nada
            # (hover) como de forma fija (clic) sin ensanchar la fila normal.
            angs_row_w = QWidget()
            angs_row_w.setStyleSheet('background:transparent; border:none;')
            angs_lay = QHBoxLayout(angs_row_w)
            angs_lay.setContentsMargins(13, 0, 1, 2)
            angs_lay.setSpacing(2)

            # 6 campos editables (J1..J6) para poder ajustar a mano el ángulo
            # guardado de cada articulación, sin tener que volver a capturar
            # la posición entera desde el robot.
            ang_edits = []
            for _ji in range(6):
                _jlbl = QLabel(f'J{_ji+1}:')
                _jlbl.setStyleSheet('color:#667788; font-size:7px; background:transparent; border:none;')
                _jedit = QLineEdit()
                _jedit.setFixedWidth(28)
                _jedit.setStyleSheet(
                    'QLineEdit { background:#0f1418; color:#7aa0c4; border:1px solid #2a3238;'
                    ' border-radius:2px; font-size:8px; padding:1px 2px; }')
                angs_lay.addWidget(_jlbl)
                angs_lay.addWidget(_jedit)
                ang_edits.append(_jedit)
            angs_lay.addStretch()
            angs_row_w.setVisible(False)

            def _fmt_angles(angles):
                nombres = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
                return '  '.join(f'{n}:{a:.1f}°' for n, a in zip(nombres, angles))

            def _refresh_ang_lbl(vn=var_name, ne=name_edit, al=ang_lbl,
                                  edits=ang_edits, arw=angs_row_w):
                vn2 = ne.text().strip() or vn
                angles = self._position_vars.get(vn2, {}).get('angles')
                tiene = bool(angles)
                al.setText('🦾' if tiene else '—')
                al.setStyleSheet(
                    ('color:#5aaa5a;' if tiene else 'color:#445055;') +
                    ' font-size:9px; background:transparent; border:none;')
                if tiene:
                    al.setToolTip('Ángulos guardados (clic para mostrar/ocultar y editar):\n'
                                   + _fmt_angles(angles))
                    for i, e in enumerate(edits):
                        if not e.hasFocus():
                            e.setText(f'{angles[i]:.2f}')
                        e.setEnabled(True)
                else:
                    al.setToolTip('Sin ángulos guardados — captura la posición con 📍')
                    arw.setVisible(False)
                    for e in edits:
                        e.clear()
                        e.setEnabled(False)
            _refresh_ang_lbl()

            def _make_angle_edit_handler(idx, ed, vn_ref=[var_name], ne=name_edit,
                                          xe=x_edit, ye=y_edit, ze=z_edit):
                def _on_angle_edit_changed():
                    vn2 = ne.text().strip() or vn_ref[0]
                    vd = self._position_vars.get(vn2)
                    if vd is None or not vd.get('angles'):
                        return
                    try:
                        vd['angles'][idx] = float(ed.text())
                    except ValueError:
                        ed.setText(f"{vd['angles'][idx]:.2f}")
                        return
                    # Al tocar un ángulo a mano, el XYZ mostrado (derivado por
                    # cinemática directa) ya no correspondía a la pose real —
                    # se recalcula aquí para que ambos queden consistentes.
                    try:
                        import numpy as np
                        thetas = np.deg2rad(np.array(vd['angles'], dtype=float))
                        T = T_MDH_TO_FUSION @ cinematica_directa(
                            thetas, self.rotation_signs_mdh)
                        xv, yv, zv = T[0, 3]*1000, T[1, 3]*1000, T[2, 3]*1000
                        vd['x'], vd['y'], vd['z'] = xv, yv, zv
                        xe.setText(f'{xv:.1f}')
                        ye.setText(f'{yv:.1f}')
                        ze.setText(f'{zv:.1f}')
                    except Exception as ex:
                        print(f'[Vars] Error recalculando XYZ tras editar ángulo: {ex}')
                return _on_angle_edit_changed
            for _ji, _jedit in enumerate(ang_edits):
                _jedit.editingFinished.connect(_make_angle_edit_handler(_ji, _jedit))

            def _toggle_angs_row(_ev=None, arw=angs_row_w, vn=var_name, ne=name_edit):
                vn2 = ne.text().strip() or vn
                if not self._position_vars.get(vn2, {}).get('angles'):
                    return
                arw.setVisible(not arw.isVisible())
            ang_lbl.mousePressEvent = _toggle_angs_row

            def _on_coord_changed(x_w=x_edit, y_w=y_edit, z_w=z_edit,
                                   tc=traj_cmb, vn_ref=[var_name],
                                   refresh_ang=_refresh_ang_lbl, clear_angles=False):
                vn = name_edit.text().strip() or vn_ref[0]
                if vn in self._position_vars:
                    try:
                        self._position_vars[vn]['x'] = float(x_w.text() or 0)
                        self._position_vars[vn]['y'] = float(y_w.text() or 0)
                        self._position_vars[vn]['z'] = float(z_w.text() or 0)
                        self._position_vars[vn]['traj'] = tc.currentText()
                        if clear_angles:
                            # Si el usuario teclea un XYZ nuevo a mano, los
                            # ángulos guardados de la captura anterior ya NO
                            # corresponden a esta posición — se invalidan para
                            # que el modo "ángulos directos" no lleve al robot
                            # al sitio viejo con el XYZ nuevo en pantalla.
                            self._position_vars[vn]['angles'] = None
                            refresh_ang()
                    except ValueError:
                        pass
            for w in (x_edit, y_edit, z_edit):
                w.editingFinished.connect(
                    lambda w=w: _on_coord_changed(clear_angles=True))
            traj_cmb.currentIndexChanged.connect(lambda _i: _on_coord_changed())

            cap_btn = QPushButton('📍')
            cap_btn.setFixedSize(13, 13)
            cap_btn.setToolTip('Capturar posición actual del robot (XYZ + ángulos)')
            cap_btn.setStyleSheet(
                'QPushButton { border:1px solid #3a6a3a; color:#5aaa5a;'
                ' background:#0d1a0d; border-radius:3px; font-size:9px; padding:0; }'
                'QPushButton:hover { background:#1a3a1a; }')

            goto_vel_spin = QSpinBox()
            goto_vel_spin.setRange(10, 100)
            goto_vel_spin.setValue(int(vdata.get('vel_pct', 100)))
            goto_vel_spin.setSuffix('%')
            goto_vel_spin.setFixedWidth(38)
            goto_vel_spin.setStyleSheet(_INP)
            goto_vel_spin.setToolTip(
                'Velocidad para el movimiento con 🎯/🦾 de esta fila\n'
                '(% de la velocidad máxima configurada en cada articulación)')

            def _on_goto_vel_changed(value, vn_ref=[var_name], ne=name_edit):
                vn = ne.text().strip() or vn_ref[0]
                if vn in self._position_vars:
                    self._position_vars[vn]['vel_pct'] = value
            goto_vel_spin.valueChanged.connect(_on_goto_vel_changed)

            goto_xyz_btn = QPushButton('🎯')
            goto_xyz_btn.setFixedSize(14, 13)
            goto_xyz_btn.setToolTip('Ir a estas coordenadas XYZ (por cinemática inversa)')
            goto_xyz_btn.setStyleSheet(
                'QPushButton { border:1px solid #336699; color:#5599cc;'
                ' background:#0d1a2e; border-radius:3px; font-size:8px; padding:0; }'
                'QPushButton:hover { background:#1a2a40; }')
            goto_xyz_btn.clicked.connect(
                lambda _, vn=var_name, ne=name_edit:
                    self._var_goto_xyz(ne.text().strip() or vn))

            goto_ang_btn = QPushButton('🦾')
            goto_ang_btn.setFixedSize(14, 13)
            goto_ang_btn.setToolTip(
                'Ir directamente a los ángulos articulares guardados\n'
                '(sin pasar por cinemática inversa — reproduce\n'
                'exactamente la pose capturada con 📍)')
            goto_ang_btn.setStyleSheet(
                'QPushButton { border:1px solid #3a6a3a; color:#5aaa5a;'
                ' background:#0d1a0d; border-radius:3px; font-size:8px; padding:0; }'
                'QPushButton:hover { background:#1a3a1a; }')
            goto_ang_btn.clicked.connect(
                lambda _, vn=var_name, ne=name_edit:
                    self._var_goto_angles(ne.text().strip() or vn))

            del_btn = QPushButton('✕')
            del_btn.setFixedSize(12, 12)
            del_btn.setStyleSheet(
                'QPushButton { border:1px solid #5a2020; color:#aa5555;'
                ' background:#1a0d0d; border-radius:3px; font-size:8px; font-weight:bold; padding:0; }'
                'QPushButton:hover { background:#3a1515; }')

            def _do_capture(_checked=False, xe=x_edit, ye=y_edit, ze=z_edit,
                            ne=name_edit, vn_ref=[var_name], refresh_ang=_refresh_ang_lbl):
                try:
                    import numpy as np
                    _angles = (self.real_angles_feedback
                               if getattr(self, '_free_move', False)
                               else self.angles)
                    thetas = np.deg2rad(np.array(_angles, dtype=float))
                    T = T_MDH_TO_FUSION @ cinematica_directa(
                        thetas, self.rotation_signs_mdh)
                    xv, yv, zv = T[0,3]*1000, T[1,3]*1000, T[2,3]*1000
                    xe.setText(f'{xv:.1f}')
                    ye.setText(f'{yv:.1f}')
                    ze.setText(f'{zv:.1f}')
                    vn = ne.text().strip() or vn_ref[0]
                    if vn in self._position_vars:
                        # Guarda XYZ y los 6 ángulos de la MISMA pose a la vez:
                        # así el bloque "Mover a" puede, para esta variable,
                        # llegar por IK (coordenadas) o directo (estos ángulos,
                        # sin pasar por IK) — ver toggle en el bloque.
                        self._position_vars[vn].update(
                            x=xv, y=yv, z=zv,
                            angles=[float(a) for a in _angles],
                        )
                        refresh_ang()
                except Exception as ex:
                    print(f'[Vars] Error capturando: {ex}')
            cap_btn.clicked.connect(_do_capture)

            del_btn.clicked.connect(
                lambda _, vn=var_name: self._delete_position_var(vn))

            lbl_x = QLabel('X:')
            lbl_x.setStyleSheet('color:#667788; font-size:8px; background:transparent; border:none;')
            lbl_y = QLabel('Y:')
            lbl_y.setStyleSheet('color:#667788; font-size:8px; background:transparent; border:none;')
            lbl_z = QLabel('Z:')
            lbl_z.setStyleSheet('color:#667788; font-size:8px; background:transparent; border:none;')

            row_lay.addWidget(name_edit)
            row_lay.addWidget(lbl_x); row_lay.addWidget(x_edit)
            row_lay.addWidget(lbl_y); row_lay.addWidget(y_edit)
            row_lay.addWidget(lbl_z); row_lay.addWidget(z_edit)
            row_lay.addWidget(traj_cmb)
            row_lay.addWidget(cap_btn)
            row_lay.addWidget(ang_lbl)
            row_lay.addWidget(goto_vel_spin)
            row_lay.addWidget(goto_xyz_btn)
            row_lay.addWidget(goto_ang_btn)
            row_lay.addWidget(del_btn)
            row_lay.addStretch()

            outer_lay.addWidget(top_row_w)
            outer_lay.addWidget(angs_row_w)

            self._var_rows_layout.addWidget(row_w)

        count = len(self._position_vars)
        self._var_count_lbl.setText(f'({count})')

    def _sync_position_vars(self):
        """Lee los valores actuales de las filas del panel y actualiza _position_vars."""
        # Los campos se mantienen sincronizados en tiempo real via editingFinished;
        # esta función es solo una garantía adicional antes de guardar/ejecutar.
        pass

    def _refresh_all_var_combos(self):
        """Propaga el listado de variables a todos los bloques move en modo var."""
        if hasattr(self, '_prog_container'):
            self._prog_container.refresh_var_combos()

    def _var_move_guard(self):
        """Impide mover el robot a mano (botones 🎯/🦾 de una variable)
        mientras una rutina está corriendo o en pausa — evita que un
        movimiento manual y el motor de la rutina compitan por el mismo
        robot al mismo tiempo."""
        if getattr(self, '_prog_running', False) or getattr(self, '_prog_paused', False):
            QMessageBox.warning(
                self, 'Rutina en curso',
                'Detén o cancela la rutina antes de mover el robot manualmente.')
            return True
        return False

    def _apply_angles_and_send(self, q_deg, vel_pct=100.0):
        """Aplica unos ángulos articulares al modelo/UI y los envía al robot
        a la velocidad indicada (% de la velocidad máxima configurada en
        cada articulación)."""
        for i in range(6):
            self.angles[i] = float(q_deg[i])
            self.sliders[i].blockSignals(True)
            self.sliders[i].setValue(int(q_deg[i]))
            self.sliders[i].blockSignals(False)
            self.inputs[i].setText(f'{q_deg[i]:.2f}')
            self.labels[i].setText(f'Articulación {i+1}: {q_deg[i]:.2f}°')
        self.update_xyz_display()
        self.update_3d_visualization()
        self.send_setpoints(self.angles, vel_pct=vel_pct)
        self._pos_history_add()

    def _var_goto_xyz(self, name):
        """Mueve el robot a las coordenadas XYZ guardadas en la variable, por
        IK, a la velocidad configurada en esa fila."""
        if self._var_move_guard():
            return
        vd = self._position_vars.get(name)
        if not vd:
            return
        x, y, z = vd.get('x', 0), vd.get('y', 0), vd.get('z', 0)
        q_deg, err_mm, _, ok = ik_dls([x, y, z], self.angles, self.rotation_signs_mdh)
        if not ok:
            QMessageBox.warning(
                self, 'IK sin solución',
                f'No se encontró solución para ({x:.0f}, {y:.0f}, {z:.0f})'
                f' — error {err_mm:.0f} mm.')
            return
        self._apply_angles_and_send(q_deg, vd.get('vel_pct', 100))

    def _var_goto_angles(self, name):
        """Mueve el robot directamente a los ángulos guardados con la variable
        (sin pasar por IK — reproduce exactamente la pose capturada), a la
        velocidad configurada en esa fila."""
        if self._var_move_guard():
            return
        vd = self._position_vars.get(name)
        if not vd or not vd.get('angles'):
            QMessageBox.warning(
                self, 'Sin ángulos guardados',
                'Esta variable no tiene ángulos guardados — captúrala con 📍 primero.')
            return
        self._apply_angles_and_send(vd['angles'], vd.get('vel_pct', 100))

    # ── Trayectoria 3D ───────────────────────────────────────────────────

    def _traj_toggle(self, checked):
        self._traj_tracing = checked
        self._traj_btn.setText('🔴 Traza ON' if checked else '⚫ Traza')
        if not checked:
            self._traj_clear()

    def _traj_add_point(self):
        try:
            thetas = np.deg2rad(np.array(self.angles, dtype=float))
            T = T_MDH_TO_FUSION @ cinematica_directa(thetas, self.rotation_signs_mdh)
            x, y, z = T[0,3]*100, T[1,3]*100, T[2,3]*100
            self._traj_pts.append([x, y, z])
            if len(self._traj_pts) >= 2:
                self._traj_update_actor()
        except Exception as _e:
            pass

    def _traj_update_actor(self):
        try:
            pts = np.array(self._traj_pts, dtype=float)
            n = len(pts)
            if n < 2:
                return
            pd_mesh = pv.PolyData()
            pd_mesh.points = pts
            cells = np.concatenate([[n], np.arange(n)])
            pd_mesh.lines = cells
            if self._traj_actor is not None:
                try:
                    self.plotter_widget.remove_actor(self._traj_actor)
                except Exception:
                    pass
            self._traj_actor = self.plotter_widget.add_mesh(
                pd_mesh, color='#ff4488', line_width=3,
                render_lines_as_tubes=True)
            self.plotter_widget.render()
        except Exception as _e:
            print(f'[Traza] {_e}')

    def _traj_clear(self):
        self._traj_pts.clear()
        if self._traj_actor is not None:
            try:
                self.plotter_widget.remove_actor(self._traj_actor)
                self.plotter_widget.render()
            except Exception:
                pass
            self._traj_actor = None

    @staticmethod
    def _catmull_rom_chain(pts, n_per_seg=6):
        """Catmull-Rom spline through pts. Returns dense list of [x,y,z]."""
        if len(pts) < 2:
            return list(pts)
        chain = [pts[0]] + list(pts) + [pts[-1]]
        result = []
        for i in range(1, len(chain) - 2):
            p0, p1, p2, p3 = (np.array(chain[k]) for k in (i-1, i, i+1, i+2))
            for j in range(n_per_seg):
                t  = j / n_per_seg
                t2 = t * t;  t3 = t2 * t
                pt = 0.5 * ((2*p1)
                            + (-p0 + p2) * t
                            + (2*p0 - 5*p1 + 4*p2 - p3) * t2
                            + (-p0 + 3*p1 - 3*p2 + p3) * t3)
                result.append(pt.tolist())
        result.append(list(pts[-1]))
        return result

    def _preprocess_spline_blocks(self, blocks):
        """Expande secuencias de bloques Spline consecutivos a waypoints Lineal."""
        result = []
        i = 0
        while i < len(blocks):
            b = blocks[i]
            is_spline = (b.get('type') == 'move' and b.get('traj') == 'Spline')
            if is_spline:
                # Collect consecutive Spline blocks
                run = []
                while i < len(blocks):
                    bi = blocks[i]
                    if bi.get('type') == 'move' and bi.get('traj') == 'Spline':
                        run.append(bi)
                        i += 1
                    else:
                        break
                # Start from current FK position
                try:
                    thetas = np.deg2rad(np.array(self.angles, dtype=float))
                    T = T_MDH_TO_FUSION @ cinematica_directa(
                        thetas, self.rotation_signs_mdh)
                    start = [T[0,3]*1000, T[1,3]*1000, T[2,3]*1000]
                except Exception:
                    start = [0.0, 0.0, 200.0]
                waypoints = [start]
                for rb in run:
                    if 'var' in rb:
                        vd = self._position_vars.get(rb['var'], {})
                        waypoints.append([vd.get('x',0), vd.get('y',0), vd.get('z',200)])
                    else:
                        waypoints.append([rb.get('x',0), rb.get('y',0), rb.get('z',200)])
                dense = self._catmull_rom_chain(waypoints, n_per_seg=6)
                for pt in dense[1:]:  # skip starting position
                    result.append({'type':'move', 'x':pt[0], 'y':pt[1],
                                   'z':pt[2], 'traj':'Lineal'})
            else:
                nb = dict(b)
                if 'body' in nb:
                    nb['body'] = self._preprocess_spline_blocks(nb['body'])
                if 'else_body' in nb:
                    nb['else_body'] = self._preprocess_spline_blocks(nb['else_body'])
                result.append(nb)
                i += 1
        return result

    # -- Deshacer / Rehacer (editor de rutinas) ------------------------------
    def _push_undo_snapshot(self):
        """Guarda el estado actual de la rutina antes de una modificacion."""
        try:
            snap = self._prog_container.get_data()
        except Exception:
            return
        self._prog_undo_stack.append(snap)
        if len(self._prog_undo_stack) > self._prog_undo_max:
            self._prog_undo_stack.pop(0)
        self._prog_redo_stack.clear()
        self._update_undo_redo_buttons()

    def _restore_prog_from_data(self, data_list):
        self._prog_container.clear_all()
        for block_data in data_list:
            _block_restore(block_data, self._prog_container, self)
        self._refresh_all_var_combos()

    def _prog_undo(self):
        if not self._prog_undo_stack:
            return
        current = self._prog_container.get_data()
        prev = self._prog_undo_stack.pop()
        self._prog_redo_stack.append(current)
        self._restore_prog_from_data(prev)
        self._update_undo_redo_buttons()

    def _prog_redo(self):
        if not self._prog_redo_stack:
            return
        current = self._prog_container.get_data()
        nxt = self._prog_redo_stack.pop()
        self._prog_undo_stack.append(current)
        self._restore_prog_from_data(nxt)
        self._update_undo_redo_buttons()

    def _update_undo_redo_buttons(self):
        if hasattr(self, '_undo_btn'):
            self._undo_btn.setEnabled(bool(self._prog_undo_stack))
        if hasattr(self, '_redo_btn'):
            self._redo_btn.setEnabled(bool(self._prog_redo_stack))

    # -- Modo simulacion (dry-run) --------------------------------------------
    def _toggle_dry_run(self, checked):
        self._dry_run_mode = checked
        self._dry_run_btn.setText('Simulacion: ON' if checked else 'Simulacion: OFF')
        if checked:
            self._log_error('Modo simulacion activado: la rutina no enviara nada al hardware.', 'INFO')

    # -- Modo Teach (grabar moviendo el brazo a mano) -------------------------
    def _toggle_teach_mode(self, checked):
        self._teach_mode_active = checked
        self._teach_btn.setText('Teach: ON' if checked else 'Teach: OFF')
        self._teach_capture_btn.setEnabled(checked)
        if checked and not self._free_btn.isChecked():
            self._free_btn.setChecked(True)
            self._toggle_free_move(True)
        elif not checked and self._free_btn.isChecked():
            self._free_btn.setChecked(False)
            self._toggle_free_move(False)

    def _teach_capture_point(self):
        """Captura la pose actual (real si Mover libre esta activo) como bloque move_ori."""
        try:
            _angles = (self.real_angles_feedback if self._free_move else self.angles)
            thetas = np.deg2rad(np.array(_angles, dtype=float))
            T = T_MDH_TO_FUSION @ cinematica_directa(thetas, self.rotation_signs_mdh)
            x, y, z = T[0, 3] * 1000, T[1, 3] * 1000, T[2, 3] * 1000
            rpy = np.rad2deg(rpy_from_matrix(T[:3, :3]))
            self._push_undo_snapshot()
            block = self._prog_container.add_block('move_ori')
            block._mx.setText(f'{x:.1f}')
            block._my.setText(f'{y:.1f}')
            block._mz.setText(f'{z:.1f}')
            block._mroll.setText(f'{rpy[0]:.1f}')
            block._mpitch.setText(f'{rpy[1]:.1f}')
            block._myaw.setText(f'{rpy[2]:.1f}')
            self._log_error('Teach: punto capturado y anadido a la rutina.', 'INFO')
        except Exception as e:
            print(f'[Teach] Error capturando punto: {e}')
            try:
                self._log_error(f'Teach: error capturando punto: {e}', 'WARN')
            except Exception:
                pass

    # -- Perfiles de herramienta (TCP offset) ---------------------------------
    def _load_tool_profiles(self):
        default_profiles = {'Sin offset': {'dx': 0, 'dy': 0, 'dz': 0, 'roll': 0, 'pitch': 0, 'yaw': 0}}
        try:
            if os.path.isfile(TOOLS_FILE):
                with open(TOOLS_FILE, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                self._tool_profiles = data.get('profiles', default_profiles)
                self._active_tool_profile = data.get('active', 'Sin offset')
            else:
                self._tool_profiles = default_profiles
                self._active_tool_profile = 'Sin offset'
        except Exception as e:
            print(f'[TCP] Error cargando perfiles de herramienta: {e}')
            self._tool_profiles = default_profiles
            self._active_tool_profile = 'Sin offset'
        if self._active_tool_profile not in self._tool_profiles:
            self._tool_profiles.setdefault('Sin offset', default_profiles['Sin offset'])
            self._active_tool_profile = 'Sin offset'
        self._apply_active_tool_profile()

    def _apply_active_tool_profile(self):
        p = self._tool_profiles.get(self._active_tool_profile,
                                     {'dx': 0, 'dy': 0, 'dz': 0, 'roll': 0, 'pitch': 0, 'yaw': 0})
        try:
            set_tool_offset(p.get('dx', 0), p.get('dy', 0), p.get('dz', 0),
                             p.get('roll', 0), p.get('pitch', 0), p.get('yaw', 0))
        except Exception as e:
            print(f'[TCP] Error aplicando perfil de herramienta: {e}')

    def _save_tool_profiles(self):
        try:
            with open(TOOLS_FILE, 'w', encoding='utf-8') as f:
                json.dump({'profiles': self._tool_profiles,
                           'active': self._active_tool_profile}, f, indent=2, ensure_ascii=False)
        except Exception as e:
            print(f'[TCP] Error guardando perfiles de herramienta: {e}')
            try:
                self._log_error(f'Guardando perfiles TCP: {e}', 'WARN')
            except Exception:
                pass

    def _open_tool_profile_dialog(self):
        dlg = ToolProfileDialog(self)
        dlg.exec_()

    def _prog_clear(self):
        self._push_undo_snapshot()
        self._prog_container.clear_all()
        self._prog_call_stack.clear()

    # ── Guardar / cargar rutinas en JSON ──────────────────────────────────

    def _save_rutina(self):
        """Guarda la rutina actual como JSON en la carpeta 'rutinas/'."""
        blocks = self._prog_container.get_data()
        if not blocks:
            QMessageBox.information(self, "Sin bloques",
                                    "La rutina está vacía. Añade bloques antes de guardar.")
            return
        path, _ = QFileDialog.getSaveFileName(
            self,
            "Guardar rutina",
            RUTINAS_DIR,
            "Rutinas JSON (*.json);;Todos los archivos (*)"
        )
        if not path:
            return
        if not path.lower().endswith('.json'):
            path += '.json'
        try:
            self._sync_position_vars()
            payload = {
                'variables': self._position_vars,
                'blocks': blocks
            }
            with open(path, 'w', encoding='utf-8') as f:
                json.dump(payload, f, indent=2, ensure_ascii=False)
            QMessageBox.information(self, "Guardada",
                                    f"Rutina guardada en:\n{path}")
        except Exception as e:
            QMessageBox.critical(self, "Error al guardar", str(e))

    def _load_rutina(self):
        """Carga una rutina desde un archivo JSON y la muestra en el editor."""
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Abrir rutina",
            RUTINAS_DIR,
            "Rutinas JSON (*.json);;Todos los archivos (*)"
        )
        if not path:
            return
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            # Confirmar si hay bloques actuales
            if self._prog_container.get_data():
                resp = QMessageBox.question(
                    self, "Reemplazar rutina",
                    "¿Reemplazar la rutina actual con la cargada?",
                    QMessageBox.Yes | QMessageBox.No
                )
                if resp != QMessageBox.Yes:
                    return
            # Soporte formato nuevo {variables, blocks} y antiguo [lista de bloques]
            if isinstance(data, dict) and 'blocks' in data:
                variables = data.get('variables', {})
                blocks = data['blocks']
            else:
                variables = {}
                blocks = data  # formato antiguo
            # Cargar variables
            self._position_vars = variables
            self._rebuild_var_rows()
            # Limpiar y reconstruir bloques
            self._push_undo_snapshot()
            self._prog_container.clear_all()
            for block_data in blocks:
                _block_restore(block_data, self._prog_container, self)
            self._refresh_all_var_combos()
            QMessageBox.information(self, "Cargada",
                                    f"Rutina cargada:\n{os.path.basename(path)}")
        except Exception as e:
            QMessageBox.critical(self, "Error al cargar", str(e))

    # ─────────────────────────────────────────────────────────────────────────
    #  Programa por bloques — motor de ejecución
    # ─────────────────────────────────────────────────────────────────────────

    def _on_prog_timeout_changed(self, value):
        self._prog_settle_timeout_s = float(value)

    def _on_prog_tol_changed(self, value):
        self._PROG_MOVEMENT_TOL_DEG = float(value)

    def _on_prog_stable_changed(self, value):
        self._prog_stable_window_s = float(value)

    def _run_program(self):
        if self._prog_paused:
            self._resume_program()
            return
        blocks = self._prog_container.get_data(include_widget=True)
        blocks = self._preprocess_spline_blocks(blocks)
        if not blocks:
            self._prog_status.setText('⚠ Añade bloques primero')
            return
        self._prog_running = True
        self._prog_paused  = False
        self._prog_gen     = self._build_prog_gen(blocks)
        self._prog_state   = 'needs_next'
        self._prog_run_btn.setEnabled(False)
        self._prog_run_btn.setText('▶  Run')
        self._prog_stop_btn.setEnabled(True)
        self._prog_cancel_btn.setEnabled(True)
        self._prog_status.setText('▶ Ejecutando...')
        self._prog_status.setStyleSheet(
            'font-size:9px; font-weight:bold; color:#70e570;')
        self._prog_timer.start(25)

    def _pause_program(self):
        """Botón 'Stop': congela la rutina exactamente donde está. El timer
        deja de avanzar — así un retardo en curso se queda con su cuenta
        atrás congelada, y una animación de movimiento se detiene a medio
        camino — y, si hay robot real conectado, se le manda un setpoint a
        su posición ACTUAL para que el firmware deje de perseguir el
        objetivo anterior y se quede quieto de verdad, no solo en la
        simulación. Reanudable con 'Run' (que pasa a decir 'Reanudar')."""
        if not self._prog_running or self._prog_paused:
            return
        self._prog_timer.stop()
        self._prog_paused = True
        self._hold_current_position()
        self._prog_run_btn.setEnabled(True)
        self._prog_run_btn.setText('▶  Reanudar')
        self._prog_stop_btn.setEnabled(False)
        self._prog_cancel_btn.setEnabled(True)
        self._prog_status.setText('⏸  En pausa — pulsa Reanudar')
        self._prog_status.setStyleSheet(
            'font-size:9px; font-weight:bold; color:#ffcc44;')

    def _resume_program(self):
        """Reanuda tras una pausa. Si estaba a medio movimiento animado,
        recalcula el punto de partida desde la posición REAL del robot (no
        tiene por qué coincidir exactamente con el frame en que se congeló
        la animación local) para que la interpolación siga hacia el mismo
        destino sin saltos ni perder la meta."""
        if not self._prog_paused:
            return
        if self._prog_state == 'moving':
            if (not self._dry_run_mode) and self.serial_port and self.serial_port.is_open:
                self._prog_anim_q_start = np.array(self.real_angles_feedback, dtype=float)
                self.angles = list(self.real_angles_feedback)
            else:
                self._prog_anim_q_start = np.array(self.angles, dtype=float)
            # Re-resolver el destino "envuelto" (0/360) desde el nuevo punto
            # de partida real — el que se calculó al lanzar el movimiento
            # pudo dejar de ser el más corto/seguro tras el hueco de la
            # pausa (ver _resolve_anim_q_end).
            self._prog_anim_q_end = _resolve_anim_q_end(
                self._prog_anim_q_start, self._prog_anim_q_end)
            max_d = np.max(np.abs(self._prog_anim_q_end - self._prog_anim_q_start))
            self._prog_anim_total = max(int(np.ceil(max_d / 3.0)), 1)
            self._prog_anim_step = 0
            # Al pausar, _hold_current_position() mandó al robot un setpoint
            # igual a su posición actual (para que deje de perseguir el
            # destino del bloque) — hay que re-mandar el destino real para
            # que retome la marcha, y reiniciar desde cero el chequeo de
            # estabilidad en paralelo (ver _prog_settle_tick()).
            if (not self._dry_run_mode) and self.serial_port and self.serial_port.is_open:
                q_target_wrapped = [_wrap360(float(v)) for v in self._prog_anim_q_end]
                self._prog_settle_target = np.array(q_target_wrapped, dtype=float)
                self.send_setpoints(q_target_wrapped, vel_pct=self._prog_current_vel_pct)
            self._prog_settle_ticks  = 0
            self._prog_settle_stable = 0
            self._prog_settle_prev   = list(self.real_angles_feedback)
        elif self._prog_state == 'settling':
            # Reinicia el chequeo de estabilidad desde cero: comparar contra
            # datos de antes de la pausa daría una lectura de movimiento
            # falsa (el hueco de tiempo en pausa no es "el robot quieto").
            self._prog_settle_ticks  = 0
            self._prog_settle_stable = 0
            self._prog_settle_prev   = list(self.real_angles_feedback)
        self._prog_paused = False
        self._prog_run_btn.setEnabled(False)
        self._prog_run_btn.setText('▶  Run')
        self._prog_stop_btn.setEnabled(True)
        self._prog_cancel_btn.setEnabled(True)
        self._prog_status.setText('▶ Ejecutando...')
        self._prog_status.setStyleSheet(
            'font-size:9px; font-weight:bold; color:#70e570;')
        self._prog_timer.start(25)

    def _cancel_program(self):
        """Botón 'Cancelar': aborta la rutina por completo, esté corriendo o
        en pausa. El robot se manda a sostener su posición actual (si hay
        conexión real) y no se le vuelve a mandar ningún setpoint hasta que
        el usuario lo mueva a mano (mando o aplicación) — a diferencia de
        Stop, no queda generador ni estado que reanudar."""
        was_active = self._prog_running or self._prog_paused
        if was_active:
            self._hold_current_position()
        self._reset_program_state(
            '⛔  Cancelado' if was_active else 'Detenido', '#ff6b6b')

    def _reset_program_state(self, status_text, color):
        """Vuelve el motor de rutinas a reposo total (usado tanto al
        cancelar como al completarse normalmente)."""
        self._prog_timer.stop()
        self._prog_running = False
        self._prog_paused  = False
        self._prog_gen     = None
        self._prog_call_stack.clear()
        self._prog_set_highlight(None)
        self._prog_run_btn.setEnabled(True)
        self._prog_run_btn.setText('▶  Run')
        self._prog_stop_btn.setEnabled(False)
        self._prog_cancel_btn.setEnabled(False)
        self._prog_status.setText(status_text)
        self._prog_status.setStyleSheet(
            f'font-size:9px; font-weight:bold; color:{color};')

    def _hold_current_position(self):
        """Manda al robot real un setpoint igual a su posición ACTUAL (según
        el feedback de los encoders), para que el firmware deje de perseguir
        el objetivo del movimiento en curso y se quede quieto justo donde
        está. Sin conexión real (o en modo simulación) no hay nada físico
        que frenar, así que no hace falta enviar nada."""
        if self._dry_run_mode or not (self.serial_port and self.serial_port.is_open):
            return
        try:
            self.angles = list(self.real_angles_feedback)
            for i in range(6):
                self.sliders[i].blockSignals(True)
                self.sliders[i].setValue(int(self.angles[i]))
                self.sliders[i].blockSignals(False)
                self.inputs[i].setText(f'{self.angles[i]:.2f}')
                self.labels[i].setText(f'Articulación {i+1}: {self.angles[i]:.2f}°')
            self.update_xyz_display()
            self.update_3d_visualization()
            self.send_setpoints(self.angles)
        except Exception as ex:
            print(f'[Rutinas] Error al congelar posición: {ex}')

    def _prog_set_highlight(self, widget):
        """Resalta visualmente el bloque que se está ejecutando ahora mismo
        (o quita el resaltado del anterior). 'widget' puede ser None. Se
        conserva durante una pausa (no se llama con None al pausar) para que
        se vea sobre qué bloque se quedó congelada la rutina."""
        prev = self._prog_highlighted_widget
        if prev is not None and prev is not widget:
            try:
                prev.set_executing(False)
            except RuntimeError:
                pass  # el widget ya no existe (se editó la rutina)
        if widget is not None and widget is not prev:
            try:
                widget.set_executing(True)
            except RuntimeError:
                widget = None
        self._prog_highlighted_widget = widget

    def _prog_tick(self):
        """Llamado cada 25 ms — avanza el generador o anima el movimiento."""
        if self._prog_state == 'needs_next':
            try:
                event = next(self._prog_gen)
            except StopIteration:
                self._reset_program_state('✅  Completado', '#70e570')
                return
            kind = event[0]
            data = event[1] if len(event) > 1 else {}
            widget = data.get('_widget') if isinstance(data, dict) else None
            self._prog_set_highlight(widget)
            if kind in ('move', 'move_ori'):
                needs_anim = self._prog_start_move(data)
                if needs_anim:
                    self._prog_state = 'moving'
                # else: state already set inside _prog_start_move
            elif kind == 'vacuum':
                state = data.get('state', False) if isinstance(data, dict) else bool(data)
                cmd = f"bomba={'1' if state else '0'}\n"
                if (not self._dry_run_mode) and self.serial_port and self.serial_port.is_open:
                    try:
                        self.serial_port.write(cmd.encode('ascii'))
                    except Exception:
                        pass
                icon = '🟣' if state else '⚫'
                self._prog_status.setText(
                    f"{icon} Bomba {'ON' if state else 'OFF'}")
                # estado permanece needs_next — se avanza en el siguiente tick
            elif kind == 'cinta':
                state = data.get('state', False) if isinstance(data, dict) else bool(data)
                vel_pct = data.get('vel_pct', 50) if isinstance(data, dict) else 50
                self.send_cinta_cmd(state, vel_pct)
                icon = '📦'
                self._prog_status.setText(
                    f"{icon} Cinta {'ARRANQUE ' + str(vel_pct) + '%' if state else 'PARO'}")
                # estado permanece needs_next — se avanza en el siguiente tick
            elif kind == 'home':
                # Directo: manda las 6 articulaciones a 0° de inmediato,
                # sin animación simulada en el visor 3D (igual que
                # trayectoria 'Directo' en un bloque de movimiento).
                self._prog_status.setText('🏠 Home (0° directo)')
                thetas0 = np.deg2rad(np.zeros(6))
                T0 = T_MDH_TO_FUSION @ cinematica_directa(thetas0, self.rotation_signs_mdh)
                self._prog_target_xyz = (T0[0, 3] * 1000, T0[1, 3] * 1000, T0[2, 3] * 1000)
                self._prog_target_var = 'HOME'
                self.angles = [0.0] * 6
                for i in range(6):
                    self.sliders[i].blockSignals(True)
                    self.sliders[i].setValue(0)
                    self.sliders[i].blockSignals(False)
                    self.inputs[i].setText('0.00')
                    self.labels[i].setText(f'Articulación {i+1}: 0.00°')
                self.update_xyz_display()
                self.update_3d_visualization()
                self.send_setpoints(self.angles)
                self._prog_settle_target = np.array(self.angles, dtype=float)
                self._prog_enter_settling()

            elif kind == 'wait':
                seconds = data.get('seconds', 1) if isinstance(data, dict) else data
                ticks = max(0, int(seconds * 40))
                if ticks > 0:
                    self._prog_wait_ticks = ticks
                    self._prog_state = 'waiting'
                    if widget is not None:
                        widget.set_wait_countdown(ticks / 40.0)
                # else: 0-second wait, stay needs_next

        elif self._prog_state == 'moving':
            # El setpoint real YA se mandó al robot al arrancar este bloque
            # (ver _prog_start_move) — esta animación es solo la vista 3D
            # local y NO condiciona ni retrasa al robot físico. Corre a su
            # cadencia nominal fija (3°/tick), sin depender del vel_pct del
            # bloque, así que si el robot real va rápido puede llegar (y
            # confirmarse abajo, vía _prog_settle_tick) antes de que esta
            # animación termine — eso es intencional, no un error.
            if self._prog_anim_step < self._prog_anim_total:
                self._prog_anim_step += 1
                t = min(self._prog_anim_step / self._prog_anim_total, 1.0)
                angles = (self._prog_anim_q_start
                          + t * (self._prog_anim_q_end - self._prog_anim_q_start))
                for i in range(6):
                    self.angles[i] = float(angles[i])
                    self.sliders[i].blockSignals(True)
                    self.sliders[i].setValue(int(angles[i]))
                    self.sliders[i].blockSignals(False)
                    self.inputs[i].setText(f'{angles[i]:.2f}')
                    self.labels[i].setText(f'Articulación {i+1}: {angles[i]:.2f}°')
                self.update_xyz_display()
                self.update_3d_visualization()
                if getattr(self, '_traj_tracing', False):
                    self._traj_add_point()
                if self._prog_anim_step >= self._prog_anim_total:
                    # Normalizar a [0,360): la interpolacion pudo usar una
                    # representacion "envuelta" (p.ej. 360 grados en vez de 0)
                    # para esquivar una zona prohibida durante el trayecto; el
                    # angulo final es el mismo fisicamente, se muestra canonico.
                    # No se manda nada por serie aquí: el setpoint real ya
                    # salió al empezar el bloque.
                    for i in range(6):
                        self.angles[i] = _wrap360(self.angles[i])
                        self.sliders[i].blockSignals(True)
                        self.sliders[i].setValue(int(self.angles[i]))
                        self.sliders[i].blockSignals(False)
                        self.inputs[i].setText(f'{self.angles[i]:.2f}')
                        self.labels[i].setText(f'Articulación {i+1}: {self.angles[i]:.2f}°')
                    self.update_xyz_display()
                    self.update_3d_visualization()

            # Comprobación de llegada real, en paralelo a la animación de
            # arriba (no después de ella) — ver _prog_settle_tick().
            self._prog_settle_tick()

        elif self._prog_state == 'settling':
            # Bloques sin animación local (Directo / Home): la comprobación
            # de llegada es la misma que arriba, solo que no hay vista 3D
            # que sincronizar de por medio.
            self._prog_settle_tick()

        elif self._prog_state == 'waiting':
            if self._prog_wait_ticks > 0:
                self._prog_wait_ticks -= 1
                secs_left = self._prog_wait_ticks / 40.0
                self._prog_status.setText(f'⏸ Retardo… {secs_left:.1f}s')
                w = self._prog_highlighted_widget
                if w is not None:
                    try:
                        w.set_wait_countdown(secs_left)
                    except RuntimeError:
                        pass
            else:
                w = self._prog_highlighted_widget
                if w is not None:
                    try:
                        w.set_wait_countdown(None)
                    except RuntimeError:
                        pass
                self._prog_state = 'needs_next'

    def _prog_start_move(self, block_data):
        """
        Prepara el movimiento. Devuelve True si necesita animación,
        False si se manejó directamente (Directo o IK fallido).
        """
        x    = block_data.get('x', 0)
        y    = block_data.get('y', 0)
        z    = block_data.get('z', 0)
        traj = block_data.get('traj', 'Articular')

        # Recordar el destino de este movimiento (y el nombre de variable, si
        # se usó una) para poder comparar al final contra dónde se paró
        # realmente el robot — ver _prog_log_arrival(), llamado al terminar
        # el "settling".
        self._prog_target_xyz = (x, y, z)
        self._prog_target_var = block_data.get('var_name')
        self._prog_current_vel_pct = block_data.get('vel_pct', 100)

        target_rpy = None
        if block_data.get('type') == 'move_ori' or 'roll' in block_data:
            target_rpy = [block_data.get('roll', 0),
                           block_data.get('pitch', 0),
                           block_data.get('yaw', 0)]

        # ── Sincronizar desde el robot real ──────────────────────────────
        # Si hay feedback serial, usar los ángulos reales como punto de
        # partida para la IK (no la posición del modelo 3D animado).
        if (not self._dry_run_mode) and self.serial_port and self.serial_port.is_open:
            for i in range(6):
                self.angles[i] = self.real_angles_feedback[i]
            for i in range(6):
                self.sliders[i].blockSignals(True)
                self.sliders[i].setValue(int(self.angles[i]))
                self.sliders[i].blockSignals(False)
                self.inputs[i].setText(f'{self.angles[i]:.2f}')
                self.labels[i].setText(f'Articulación {i+1}: {self.angles[i]:.2f}°')
            self.update_xyz_display()
            self.update_3d_visualization()

        # Movimiento por variable en modo "ángulos (directo)": el destino ya
        # son los 6 ángulos guardados con la variable — se salta la IK por
        # completo (nada que resolver: son exactamente los ángulos de la pose
        # capturada). x/y/z siguen disponibles arriba (vienen de la propia
        # variable) solo para el log de llegada y, si la trayectoria es
        # 'Lineal', para estimar la distancia cartesiana.
        if 'q_deg' in block_data:
            q_deg = np.array(block_data['q_deg'], dtype=float)
        else:
            q_deg, _, _, ok = ik_dls([x, y, z], self.angles, self.rotation_signs_mdh,
                                      target_rpy_deg=target_rpy)
            if not ok:
                detalle = _describe_ik_failure(q_deg)
                msg_corto = f'IK sin solución para ({x:.0f},{y:.0f},{z:.0f})'
                self._prog_status.setText(f'❌ {msg_corto}')
                self._log_error(f'{msg_corto} — {detalle}', 'WARN')
                self._prog_state = 'needs_next'
                return False

        self._prog_anim_q_start = np.array(self.angles, dtype=float)
        self._prog_anim_q_end   = np.array(q_deg, dtype=float)

        # Evitar que la interpolacion articular cruce una zona prohibida:
        # si existe un camino equivalente (mismo angulo final, dando la
        # vuelta por 0/360 grados) que la esquiva, usarlo en vez del directo.
        self._prog_anim_q_end = _resolve_anim_q_end(
            self._prog_anim_q_start, self._prog_anim_q_end)

        if traj == 'Directo':
            for i in range(6):
                self.angles[i] = q_deg[i]
                self.sliders[i].blockSignals(True)
                self.sliders[i].setValue(int(q_deg[i]))
                self.sliders[i].blockSignals(False)
                self.inputs[i].setText(f'{q_deg[i]:.2f}')
                self.labels[i].setText(f'Articulación {i+1}: {q_deg[i]:.2f}°')
            self.update_xyz_display()
            self.update_3d_visualization()
            self.send_setpoints(self.angles, vel_pct=self._prog_current_vel_pct)
            self._prog_settle_target = np.array(self.angles, dtype=float)
            self._prog_enter_settling()
            return False

        # ── Trayectoria animada (Articular / Lineal) ─────────────────────
        # Mandar el setpoint real YA, en el mismo instante en que arranca
        # la animación local del visor 3D, en vez de esperar a que la
        # animación termine (como antes). Así el robot físico nunca espera
        # a la simulación: si va rápido puede llegar (y confirmarse, ver
        # _prog_settle_tick) antes de que la animación acabe de dibujarse.
        q_target_wrapped = [_wrap360(float(v)) for v in self._prog_anim_q_end]
        self._prog_settle_target = np.array(q_target_wrapped, dtype=float)
        self.send_setpoints(q_target_wrapped, vel_pct=self._prog_current_vel_pct)
        self._pos_history_add(angles_override=q_target_wrapped)
        self._prog_settle_ticks  = 0
        self._prog_settle_stable = 0
        self._prog_settle_prev   = list(self.real_angles_feedback)

        if traj == 'Lineal':
            T_curr = T_MDH_TO_FUSION @ cinematica_directa(
                np.deg2rad(self.angles), self.rotation_signs_mdh)
            p0 = T_curr[:3, 3] * 1000
            dist = np.linalg.norm(np.array([x, y, z]) - p0)
            self._prog_anim_total = max(int(dist / 3.0), 1)
        else:  # Articular
            max_d = np.max(np.abs(self._prog_anim_q_end - self._prog_anim_q_start))
            self._prog_anim_total = max(int(np.ceil(max_d / 3.0)), 1)

        self._prog_anim_step = 0
        return True

    def _prog_enter_settling(self):
        """Inicia el estado de espera hasta que las articulaciones se estabilicen.
        Usado por las trayectorias sin animación (Directo / Home), donde
        self._prog_settle_target ya se dejó puesto justo antes de llamar
        aquí. Los bloques animados (Articular/Lineal) no pasan por este
        método — inician su propio chequeo en paralelo, ver _prog_start_move."""
        self._pos_history_add()   # registrar posición alcanzada
        self._prog_settle_ticks  = 0
        self._prog_settle_stable = 0
        self._prog_settle_prev   = list(self.real_angles_feedback)
        self._prog_state = 'settling'

    def _prog_settle_tick(self):
        """Un tick (25 ms) del chequeo de 'llegada estable' del robot real,
        comparando contra self._prog_settle_target. Compartido entre el
        estado 'settling' (Directo/Home) y el estado 'moving' (Articular/
        Lineal), donde ahora corre en paralelo a la animación local del
        visor 3D — ver _prog_tick()."""
        self._prog_settle_ticks += 1

        # Timeout de seguridad (editable, self._prog_settle_timeout_s,
        # 30 s por defecto) → continuar sin importar, pero avisando en el
        # log de que no se confirmó la llegada (mejor que avanzar en
        # silencio y que parezca que todo fue bien).
        if self._prog_settle_ticks > self._prog_settle_timeout_s * 40:
            self._prog_finish_move(timeout=True)
            return

        # Sin serial (o modo simulación) → esperar mínimo 800 ms (32 ticks × 25 ms).
        # No hay feedback real con el que comparar, así que no se puede
        # comprobar cercanía ni registrar el log de llegada.
        if self._dry_run_mode or not (self.serial_port and self.serial_port.is_open):
            if self._prog_settle_ticks >= 32:
                self._prog_finish_move()
            return

        # Periodo mínimo de arranque: 300 ms (12 ticks).
        # Evita contar como "estable" el instante antes de que el motor arranque.
        if self._prog_settle_ticks < 12:
            self._prog_settle_prev = list(self.real_angles_feedback)
            return

        # Comparar con snapshot anterior (con wraparound 0/360, igual que
        # robotEnMovimiento() en el firmware del central)
        curr = list(self.real_angles_feedback)
        max_delta = max(abs(_angular_diff(curr[i], self._prog_settle_prev[i]))
                         for i in range(6))
        self._prog_settle_prev = curr

        # Distancia angular al OBJETIVO REAL del bloque (self._prog_settle_target),
        # NO a self.angles: mientras el chequeo corre en paralelo a la
        # animación local (estado 'moving'), self.angles todavía puede estar
        # a medio camino aunque el robot real ya haya llegado. Además, en el
        # tramo final de la trayectoria el firmware usa velocidad reducida
        # (approach_vel/min_vel), y a esa velocidad el cambio real entre dos
        # muestras de 25 ms puede caer por debajo del umbral de "estable"
        # (<0.5°) MIENTRAS EL BRAZO TODAVÍA SE ESTÁ ACERCANDO — exigir también
        # estar dentro de _PROG_MOVEMENT_TOL_DEG del objetivo evita ese falso
        # "ya llegó".
        max_err_target = max(
            abs(_angular_diff(self._prog_settle_target[i], curr[i])) for i in range(6))

        if max_delta < 0.5 and max_err_target <= self._PROG_MOVEMENT_TOL_DEG:
            self._prog_settle_stable += 1
        else:
            self._prog_settle_stable = 0

        # Ventana de estabilidad editable (self._prog_stable_spin, 0.5 s por
        # defecto): tiempo seguido sin movimiento y dentro de tolerancia
        # antes de dar el bloque por llegado.
        if self._prog_settle_stable >= self._prog_stable_window_s * 40:
            self._prog_finish_move()

    def _prog_finish_move(self, timeout=False):
        """Confirma que el robot real llegó (o fuerza el avance por
        timeout) y pasa al siguiente bloque. Si la animación local del
        visor 3D todavía no había terminado — el robot puede llegar antes
        que ella si va a alta velocidad — la deja saltada directamente al
        destino para que la vista no se quede a medio camino."""
        if (self._prog_state == 'moving' and self._prog_anim_q_end is not None
                and self._prog_anim_step < self._prog_anim_total):
            for i in range(6):
                self.angles[i] = _wrap360(float(self._prog_anim_q_end[i]))
                self.sliders[i].blockSignals(True)
                self.sliders[i].setValue(int(self.angles[i]))
                self.sliders[i].blockSignals(False)
                self.inputs[i].setText(f'{self.angles[i]:.2f}')
                self.labels[i].setText(f'Articulación {i+1}: {self.angles[i]:.2f}°')
            self.update_xyz_display()
            self.update_3d_visualization()
        self._prog_state = 'needs_next'
        self._prog_log_arrival(timeout=timeout)

    def _prog_log_arrival(self, timeout=False):
        """Registra en el log la coordenada XYZ indicada al bloque de
        movimiento (con el nombre de variable, si se usó una) frente a la
        coordenada XYZ real a la que se ha detenido el robot, calculada por
        cinemática directa a partir del feedback real de los encoders."""
        target = self._prog_target_xyz
        if target is None:
            return
        self._prog_target_xyz = None  # evita volver a loguear el mismo destino

        tx, ty, tz = target
        var_name = self._prog_target_var
        var_txt = f' ({var_name})' if var_name else ''

        thetas = np.deg2rad(np.array(self.real_angles_feedback, dtype=float))
        T = T_MDH_TO_FUSION @ cinematica_directa(thetas, rotation_signs_mdh=self.rotation_signs_mdh)
        rx, ry, rz = T[0, 3] * 1000, T[1, 3] * 1000, T[2, 3] * 1000
        dx, dy, dz = rx - tx, ry - ty, rz - tz
        err_mm = (dx**2 + dy**2 + dz**2) ** 0.5

        aviso = ' — TIMEOUT (30s), puede no haber llegado' if timeout else ''
        nivel = 'WARN' if (timeout or err_mm > 5.0) else 'INFO'
        self._log_error(
            f'Rutina{aviso} — coordenadas indicadas: {tx:.1f}, {ty:.1f}, {tz:.1f}{var_txt} ; '
            f'coordenadas en la que se ha parado: {rx:.1f}, {ry:.1f}, {rz:.1f}  '
            f'(error dX={dx:+.1f} dY={dy:+.1f} dZ={dz:+.1f}, {err_mm:.1f} mm)',
            nivel)

    def _build_prog_gen(self, blocks):
        """Generador recursivo que produce eventos de ejecución."""
        for block in blocks:
            if not self._prog_running:
                return
            t = block['type']

            if t == 'move':
                if 'var' in block:
                    vd = self._position_vars.get(block['var'])
                    if vd:
                        evento = {
                            'type': 'move',
                            'x': vd.get('x', 0),
                            'y': vd.get('y', 0),
                            'z': vd.get('z', 0),
                            'traj': block.get('traj', vd.get('traj', 'Articular')),
                            'var_name': block['var'],   # para el log de llegada
                            'vel_pct': block.get('vel_pct', 100),
                            '_widget': block.get('_widget'),
                        }
                        if block.get('var_mode') == 'angles':
                            if vd.get('angles'):
                                evento['q_deg'] = list(vd['angles'])  # sin IK
                            else:
                                self._prog_status.setText(
                                    f'⚠ "{block["var"]}" sin ángulos guardados, se usa IK')
                        yield ('move', evento)
                    else:
                        self._prog_status.setText(
                            f'⚠ Variable "{block["var"]}" no encontrada')
                else:
                    yield ('move', block)

            elif t == 'move_ori':
                if 'var' in block:
                    vd = self._position_vars.get(block['var'])
                    if vd:
                        evento = {
                            'type': 'move_ori',
                            'x': vd.get('x', 0),
                            'y': vd.get('y', 0),
                            'z': vd.get('z', 0),
                            'roll':  block.get('roll', 0),
                            'pitch': block.get('pitch', 0),
                            'yaw':   block.get('yaw', 0),
                            'traj': block.get('traj', vd.get('traj', 'Articular')),
                            'var_name': block['var'],   # para el log de llegada
                            'vel_pct': block.get('vel_pct', 100),
                            '_widget': block.get('_widget'),
                        }
                        if block.get('var_mode') == 'angles':
                            if vd.get('angles'):
                                evento['q_deg'] = list(vd['angles'])  # sin IK, ignora R/P/Yw
                            else:
                                self._prog_status.setText(
                                    f'⚠ "{block["var"]}" sin ángulos guardados, se usa IK')
                        yield ('move_ori', evento)
                    else:
                        self._prog_status.setText(
                            f'⚠ Variable "{block["var"]}" no encontrada')
                else:
                    yield ('move_ori', block)

            elif t == 'home':
                yield ('home', block)

            elif t == 'vacuum':
                yield ('vacuum', block)

            elif t == 'cinta':
                yield ('cinta', block)

            elif t == 'subroutine':
                sub_name = block.get('name', '')
                if sub_name:
                    if sub_name in self._prog_call_stack:
                        self._prog_status.setText(f'⚠ Subrutina recursiva "{sub_name}" ignorada')
                    else:
                        sub_path = os.path.join(RUTINAS_DIR, sub_name + '.json')
                        try:
                            with open(sub_path, 'r', encoding='utf-8') as _f:
                                sub_data = json.load(_f)
                            self._prog_call_stack.add(sub_name)
                            if isinstance(sub_data, dict) and 'blocks' in sub_data:
                                _sv = dict(self._position_vars)
                                self._position_vars.update(sub_data.get('variables', {}))
                                yield from self._build_prog_gen(sub_data['blocks'])
                                self._position_vars = _sv
                            else:
                                yield from self._build_prog_gen(sub_data)
                            self._prog_call_stack.discard(sub_name)
                        except Exception as _ex:
                            self._prog_status.setText(f'❌ Subrutina "{sub_name}": {_ex}')
                            self._log_error(f'Subrutina "{sub_name}": {_ex}', 'ERROR')
            elif t == 'wait':
                yield ('wait', block)

            elif t == 'for':
                for _ in range(block.get('n', 1)):
                    if not self._prog_running:
                        return
                    yield from self._build_prog_gen(block.get('body', []))

            elif t == 'while_true':
                while self._prog_running:
                    body = block.get('body', [])
                    if not body:
                        return   # evitar bucle infinito vacío
                    yield from self._build_prog_gen(body)

            elif t == 'while_cond':
                while (self._prog_running
                       and self._eval_prog_cond(block.get('cond_joint', 'J1'),
                                                block.get('cond_op', '>'),
                                                block.get('cond_val', 0))):
                    yield from self._build_prog_gen(block.get('body', []))

            elif t == 'if':
                if self._eval_prog_cond(block.get('cond_joint', 'J1'),
                                        block.get('cond_op', '>'),
                                        block.get('cond_val', 0)):
                    yield from self._build_prog_gen(block.get('body', []))

            elif t == 'if_else':
                if self._eval_prog_cond(block.get('cond_joint', 'J1'),
                                        block.get('cond_op', '>'),
                                        block.get('cond_val', 0)):
                    yield from self._build_prog_gen(block.get('body', []))
                else:
                    yield from self._build_prog_gen(block.get('else_body', []))

    def _eval_prog_cond(self, joint, op, val):
        """Evalúa una condición de articulación, posición cartesiana, o el
        sensor de la cinta transportadora ('Pieza (cinta)', ver
        cinta_objeto_detectado / _parse_cinta_estado)."""
        try:
            if joint == 'Pieza (cinta)':
                current = 1.0 if self.cinta_objeto_detectado else 0.0
            elif joint.startswith('J'):
                current = self.angles[int(joint[1]) - 1]
            else:
                thetas = np.deg2rad(np.array(self.angles, dtype=float))
                T = T_MDH_TO_FUSION @ cinematica_directa(
                    thetas, self.rotation_signs_mdh)
                axis = {'X(mm)': 0, 'Y(mm)': 1, 'Z(mm)': 2}.get(joint, 0)
                current = T[axis, 3] * 1000
            if op == '>':  return current > val
            if op == '<':  return current < val
            if op == '>=': return current >= val
            if op == '<=': return current <= val
            if op == '==': return abs(current - val) < 0.5
            if op == '!=': return abs(current - val) >= 0.5
        except Exception:
            pass
        return False
