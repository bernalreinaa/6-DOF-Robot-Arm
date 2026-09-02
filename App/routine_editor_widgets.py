# -*- coding: utf-8 -*-
"""
Widgets del editor visual de rutinas por bloques: el diálogo de solo
lectura con el diagrama de flujo (FlowchartDialog) y las piezas del editor
arrastrable (BlockContainer/BlockWidget/_DragHandle), con su función de
restauración desde JSON (_block_restore).
"""
import json
import os
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

from app_paths import RUTINAS_DIR
from kinematics import T_MDH_TO_FUSION, cinematica_directa

class FlowchartDialog(QDialog):
    """Muestra la rutina actual como diagrama de flujo interactivo (solo lectura)."""

    _NW, _NH = 155, 44
    _DW, _DH = 140, 58
    _OW, _OH = 100, 32
    _SPY     = 26
    _IND     = 52
    _LBK     = 28

    _COL = {
        'move':       ('#1a3d1a', '#4CAF50'),
        'wait':       ('#1c1c2c', '#9E9E9E'),
        'for':        ('#0c1e40', '#2196F3'),
        'while_true': ('#0c1e40', '#2196F3'),
        'while_cond': ('#0c1640', '#3F51B5'),
        'if':         ('#382600', '#FF9800'),
        'if_else':    ('#381500', '#FF5722'),
        'vacuum':     ('#1a0a2e', '#e040fb'),
        'move_ori':   ('#0d2a3d', '#26C6DA'),
        'cinta':      ('#0d251f', '#00b894'),
    }

    def __init__(self, blocks, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Diagrama de Flujo — Rutina")
        self.resize(630, 444)
        self.setStyleSheet(
            "QDialog{background:#0d1014;}"
            "QPushButton{color:#70e570;border:1px solid #70e570;"
            "background:transparent;border-radius:3px;padding:2px 10px;}"
            "QPushButton:hover{background:#70e570;color:#0d1014;}")
        vl = QVBoxLayout(self)
        vl.setContentsMargins(2, 2, 2, 2)
        tb = QHBoxLayout()
        for txt, cb in [("Ajustar",    self._fit),
                        ("+ Zoom",     lambda: self._gv.scale(1.2, 1.2)),
                        ("- Zoom",     lambda: self._gv.scale(1/1.2, 1/1.2))]:
            b = QPushButton(txt)
            b.setFixedHeight(14)
            b.clicked.connect(cb)
            tb.addWidget(b)
        tb.addStretch()
        hint = QLabel("  Rueda del raton para zoom  ·  Arrastrar para mover")
        hint.setStyleSheet("color:#555; font-size:8px;")
        tb.addWidget(hint)
        vl.addLayout(tb)
        self._sc = QGraphicsScene()
        self._gv = QGraphicsView(self._sc)
        self._gv.setRenderHints(QPainter.Antialiasing | QPainter.TextAntialiasing)
        self._gv.setStyleSheet("QGraphicsView{background:#0d1014;border:none;}")
        self._gv.setDragMode(QGraphicsView.ScrollHandDrag)
        self._gv.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        def _wheel(ev):
            f = 1.15 if ev.angleDelta().y() > 0 else 1/1.15
            self._gv.scale(f, f)
        self._gv.wheelEvent = _wheel
        vl.addWidget(self._gv, 1)
        self._render(blocks)
        QTimer.singleShot(80, self._fit)

    def _fit(self):
        self._gv.fitInView(
            self._sc.itemsBoundingRect().adjusted(-28, -28, 28, 28),
            Qt.KeepAspectRatio)

    def _pen(self, c, w=1.5, d=Qt.SolidLine):
        return QPen(QColor(c), w, d)

    def _brush(self, c):
        return QBrush(QColor(c))

    def _add_lbl(self, text, rx, ry, rw, rh, col='#c8c8c8', sz=8):
        ti = QGraphicsTextItem(text)
        ti.setDefaultTextColor(QColor(col))
        f = QFont("Segoe UI", sz)
        ti.setFont(f)
        ti.setTextWidth(rw - 8)
        bh = ti.boundingRect().height()
        ti.setPos(rx + 4, ry + max(0, (rh - bh) / 2))
        self._sc.addItem(ti)

    def _arrow(self, pts, col='#6a8a9a', lbl='', dashed=False):
        import math
        path = QPainterPath()
        path.moveTo(*pts[0])
        for p in pts[1:]:
            path.lineTo(*p)
        pi = QGraphicsPathItem(path)
        pi.setPen(self._pen(col, 1.4, Qt.DashLine if dashed else Qt.SolidLine))
        self._sc.addItem(pi)
        x1, y1 = pts[-2]; x2, y2 = pts[-1]
        a = math.atan2(y2 - y1, x2 - x1); al, aa = 8, 0.38
        tip = QPointF(x2, y2)
        lp  = QPointF(x2 - al * math.cos(a - aa), y2 - al * math.sin(a - aa))
        rp  = QPointF(x2 - al * math.cos(a + aa), y2 - al * math.sin(a + aa))
        arr = QGraphicsPolygonItem(QPolygonF([tip, lp, rp]))
        arr.setBrush(self._brush(col))
        arr.setPen(self._pen(col, 0.5))
        self._sc.addItem(arr)
        if lbl:
            t = QGraphicsTextItem(lbl)
            t.setDefaultTextColor(QColor('#ddcc44'))
            tf = QFont("Segoe UI", 7); tf.setBold(True)
            t.setFont(tf)
            t.setPos(pts[0][0] + 4, pts[0][1] + 2)
            self._sc.addItem(t)

    def _nrect(self, lbl, x, y, bg, br):
        ri = QGraphicsRectItem(x, y, self._NW, self._NH)
        ri.setBrush(self._brush(bg)); ri.setPen(self._pen(br))
        self._sc.addItem(ri)
        self._add_lbl(lbl, x, y, self._NW, self._NH)
        hw = self._NW / 2
        return (x + hw, y), (x + hw, y + self._NH), (x, y + self._NH / 2), (x + self._NW, y + self._NH / 2)

    def _noval(self, lbl, x, y, bg, br, tc='#d0ffd0'):
        ei = QGraphicsEllipseItem(x, y, self._OW, self._OH)
        ei.setBrush(self._brush(bg)); ei.setPen(self._pen(br, 2))
        self._sc.addItem(ei)
        self._add_lbl(lbl, x, y, self._OW, self._OH, col=tc, sz=9)
        hw = self._OW / 2
        return (x + hw, y), (x + hw, y + self._OH), (x, y + self._OH / 2), (x + self._OW, y + self._OH / 2)

    def _ndiamond(self, lbl, x, y, bg, br):
        w, h = self._DW, self._DH; cx, cy = x + w / 2, y + h / 2
        poly = QPolygonF([QPointF(cx, y), QPointF(x + w, cy),
                          QPointF(cx, y + h), QPointF(x, cy)])
        pi = QGraphicsPolygonItem(poly)
        pi.setBrush(self._brush(bg)); pi.setPen(self._pen(br))
        self._sc.addItem(pi)
        self._add_lbl(lbl, x + 18, y + 8, w - 36, h - 16)
        return (cx, y), (cx, y + h), (x, cy), (x + w, cy)

    def _render(self, blocks):
        self._sc.clear()
        cx = 300; x0 = cx - self._NW // 2; ox = cx - self._OW // 2
        _, bot, *_ = self._noval("INICIO", ox, 0, '#152515', '#70e570')
        y = self._OH + self._SPY
        y, bot = self._lay(blocks, x0, cx, y, bot)
        _, top, *_ = self._noval("FIN", cx - self._OW // 2, y, '#251515', '#ff6b6b', '#ffcccc')
        self._arrow([bot, top])

    def _lay(self, blocks, x0, cx, y, conn):
        NW, NH, DW, DH, SPY = self._NW, self._NH, self._DW, self._DH, self._SPY
        for b in blocks:
            t = b.get('type', '')
            bg, br = self._COL.get(t, ('#202028', '#888888'))
            if t == 'move':
                vel_txt = f" ⚡{b.get('vel_pct',100):.0f}%" if b.get('vel_pct', 100) != 100 else ''
                if 'var' in b:
                    modo = ' 🦾áng.' if b.get('var_mode') == 'angles' else ''
                    lbl = (f"Mover a\n🔖 {b['var']}{modo}\n"
                           f"[{b.get('traj','Articular')}]{vel_txt}")
                else:
                    lbl = (f"Mover a\n"
                           f"X:{b.get('x',0):.0f}  Y:{b.get('y',0):.0f}  Z:{b.get('z',0):.0f}\n"
                           f"[{b.get('traj','Articular')}]{vel_txt}")
                top, bot, *_ = self._nrect(lbl, x0, y, bg, br)
                self._arrow([conn, top]); conn = bot; y += NH + SPY
            elif t == 'move_ori':
                vel_txt = f" ⚡{b.get('vel_pct',100):.0f}%" if b.get('vel_pct', 100) != 100 else ''
                if 'var' in b:
                    if b.get('var_mode') == 'angles':
                        lbl = (f"Mover a + Orient.\n🔖 {b['var']} 🦾áng.\n"
                               f"[{b.get('traj','Articular')}]{vel_txt}")
                    else:
                        lbl = (f"Mover a + Orient.\n🔖 {b['var']}\n"
                               f"R:{b.get('roll',0):.0f} P:{b.get('pitch',0):.0f} Y:{b.get('yaw',0):.0f}\n"
                               f"[{b.get('traj','Articular')}]{vel_txt}")
                else:
                    lbl = (f"Mover a + Orient.\n"
                           f"X:{b.get('x',0):.0f} Y:{b.get('y',0):.0f} Z:{b.get('z',0):.0f}\n"
                           f"R:{b.get('roll',0):.0f} P:{b.get('pitch',0):.0f} Y:{b.get('yaw',0):.0f}\n"
                           f"[{b.get('traj','Articular')}]{vel_txt}")
                top, bot, *_ = self._nrect(lbl, x0, y, bg, br)
                self._arrow([conn, top]); conn = bot; y += NH + SPY
            elif t == 'vacuum':
                state = b.get('state', False)
                lbl = f"Bomba vacío\n{'● ON' if state else '○ OFF'}"
                top, bot, *_ = self._nrect(lbl, x0, y, bg, br)
                self._arrow([conn, top]); conn = bot; y += NH + SPY
            elif t == 'cinta':
                state = b.get('state', False)
                vel_txt = f"\n⚡{b.get('vel_pct',50):.0f}%" if state else ''
                lbl = f"Cinta\n{'▶ ARRANQUE' if state else '■ PARO'}{vel_txt}"
                top, bot, *_ = self._nrect(lbl, x0, y, bg, br)
                self._arrow([conn, top]); conn = bot; y += NH + SPY
            elif t == 'wait':
                top, bot, *_ = self._nrect(f"Pausa\n{b.get('seconds', 1)} s", x0, y, bg, br)
                self._arrow([conn, top]); conn = bot; y += NH + SPY
            elif t in ('for', 'while_true'):
                lbl = (f"Repetir {b.get('n', 3)} veces" if t == 'for' else "Mientras\nsiempre")
                top, bot, _, rgt = self._nrect(lbl, x0, y, bg, br)
                self._arrow([conn, top]); hdr_top = top; y += NH + SPY
                body = b.get('body', []); bx0 = x0 + self._IND; bcx = cx + self._IND
                y, bb = self._lay(body, bx0, bcx, y, bot) if body else (y, bot)
                bkx = x0 + NW + self._LBK
                self._arrow([bb, (bkx, bb[1]), (bkx, hdr_top[1]), hdr_top],
                            col='#4488cc', dashed=True)
                y += SPY; conn = (cx, y - SPY // 2)
            elif t == 'while_cond':
                j = b.get('cond_joint', 'J1'); op = b.get('cond_op', '>'); v = b.get('cond_val', 0)
                cond_lbl = ('📦 Pieza detectada' if v >= 0.5 else '📦 Sin pieza') if j == 'Pieza (cinta)' else f"{j} {op} {v}"
                dx = x0 + (NW - DW) // 2
                top, bot, _, rgt = self._ndiamond(cond_lbl, dx, y, bg, br)
                self._arrow([conn, top]); hdr_top = top; y += DH + SPY
                body = b.get('body', [])
                if body:
                    y, bb = self._lay(body, x0, cx, y, bot)
                    bkx = x0 + NW + self._LBK
                    self._arrow([bb, (bkx, bb[1]), (bkx, hdr_top[1]), hdr_top],
                                col='#4488cc', dashed=True, lbl='Si')
                    y += SPY
                ex_y = y
                self._arrow([rgt, (rgt[0] + 55, rgt[1]), (rgt[0] + 55, ex_y), (cx, ex_y)],
                            col='#ff9944', lbl='No')
                conn = (cx, ex_y); y = ex_y + SPY
            elif t in ('if', 'if_else'):
                j = b.get('cond_joint', 'J1'); op = b.get('cond_op', '>'); v = b.get('cond_val', 0)
                cond_lbl = ('📦 Pieza detectada' if v >= 0.5 else '📦 Sin pieza') if j == 'Pieza (cinta)' else f"{j} {op} {v}"
                dx = x0 + (NW - DW) // 2
                top, bot, _, rgt = self._ndiamond(cond_lbl, dx, y, bg, br)
                self._arrow([conn, top]); cond_bot = bot; cond_rgt = rgt; y += DH + SPY
                yes_body = b.get('body', [])
                if yes_body:
                    y, yes_bot = self._lay(yes_body, x0, cx, y, cond_bot); y += SPY
                else:
                    yes_bot = cond_bot
                if t == 'if_else':
                    no_x0 = x0 + NW + 80; no_cx = no_x0 + NW // 2; no_ys = cond_bot[1] + SPY // 2
                    self._arrow([cond_rgt, (no_x0 + NW // 2, cond_rgt[1]), (no_x0 + NW // 2, no_ys)],
                                col='#ff9944', lbl='No')
                    no_conn = (no_x0 + NW // 2, no_ys)
                    else_body = b.get('else_body', [])
                    _, no_bot = (self._lay(else_body, no_x0 - NW // 2, no_cx, no_ys, no_conn)
                                 if else_body else (None, no_conn))
                    merge_y = max(y, no_bot[1] + SPY)
                    self._arrow([yes_bot, (cx, yes_bot[1]), (cx, merge_y)], lbl='Si')
                    self._arrow([no_bot,  (cx, no_bot[1]),  (cx, merge_y)])
                    conn = (cx, merge_y); y = merge_y + SPY
                else:
                    merge_y = max(y, yes_bot[1] + SPY // 2)
                    self._arrow([cond_rgt, (cond_rgt[0]+55, cond_rgt[1]),
                                  (cond_rgt[0]+55, merge_y), (cx, merge_y)],
                                col='#ff9944', lbl='No')
                    self._arrow([yes_bot, (cx, yes_bot[1]), (cx, merge_y)], lbl='Si')
                    conn = (cx, merge_y); y = merge_y + SPY
        return y, conn

# ─────────────────────────────────────────────────────────────────────────────
#  Editor visual de programación por bloques (Rutinas)
# ─────────────────────────────────────────────────────────────────────────────

_BLOCK_DEFS = {
    'move'       : ('🟢', 'Mover a',        '#1a3d1a', '#4CAF50'),
    'for'        : ('🔵', 'Repetir',         '#0d2040', '#2196F3'),
    'while_true' : ('🔵', 'Mientras True',   '#0d2040', '#2196F3'),
    'while_cond' : ('🔷', 'Mientras que',    '#0d1840', '#3F51B5'),
    'if'         : ('🟡', 'Si',              '#3a2800', '#FF9800'),
    'if_else'    : ('🟠', 'Si / Si no',      '#3a1800', '#FF5722'),
    'wait'       : ('⏸', 'Pausa',             '#1e1e1e', '#9E9E9E'),
    'vacuum'     : ('💜', 'Bomba vacío',       '#1a0a2e', '#e040fb'),
    'subroutine' : ('📎', 'Subrutina',         '#1a1a0a', '#CDDC39'),
    'home'       : ('🏠', 'Ir a Home',         '#0d1a0d', '#8BC34A'),
    'move_ori'   : ('🧭', 'Mover a + Orient.',  '#0d2a3d', '#26C6DA'),
    'cinta'      : ('📦', 'Cinta transportadora', '#0d251f', '#00b894'),
}


def _block_restore(data, container, editor):
    """Crea recursivamente un BlockWidget desde un dict de datos."""
    block = container.add_block(data['type'])
    try:
        t = data['type']
        if t == 'move':
            if 'var' in data:
                block._use_var = True
                block._var_toggle.setChecked(True)
                block._var_toggle.setText('🔖 Var')
                block._coord_frame.setVisible(False)
                block._var_combo.setVisible(True)
                block._refresh_var_combo()
                vi = block._var_combo.findText(data['var'])
                if vi >= 0:
                    block._var_combo.setCurrentIndex(vi)
                if hasattr(block, '_var_source_combo'):
                    block._var_source_combo.setVisible(True)
                    si = block._var_source_combo.findData(data.get('var_mode', 'xyz'))
                    if si >= 0:
                        block._var_source_combo.setCurrentIndex(si)
            else:
                block._mx.setText(str(data.get('x', 0)))
                block._my.setText(str(data.get('y', 0)))
                block._mz.setText(str(data.get('z', 200)))
            idx = block._mtraj.findText(data.get('traj', 'Articular'))
            if idx >= 0:
                block._mtraj.setCurrentIndex(idx)
            if hasattr(block, '_vel_spin'):
                block._vel_spin.setValue(int(data.get('vel_pct', 100)))
        elif t == 'move_ori':
            if 'var' in data:
                block._use_var = True
                block._var_toggle.setChecked(True)
                block._var_toggle.setText('🔖 Var')
                block._coord_frame.setVisible(False)
                block._var_combo.setVisible(True)
                block._refresh_var_combo()
                vi = block._var_combo.findText(data['var'])
                if vi >= 0:
                    block._var_combo.setCurrentIndex(vi)
                if hasattr(block, '_var_source_combo'):
                    block._var_source_combo.setVisible(True)
                    si = block._var_source_combo.findData(data.get('var_mode', 'xyz'))
                    if si >= 0:
                        block._var_source_combo.setCurrentIndex(si)
            else:
                block._mx.setText(str(data.get('x', 0)))
                block._my.setText(str(data.get('y', 0)))
                block._mz.setText(str(data.get('z', 200)))
            block._mroll.setText(str(data.get('roll', 0)))
            block._mpitch.setText(str(data.get('pitch', 0)))
            block._myaw.setText(str(data.get('yaw', 0)))
            idx = block._mtraj.findText(data.get('traj', 'Articular'))
            if idx >= 0:
                block._mtraj.setCurrentIndex(idx)
            if hasattr(block, '_vel_spin'):
                block._vel_spin.setValue(int(data.get('vel_pct', 100)))
            block._update_ori_enabled()
        elif t == 'vacuum':
            checked = data.get('state', False)
            block._vbtn.setChecked(checked)
            block._vbtn.setText('● Encendida' if checked else '○ Apagada')
        elif t == 'cinta':
            checked = data.get('state', False)
            block._cbtn.setChecked(checked)
            block._cbtn.setText('▶ Arranque' if checked else '■ Parada')
            block._cvel_spin.setValue(int(data.get('vel_pct', 50)))
        elif t == 'subroutine':
            block._refresh_sub_combo()
            si = block._sub_combo.findText(data.get('name', ''))
            if si >= 0:
                block._sub_combo.setCurrentIndex(si)
        elif t == 'wait':
            block._wsecs.setText(str(data.get('seconds', 1.0)))
        elif t == 'for':
            block._fn.setText(str(data.get('n', 3)))
        elif t in ('while_cond', 'if', 'if_else'):
            ji = block._cjoint.findText(data.get('cond_joint', 'J1'))
            if ji >= 0:
                block._cjoint.setCurrentIndex(ji)  # dispara _update_cond_widgets (muestra/oculta el combo correcto)
            if data.get('cond_joint') == 'Pieza (cinta)':
                pi = block._cpieza.findData(int(data.get('cond_val', 1)))
                if pi >= 0:
                    block._cpieza.setCurrentIndex(pi)
            else:
                oi = block._cop.findText(data.get('cond_op', '>'))
                if oi >= 0:
                    block._cop.setCurrentIndex(oi)
                block._cval.setText(str(data.get('cond_val', 0)))
    except AttributeError:
        pass
    for child in data.get('body', []):
        if block._body is not None:
            _block_restore(child, block._body, editor)
    for child in data.get('else_body', []):
        if block._else_body is not None:
            _block_restore(child, block._else_body, editor)
    return block


class _DragHandle(QLabel):
    """Pequeño tirador '⠿' para arrastrar y reordenar un BlockWidget."""

    def __init__(self, block_widget):
        super().__init__('⠿')
        self._block = block_widget
        self._press_pos = None
        self.setStyleSheet(
            'color:#667788; font-size:9px; background:transparent; border:none;')
        self.setFixedWidth(8)
        self.setCursor(Qt.OpenHandCursor)
        self.setToolTip('Arrastrar para reordenar (dentro del mismo nivel)')

    def mousePressEvent(self, ev):
        if ev.button() == Qt.LeftButton:
            self._press_pos = ev.pos()
        super().mousePressEvent(ev)

    def mouseMoveEvent(self, ev):
        if (self._press_pos is not None
                and (ev.pos() - self._press_pos).manhattanLength() > 8):
            drag = QDrag(self)
            mime = QMimeData()
            mime.setText(str(id(self._block)))
            drag.setMimeData(mime)
            self._press_pos = None
            drag.exec_(Qt.MoveAction)
        super().mouseMoveEvent(ev)

    def mouseReleaseEvent(self, ev):
        self._press_pos = None
        super().mouseReleaseEvent(ev)


class BlockContainer(QFrame):
    """Contenedor vertical de BlockWidgets — barra de acción compacta."""
    _clipboard = None  # portapapeles global de bloques (dict de datos)

    _MENU_SS = (
        'QMenu { background:#1c1f24; color:#ddd; border:1px solid #444; font-size:9px; }'
        'QMenu::item { padding:5px 20px 5px 10px; }'
        'QMenu::item:selected { background:#2a3a50; color:#fff; }'
        'QMenu::separator { height:1px; background:#333; margin:2px 0; }'
    )

    def __init__(self, program_editor, depth=0, label=''):
        super().__init__()
        self._editor = program_editor
        self._depth  = depth
        self._blocks = []

        indent = depth * 14
        _border_colors = ['#4a5568', '#374151', '#2d3748']
        border = _border_colors[min(depth, 2)]

        self.setStyleSheet(
            f'QFrame#bc {{ background:transparent; border:none;'
            f' border-left:2px solid {border}; }}'
        )
        self.setObjectName('bc')

        self._layout = QVBoxLayout(self)
        self._layout.setContentsMargins(indent + 4, 1, 2, 1)
        self._layout.setSpacing(1)
        self.setAcceptDrops(True)

        if label:
            lbl = QLabel(label)
            lbl.setStyleSheet(
                f'color:{border}; font-size:8px; font-weight:bold;'
                f' padding:0px 2px; background:transparent; border:none;'
            )
            self._layout.addWidget(lbl)

        # Barra de acción: ＋ bloque  📍 pos actual  📋 pegar
        bar_w = QWidget()
        bar_w.setStyleSheet('background:transparent; border:none;')
        bar = QHBoxLayout(bar_w)
        bar.setContentsMargins(0, 1, 0, 1)
        bar.setSpacing(2)

        def _mk(icon, tooltip, color, cb):
            b = QPushButton(icon)
            b.setToolTip(tooltip)
            b.setFixedSize(18, 11)
            b.setStyleSheet(
                f'QPushButton {{ border:1px dashed {color}; color:{color};'
                f' font-size:7px; font-weight:bold; background:transparent;'
                f' border-radius:3px; padding:0px; }}'
                f'QPushButton:hover {{ background:{color}33; }}'
            )
            b.clicked.connect(cb)
            return b

        self._add_btn   = _mk('+BLQ', 'Añadir bloque',                    '#7799aa', self._show_add_menu)
        self._pos_btn   = _mk('+POS', 'Insertar posición actual del robot', '#4a9a6a', self._add_current_pos)
        self._paste_btn = _mk('PEG',  'Pegar bloque copiado',              '#8899aa', self._paste_block)

        bar.addWidget(self._add_btn)
        bar.addWidget(self._pos_btn)
        bar.addWidget(self._paste_btn)
        bar.addStretch()
        self._layout.addWidget(bar_w)
        self._bar_widget = bar_w

    # ── Posición actual ───────────────────────────────────────────────────────
    def _add_current_pos(self):
        try:
            import numpy as np
            _angles = (self._editor.real_angles_feedback
                       if getattr(self._editor, '_free_move', False)
                       else self._editor.angles)
            thetas = np.deg2rad(np.array(_angles, dtype=float))
            T = T_MDH_TO_FUSION @ cinematica_directa(thetas, self._editor.rotation_signs_mdh)
            x, y, z = T[0, 3] * 1000, T[1, 3] * 1000, T[2, 3] * 1000
            self._editor._push_undo_snapshot()
            block = self.add_block('move')
            block._mx.setText(f'{x:.1f}')
            block._my.setText(f'{y:.1f}')
            block._mz.setText(f'{z:.1f}')
        except Exception as e:
            print(f'[Rutinas] Error capturando posición: {e}')
            try: self._editor._log_error(f'Captura posición: {e}', 'WARN')
            except Exception: pass

    # ── Portapapeles ──────────────────────────────────────────────────────────
    def _paste_block(self):
        d = BlockContainer._clipboard
        if d is None:
            return
        self._editor._push_undo_snapshot()
        _block_restore(d, self, self._editor)

    # ── Drag & drop para reordenar ──────────────────────────────────────────
    def dragEnterEvent(self, ev):
        if ev.mimeData().hasText():
            try:
                bid = int(ev.mimeData().text())
            except ValueError:
                ev.ignore(); return
            if any(id(b) == bid for b in self._blocks):
                ev.acceptProposedAction()
                return
        ev.ignore()

    def dragMoveEvent(self, ev):
        self.dragEnterEvent(ev)

    def dropEvent(self, ev):
        try:
            bid = int(ev.mimeData().text())
        except ValueError:
            ev.ignore(); return
        src = next((b for b in self._blocks if id(b) == bid), None)
        if src is None:
            ev.ignore(); return
        drop_y = ev.pos().y()
        others = [b for b in self._blocks if b is not src]
        target_idx = len(others)
        for i, b in enumerate(others):
            if drop_y < b.y() + b.height() / 2:
                target_idx = i
                break
        self._editor._push_undo_snapshot()
        self._blocks.remove(src)
        self._blocks.insert(target_idx, src)
        self._reorder_layout()
        ev.acceptProposedAction()

    # ── Menú ──────────────────────────────────────────────────────────────────
    def _show_add_menu(self):
        menu = QMenu(self)
        menu.setStyleSheet(self._MENU_SS)
        items = [
            ('move',       '🟢  Mover a coordenada'),
            ('move_ori',   '🧭  Mover a coordenada + orientación'),
            ('wait',       '⏸  Pausa / Delay'),
            ('for',        '🔵  Repetir N veces'),
            ('while_true', '🔵  Mientras True'),
            ('while_cond', '🔷  Mientras [condición]'),
            ('if',         '🟡  Si [condición]'),
            ('if_else',    '🟠  Si / Si no'),
            ('vacuum',     '💜  Bomba de vacío'),
            ('cinta',      '📦  Cinta transportadora (arranque/paro)'),
            ('home',       '🏠  Ir a Home (0° todas las articulaciones)'),
            ('subroutine', '📎  Subrutina (llamar rutina guardada)'),
        ]
        for btype, blabel in items:
            act = QAction(blabel, menu)
            act.triggered.connect(lambda _, t=btype: self._add_block_with_undo(t))
            menu.addAction(act)
        if BlockContainer._clipboard is not None:
            menu.addSeparator()
            d = BlockContainer._clipboard
            icon = _BLOCK_DEFS.get(d.get('type',''), ('','','',''))[0]
            name = _BLOCK_DEFS.get(d.get('type',''), ('','Bloque','',''))[1]
            act_paste = QAction(f'{icon}  Pegar "{name}"', menu)
            act_paste.triggered.connect(self._paste_block)
            menu.addAction(act_paste)
        menu.exec_(self._add_btn.mapToGlobal(self._add_btn.rect().bottomLeft()))

    # ── Operaciones sobre bloques ─────────────────────────────────────────────
    def add_block(self, block_type):
        block = BlockWidget(block_type, self, self._editor)
        self._blocks.append(block)
        pos = self._layout.indexOf(self._bar_widget)
        self._layout.insertWidget(pos, block)
        self._editor._refresh_prog_size()
        return block

    def _add_block_with_undo(self, block_type):
        """Como add_block, pero registra el estado previo para poder deshacer."""
        self._editor._push_undo_snapshot()
        return self.add_block(block_type)

    def remove_block(self, block):
        if block in self._blocks:
            self._blocks.remove(block)
            self._layout.removeWidget(block)
            block.setParent(None)
            block.deleteLater()
            self._editor._refresh_prog_size()

    def move_up(self, block):
        idx = self._blocks.index(block)
        if idx == 0:
            return
        self._blocks.insert(idx - 1, self._blocks.pop(idx))
        self._reorder_layout()

    def move_down(self, block):
        idx = self._blocks.index(block)
        if idx >= len(self._blocks) - 1:
            return
        self._blocks.insert(idx + 1, self._blocks.pop(idx))
        self._reorder_layout()

    def _reorder_layout(self):
        for b in self._blocks:
            self._layout.removeWidget(b)
        pos = self._layout.indexOf(self._bar_widget)
        for i, b in enumerate(self._blocks):
            self._layout.insertWidget(pos + i, b)
        self._editor._refresh_prog_size()

    def get_data(self, include_widget=False):
        return [b.get_data(include_widget) for b in self._blocks]

    def refresh_var_combos(self):
        """Actualiza el combo de variables en todos los bloques 'move'/'move_ori' en modo var."""
        for bw in self._blocks:
            if hasattr(bw, '_var_combo'):   # solo 'move' y 'move_ori' tienen _var_combo
                bw._refresh_var_combo()
            if bw._body is not None:
                bw._body.refresh_var_combos()
            if bw._else_body is not None:
                bw._else_body.refresh_var_combos()

    def clear_all(self):
        for b in list(self._blocks):
            self.remove_block(b)


class BlockWidget(QFrame):
    """Widget visual compacto tipo nodo de diagrama de flujo."""

    _INP = ('background:#0d1014; color:#e0e0e0; border:1px solid #3a3f4a;'
            ' border-radius:3px; padding:1px 4px; font-size:9px;')
    _CMB = ('QComboBox { background:#0d1014; color:#ddd; border:1px solid #3a3f4a;'
            ' border-radius:3px; font-size:9px; padding:1px 3px; }'
            'QComboBox::drop-down { width:14px; border:none; }'
            'QComboBox QAbstractItemView { background:#1c1f24; color:#ddd; '
            ' selection-background-color:#2a3a50; border:1px solid #444; }')

    def __init__(self, block_type, container, program_editor):
        super().__init__()
        self._type      = block_type
        self._container = container
        self._editor    = program_editor
        self._body      = None
        self._else_body = None
        self._collapsed = False
        self._build_ui()

    def _build_ui(self):
        _, _, bg, border = _BLOCK_DEFS.get(self._type, ('', '', '#1a1d22', '#555'))
        has_body = self._type in ('if', 'if_else', 'for', 'while_true', 'while_cond')
        self._bg_color = bg
        self._border_color = border
        self._executing = False

        # Outer frame: sutil borde izquierdo coloreado, fondo muy oscuro
        self.setStyleSheet(
            f'QFrame#bw {{ background:{bg}; border:none;'
            f' border-left:3px solid {border};'
            f' border-radius:0px 4px 4px 0px; margin:1px 0; }}'
        )
        self.setObjectName('bw')

        vbox = QVBoxLayout(self)
        vbox.setContentsMargins(3, 1, 2, 1)
        vbox.setSpacing(1)

        # ── Cabecera compacta ─────────────────────────────────────────────
        hdr = QHBoxLayout()
        hdr.setSpacing(2)

        self._drag_handle = _DragHandle(self)
        hdr.addWidget(self._drag_handle)

        _bs = (f'QPushButton {{ border:none; color:{border}; font-size:8px;'
               f' background:transparent; border-radius:2px; padding:0; }}'
               f'QPushButton:hover {{ background:{border}33; }}')

        if has_body:
            self._collapse_btn = QPushButton('▾')
            self._collapse_btn.setFixedSize(10, 10)
            self._collapse_btn.setStyleSheet(_bs)
            self._collapse_btn.setToolTip('Colapsar / expandir cuerpo')
            self._collapse_btn.clicked.connect(self._toggle_collapse)
            hdr.addWidget(self._collapse_btn)

        self._build_params(hdr, border)
        hdr.addStretch()

        # Botones de control (muy pequeños)
        for icon, tip, cb in [
            ('↑', 'Subir', lambda: self._move_with_undo('up')),
            ('↓', 'Bajar', lambda: self._move_with_undo('down')),
            ('⧉', 'Copiar bloque', self._copy_to_clipboard),
            ('✕', 'Eliminar', self._remove_with_undo),
        ]:
            b = QPushButton(icon)
            b.setFixedSize(10, 10)
            b.setToolTip(tip)
            b.setStyleSheet(_bs)
            b.clicked.connect(cb)
            hdr.addWidget(b)

        vbox.addLayout(hdr)

        # ── Cuerpos anidados ──────────────────────────────────────────────
        if not has_body:
            return

        self._body_widget = QWidget()
        self._body_widget.setStyleSheet('background:transparent; border:none;')
        bvbox = QVBoxLayout(self._body_widget)
        bvbox.setContentsMargins(0, 1, 0, 1)
        bvbox.setSpacing(1)

        t = self._type
        depth = self._container._depth + 1

        if t == 'if':
            bvbox.addWidget(self._sec_lbl('▸ ENTONCES:', border))
            self._body = BlockContainer(self._editor, depth=depth)
            bvbox.addWidget(self._body)

        elif t == 'if_else':
            bvbox.addWidget(self._sec_lbl('▸ ENTONCES:', border))
            self._body = BlockContainer(self._editor, depth=depth)
            bvbox.addWidget(self._body)
            bvbox.addWidget(self._sec_lbl('▸ SI NO:', '#FF5722'))
            self._else_body = BlockContainer(self._editor, depth=depth)
            bvbox.addWidget(self._else_body)

        elif t in ('for', 'while_true', 'while_cond'):
            self._body = BlockContainer(self._editor, depth=depth)
            bvbox.addWidget(self._body)

        vbox.addWidget(self._body_widget)

    def set_executing(self, active):
        """Resalta (o quita el resaltado de) este bloque como el que la
        rutina está ejecutando ahora mismo. Se conserva mientras la rutina
        está en pausa (se ve congelado sobre el bloque en curso, no
        desaparece), y solo se limpia al detener/cancelar o al pasar al
        siguiente bloque."""
        self._executing = active
        if active:
            self.setStyleSheet(
                f'QFrame#bw {{ background:{self._bg_color}; border:2px solid #ffd633;'
                f' border-left:5px solid #ffd633;'
                f' border-radius:2px 4px 4px 2px; margin:1px 0; }}'
            )
        else:
            self.setStyleSheet(
                f'QFrame#bw {{ background:{self._bg_color}; border:none;'
                f' border-left:3px solid {self._border_color};'
                f' border-radius:0px 4px 4px 0px; margin:1px 0; }}'
            )
            self.set_wait_countdown(None)

    def set_wait_countdown(self, secs_remaining):
        """Actualiza la cuenta atrás mostrada en un bloque de Pausa/Delay
        mientras se está ejecutando (o congelada, si la rutina está en
        pausa). None = ocultar."""
        if not hasattr(self, '_wait_countdown_lbl'):
            return
        if secs_remaining is None:
            self._wait_countdown_lbl.setText('')
        else:
            self._wait_countdown_lbl.setText(f'⏳ {secs_remaining:.1f}s')

    def _toggle_collapse(self):
        self._collapsed = not self._collapsed
        self._body_widget.setVisible(not self._collapsed)
        self._collapse_btn.setText('▸' if self._collapsed else '▾')
        self._editor._refresh_prog_size()

    def _copy_to_clipboard(self):
        BlockContainer._clipboard = self.get_data()
        icon = _BLOCK_DEFS.get(self._type, ('','','',''))[0]
        name = _BLOCK_DEFS.get(self._type, ('','Bloque','',''))[1]
        print(f'[Rutinas] Copiado: {icon} {name}')

    def _move_with_undo(self, direction):
        self._editor._push_undo_snapshot()
        if direction == 'up':
            self._container.move_up(self)
        else:
            self._container.move_down(self)

    def _remove_with_undo(self):
        self._editor._push_undo_snapshot()
        self._container.remove_block(self)

    def _sec_lbl(self, text, color):
        l = QLabel(text)
        l.setStyleSheet(
            f'color:{color}; font-size:8px; font-weight:bold;'
            f' background:transparent; border:none; padding:0 2px;')
        return l

    def _lbl(self, text, color='#888888'):
        l = QLabel(text)
        l.setStyleSheet(
            f'color:{color}; font-size:9px; background:transparent; border:none;')
        return l

    def _build_params(self, layout, border):
        from PyQt5.QtWidgets import QSpinBox
        icon, name, _, _ = _BLOCK_DEFS.get(self._type, ('', self._type, '', ''))
        # Badge de tipo — compacto
        badge = QLabel(f'{icon} {name}')
        badge.setStyleSheet(
            f'color:{border}; font-weight:bold; font-size:9px;'
            f' background:transparent; border:none;')
        layout.addWidget(badge)

        if self._type == 'move':
            # Toggle coords / variable
            self._use_var = False
            self._var_toggle = QPushButton('📍 Coords')
            self._var_toggle.setCheckable(True)
            self._var_toggle.setFixedWidth(43)
            self._var_toggle.setStyleSheet(
                'QPushButton { background:#0d1420; color:#6699cc; border:1px solid #336699;'
                ' border-radius:3px; padding:1px 3px; font-size:8px; font-weight:bold; }'
                'QPushButton:checked { background:#1a2a40; color:#88bbff; border-color:#5588cc; }')
            self._var_toggle.clicked.connect(self._toggle_var_mode)
            layout.addWidget(self._var_toggle)

            # Coords frame
            self._coord_frame = QWidget()
            self._coord_frame.setStyleSheet('background:transparent; border:none;')
            cf_lay = QHBoxLayout(self._coord_frame)
            cf_lay.setContentsMargins(0, 0, 0, 0)
            cf_lay.setSpacing(1)
            self._mx = QLineEdit('0');   self._mx.setFixedWidth(29)
            self._my = QLineEdit('0');   self._my.setFixedWidth(29)
            self._mz = QLineEdit('200'); self._mz.setFixedWidth(29)
            for w in (self._mx, self._my, self._mz):
                w.setStyleSheet(self._INP)
            for lbl_txt, w in [('X:', self._mx), ('Y:', self._my), ('Z:', self._mz)]:
                cf_lay.addWidget(self._lbl(lbl_txt))
                cf_lay.addWidget(w)
            layout.addWidget(self._coord_frame)

            # Variable combo (oculto por defecto)
            self._var_combo = QComboBox()
            self._var_combo.setFixedWidth(78)
            self._var_combo.setStyleSheet(self._CMB)
            self._var_combo.setVisible(False)
            layout.addWidget(self._var_combo)

            # Fuente del movimiento cuando se usa variable: por IK (coordenadas
            # XYZ de la variable) o directo (los 6 ángulos guardados con ella,
            # sin pasar por IK — reproduce exactamente la pose capturada).
            # Oculto salvo en modo variable, igual que _var_combo.
            self._var_source_combo = QComboBox()
            self._var_source_combo.addItem('📐 Coordenadas (IK)', 'xyz')
            self._var_source_combo.addItem('🦾 Ángulos (directo)', 'angles')
            self._var_source_combo.setFixedWidth(92)
            self._var_source_combo.setStyleSheet(self._CMB)
            self._var_source_combo.setVisible(False)
            layout.addWidget(self._var_source_combo)

            self._mtraj = QComboBox()
            self._mtraj.addItems(['Articular', 'Lineal', 'Directo', 'Spline'])
            self._mtraj.setFixedWidth(60)
            self._mtraj.setStyleSheet(self._CMB)
            layout.addWidget(self._mtraj)

            # % de velocidad para este movimiento (100 = velocidad normal,
            # configurada por articulación en "Configurar nodos"). Viaja con
            # cada setpoint hasta el firmware, que lo aplica como factor sobre
            # su perfil de velocidad — ver protocolo V=NN en send_setpoints().
            self._vel_spin = QSpinBox()
            self._vel_spin.setRange(10, 100)
            self._vel_spin.setValue(100)
            self._vel_spin.setSuffix('%')
            self._vel_spin.setFixedWidth(56)
            self._vel_spin.setStyleSheet(self._INP)
            self._vel_spin.setToolTip('Velocidad para este movimiento (% de la velocidad máxima configurada en cada articulación)')
            layout.addWidget(self._vel_spin)

        elif self._type == 'move_ori':
            # Toggle coords / variable — igual que 'move'. Las variables de
            # posición solo guardan X/Y/Z (no orientación), así que R/P/Yw
            # siempre se editan a mano, esté o no activado el modo variable.
            self._use_var = False
            self._var_toggle = QPushButton('📍 Coords')
            self._var_toggle.setCheckable(True)
            self._var_toggle.setFixedWidth(43)
            self._var_toggle.setStyleSheet(
                'QPushButton { background:#0d1420; color:#6699cc; border:1px solid #336699;'
                ' border-radius:3px; padding:1px 3px; font-size:8px; font-weight:bold; }'
                'QPushButton:checked { background:#1a2a40; color:#88bbff; border-color:#5588cc; }')
            self._var_toggle.clicked.connect(self._toggle_var_mode)
            layout.addWidget(self._var_toggle)

            # Coords frame (X, Y, Z manuales)
            self._coord_frame = QWidget()
            self._coord_frame.setStyleSheet('background:transparent; border:none;')
            cf_lay = QHBoxLayout(self._coord_frame)
            cf_lay.setContentsMargins(0, 0, 0, 0)
            cf_lay.setSpacing(1)
            self._mx = QLineEdit('0');   self._mx.setFixedWidth(24)
            self._my = QLineEdit('0');   self._my.setFixedWidth(24)
            self._mz = QLineEdit('200'); self._mz.setFixedWidth(24)
            for w in (self._mx, self._my, self._mz):
                w.setStyleSheet(self._INP)
            for lbl_txt, w in [('X:', self._mx), ('Y:', self._my), ('Z:', self._mz)]:
                cf_lay.addWidget(self._lbl(lbl_txt))
                cf_lay.addWidget(w)
            layout.addWidget(self._coord_frame)

            # Variable combo (oculto por defecto)
            self._var_combo = QComboBox()
            self._var_combo.setFixedWidth(78)
            self._var_combo.setStyleSheet(self._CMB)
            self._var_combo.setVisible(False)
            layout.addWidget(self._var_combo)

            # Fuente del movimiento cuando se usa variable: por IK (coordenadas
            # + la orientación R/P/Yw de abajo) o directo (los 6 ángulos
            # guardados con la variable, sin IK — en ese caso R/P/Yw no pintan
            # nada, así que se deshabilitan visualmente).
            self._var_source_combo = QComboBox()
            self._var_source_combo.addItem('📐 Coordenadas (IK)', 'xyz')
            self._var_source_combo.addItem('🦾 Ángulos (directo)', 'angles')
            self._var_source_combo.setFixedWidth(92)
            self._var_source_combo.setStyleSheet(self._CMB)
            self._var_source_combo.setVisible(False)
            self._var_source_combo.currentIndexChanged.connect(self._update_ori_enabled)
            layout.addWidget(self._var_source_combo)

            # Orientación: siempre manual, no depende del toggle de arriba
            self._mroll  = QLineEdit('0'); self._mroll.setFixedWidth(24)
            self._mpitch = QLineEdit('0'); self._mpitch.setFixedWidth(24)
            self._myaw   = QLineEdit('0'); self._myaw.setFixedWidth(24)
            for w in (self._mroll, self._mpitch, self._myaw):
                w.setStyleSheet(self._INP)
            for lbl_txt, w in [('R:', self._mroll), ('P:', self._mpitch), ('Yw:', self._myaw)]:
                layout.addWidget(self._lbl(lbl_txt))
                layout.addWidget(w)

            self._mtraj = QComboBox()
            self._mtraj.addItems(['Articular', 'Lineal', 'Directo', 'Spline'])
            self._mtraj.setFixedWidth(60)
            self._mtraj.setStyleSheet(self._CMB)
            layout.addWidget(self._mtraj)

            self._vel_spin = QSpinBox()
            self._vel_spin.setRange(10, 100)
            self._vel_spin.setValue(100)
            self._vel_spin.setSuffix('%')
            self._vel_spin.setFixedWidth(56)
            self._vel_spin.setStyleSheet(self._INP)
            self._vel_spin.setToolTip('Velocidad para este movimiento (% de la velocidad máxima configurada en cada articulación)')
            layout.addWidget(self._vel_spin)

        elif self._type == 'wait':
            self._wsecs = QLineEdit('1.0')
            self._wsecs.setFixedWidth(24)
            self._wsecs.setStyleSheet(self._INP)
            layout.addWidget(self._wsecs)
            layout.addWidget(self._lbl('s'))
            self._wait_countdown_lbl = QLabel('')
            self._wait_countdown_lbl.setStyleSheet(
                'color:#ffd633; font-size:9px; font-weight:bold;'
                ' background:transparent; border:none;')
            layout.addWidget(self._wait_countdown_lbl)

        elif self._type == 'for':
            self._fn = QLineEdit('3')
            self._fn.setFixedWidth(20)
            self._fn.setStyleSheet(self._INP)
            layout.addWidget(self._fn)
            layout.addWidget(self._lbl('veces'))

        elif self._type == 'while_true':
            pass  # sin parámetros

        elif self._type == 'vacuum':
            self._vbtn = QPushButton('○ Apagada')
            self._vbtn.setCheckable(True)
            self._vbtn.setChecked(False)
            self._vbtn.setFixedWidth(52)
            self._vbtn.setStyleSheet(
                'QPushButton { background:#1a0a2e; color:#888; border:1px solid #555;'
                ' border-radius:3px; padding:2px 4px; font-weight:bold; }'
                'QPushButton:checked { background:#6a0dad; color:#fff; border-color:#e040fb; }')
            def _upd_v(checked, b=self._vbtn):
                b.setText('● Encendida' if checked else '○ Apagada')
            self._vbtn.toggled.connect(_upd_v)
            layout.addWidget(self._vbtn)

        elif self._type == 'cinta':
            self._cbtn = QPushButton('■ Parada')
            self._cbtn.setCheckable(True)
            self._cbtn.setChecked(False)
            self._cbtn.setFixedWidth(58)
            self._cbtn.setStyleSheet(
                'QPushButton { background:#0d251f; color:#888; border:1px solid #555;'
                ' border-radius:3px; padding:2px 4px; font-weight:bold; }'
                'QPushButton:checked { background:#00694f; color:#fff; border-color:#00b894; }')
            def _upd_c(checked, b=self._cbtn):
                b.setText('▶ Arranque' if checked else '■ Parada')
            self._cbtn.toggled.connect(_upd_c)
            layout.addWidget(self._cbtn)

            self._cvel_spin = QSpinBox()
            self._cvel_spin.setRange(0, 100)
            self._cvel_spin.setValue(50)
            self._cvel_spin.setSuffix('%')
            self._cvel_spin.setFixedWidth(56)
            self._cvel_spin.setStyleSheet(self._INP)
            self._cvel_spin.setToolTip('Velocidad de la cinta (% de su velocidad máxima) al arrancar')
            layout.addWidget(self._cvel_spin)

        elif self._type == 'home':
            pass  # sin parámetros — mueve todo a 0°

        elif self._type == 'subroutine':
            self._sub_combo = QComboBox()
            self._sub_combo.setFixedWidth(89)
            self._sub_combo.setStyleSheet(self._CMB)
            self._refresh_sub_combo()
            sub_ref = QPushButton('🔄')
            sub_ref.setFixedSize(12, 12)
            sub_ref.setToolTip('Actualizar lista de rutinas')
            sub_ref.setStyleSheet(
                'QPushButton { border:1px solid #666; color:#999; background:#111;'
                ' border-radius:3px; font-size:8px; padding:0; }'
                'QPushButton:hover { background:#222; }')
            sub_ref.clicked.connect(self._refresh_sub_combo)
            layout.addWidget(self._sub_combo)
            layout.addWidget(sub_ref)

        elif self._type in ('while_cond', 'if', 'if_else'):
            self._cjoint = QComboBox()
            self._cjoint.addItems(
                ['J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'X(mm)', 'Y(mm)', 'Z(mm)', 'Pieza (cinta)'])
            self._cjoint.setFixedWidth(58)
            self._cjoint.setStyleSheet(self._CMB)
            self._cop = QComboBox()
            self._cop.addItems(['>', '<', '>=', '<=', '==', '!='])
            self._cop.setFixedWidth(23)
            self._cop.setStyleSheet(self._CMB)
            self._cval = QLineEdit('0')
            self._cval.setFixedWidth(24)
            self._cval.setStyleSheet(self._INP)
            # Combo alternativo para la condición "Pieza (cinta)": booleana,
            # no tiene sentido pedir operador/valor numérico como en J1../XYZ.
            # Se solapa con _cop/_cval (uno u otro visible según _cjoint).
            self._cpieza = QComboBox()
            self._cpieza.addItem('Detectada', 1)
            self._cpieza.addItem('No detectada', 0)
            self._cpieza.setFixedWidth(72)
            self._cpieza.setStyleSheet(self._CMB)
            self._cpieza.setVisible(False)
            self._cjoint.currentIndexChanged.connect(self._update_cond_widgets)
            layout.addWidget(self._cjoint)
            layout.addWidget(self._cop)
            layout.addWidget(self._cval)
            layout.addWidget(self._cpieza)
            self._update_cond_widgets()

    def _refresh_sub_combo(self):
        if not hasattr(self, '_sub_combo'):
            return
        current = self._sub_combo.currentText()
        self._sub_combo.clear()
        if os.path.isdir(RUTINAS_DIR):
            for fn in sorted(os.listdir(RUTINAS_DIR)):
                if fn.endswith('.json'):
                    self._sub_combo.addItem(fn[:-5])
        idx = self._sub_combo.findText(current)
        if idx >= 0:
            self._sub_combo.setCurrentIndex(idx)

    def _toggle_var_mode(self, checked):
        self._use_var = checked
        self._var_toggle.setText('🔖 Var' if checked else '📍 Coords')
        self._coord_frame.setVisible(not checked)
        self._var_combo.setVisible(checked)
        if hasattr(self, '_var_source_combo'):
            self._var_source_combo.setVisible(checked)
        if checked:
            self._refresh_var_combo()
        self._update_ori_enabled()

    def _refresh_var_combo(self):
        current = self._var_combo.currentText()
        self._var_combo.clear()
        vars_dict = getattr(self._editor, '_position_vars', {})
        for name in sorted(vars_dict.keys()):
            self._var_combo.addItem(name)
        idx = self._var_combo.findText(current)
        if idx >= 0:
            self._var_combo.setCurrentIndex(idx)

    def _update_ori_enabled(self, *_args):
        """En 'move_ori', R/P/Yw solo se usan cuando el movimiento se resuelve
        por IK (coordenadas manuales, o variable en modo 'Coordenadas'). En
        modo 'Ángulos (directo)' el destino ya son los 6 ángulos guardados,
        así que la orientación no interviene — se deshabilitan para que no
        parezca que hacen algo."""
        if self._type != 'move_ori' or not hasattr(self, '_mroll'):
            return
        usa_angulos = (getattr(self, '_use_var', False)
                       and hasattr(self, '_var_source_combo')
                       and self._var_source_combo.currentData() == 'angles')
        for w in (self._mroll, self._mpitch, self._myaw):
            w.setEnabled(not usa_angulos)

    def _update_cond_widgets(self, *_args):
        """En bloques 'while_cond'/'if'/'if_else': "Pieza (cinta)" es una
        condición booleana (hay/no hay pieza), no tiene sentido pedir
        operador+valor numérico como con J1../XYZ — se muestra en su lugar
        el combo _cpieza ('Detectada'/'No detectada')."""
        if self._type not in ('while_cond', 'if', 'if_else') or not hasattr(self, '_cpieza'):
            return
        es_pieza = (self._cjoint.currentText() == 'Pieza (cinta)')
        self._cop.setVisible(not es_pieza)
        self._cval.setVisible(not es_pieza)
        self._cpieza.setVisible(es_pieza)

    def get_data(self, include_widget=False):
        """'include_widget' añade una clave '_widget' (referencia a este
        BlockWidget, NO serializable) — solo la usa el motor de ejecución
        para poder resaltar visualmente el bloque en curso; el guardado a
        JSON de rutinas SIEMPRE llama a get_data() sin este flag."""
        d = {'type': self._type}
        if include_widget:
            d['_widget'] = self
        try:
            if self._type == 'move':
                if getattr(self, '_use_var', False) and self._var_combo.currentText():
                    d['var'] = self._var_combo.currentText()
                    d['traj'] = self._mtraj.currentText()
                    if hasattr(self, '_var_source_combo'):
                        d['var_mode'] = self._var_source_combo.currentData() or 'xyz'
                else:
                    d.update(x=float(self._mx.text() or 0),
                             y=float(self._my.text() or 0),
                             z=float(self._mz.text() or 0),
                             traj=self._mtraj.currentText())
                if hasattr(self, '_vel_spin'):
                    d['vel_pct'] = self._vel_spin.value()
            elif self._type == 'move_ori':
                if getattr(self, '_use_var', False) and self._var_combo.currentText():
                    d['var'] = self._var_combo.currentText()
                    d.update(roll=float(self._mroll.text() or 0),
                             pitch=float(self._mpitch.text() or 0),
                             yaw=float(self._myaw.text() or 0),
                             traj=self._mtraj.currentText())
                    if hasattr(self, '_var_source_combo'):
                        d['var_mode'] = self._var_source_combo.currentData() or 'xyz'
                else:
                    d.update(x=float(self._mx.text() or 0),
                             y=float(self._my.text() or 0),
                             z=float(self._mz.text() or 0),
                             roll=float(self._mroll.text() or 0),
                             pitch=float(self._mpitch.text() or 0),
                             yaw=float(self._myaw.text() or 0),
                             traj=self._mtraj.currentText())
                if hasattr(self, '_vel_spin'):
                    d['vel_pct'] = self._vel_spin.value()
            elif self._type == 'wait':
                d['seconds'] = float(self._wsecs.text() or 1)
            elif self._type == 'for':
                d['n'] = max(1, int(self._fn.text() or 1))
            elif self._type == 'vacuum':
                d['state'] = self._vbtn.isChecked()
            elif self._type == 'cinta':
                d['state'] = self._cbtn.isChecked()
                d['vel_pct'] = self._cvel_spin.value()
            elif self._type == 'subroutine':
                d['name'] = self._sub_combo.currentText()
            elif self._type in ('while_cond', 'if', 'if_else'):
                if self._cjoint.currentText() == 'Pieza (cinta)':
                    # Condición booleana: se guarda como cond_op='==' y
                    # cond_val 1/0 para reutilizar el mismo mecanismo de
                    # evaluación genérico (_eval_prog_cond) que J1../XYZ.
                    d.update(cond_joint='Pieza (cinta)', cond_op='==',
                             cond_val=float(self._cpieza.currentData()))
                else:
                    d.update(cond_joint=self._cjoint.currentText(),
                             cond_op=self._cop.currentText(),
                             cond_val=float(self._cval.text() or 0))
        except (ValueError, AttributeError):
            pass
        if self._body is not None:
            d['body'] = self._body.get_data(include_widget)
        if self._else_body is not None:
            d['else_body'] = self._else_body.get_data(include_widget)
        return d
