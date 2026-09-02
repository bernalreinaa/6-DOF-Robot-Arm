# -*- coding: utf-8 -*-
"""Widgets Qt pequeños y reutilizables que no encajan en ningún otro módulo."""
from PyQt5.QtWidgets import QWidget
from PyQt5.QtGui import QPainter, QPen, QColor, QFont
from PyQt5.QtCore import Qt
from collections import deque

class ResponseGraph(QWidget):
    """Gráfica en tiempo real de setpoint vs feedback para tuning PID."""
    MAXPTS = 300

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(90)
        self._sp = deque(maxlen=self.MAXPTS)
        self._fb = deque(maxlen=self.MAXPTS)
        self.setStyleSheet('background:#0a0e14; border:1px solid #333;')

    def push(self, setpoint, feedback):
        self._sp.append(setpoint)
        self._fb.append(feedback)
        self.update()

    def clear_data(self):
        self._sp.clear(); self._fb.clear(); self.update()

    def paintEvent(self, event):
        from PyQt5.QtGui import QPainter
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(0, 0, w, h, QColor('#0a0e14'))
        # grid
        p.setPen(QPen(QColor('#1a2030'), 1))
        for i in range(1, 5):
            gy = int(h * i / 4)
            p.drawLine(0, gy, w, gy)
        if not self._fb:
            p.setPen(QPen(QColor('#445566')))
            p.setFont(QFont('monospace', 9))
            p.drawText(0, 0, w, h, Qt.AlignCenter, 'Mueve el brazo para ver datos')
            p.end(); return
        # auto-range
        vals = list(self._sp) + list(self._fb)
        mn, mx = min(vals), max(vals)
        span = max(mx - mn, 4.0)
        pad = span * 0.15
        mn -= pad; mx += pad + pad
        def ty(v):
            return int(h - (v - mn) / (mx - mn) * h)
        def draw_line(hist, col, lw=1.5):
            pts = list(hist)
            nn = len(pts)
            if nn < 2: return
            pen = QPen(QColor(col), lw)
            p.setPen(pen)
            for ii in range(1, nn):
                x0 = int((ii-1) / (self.MAXPTS-1) * w)
                x1 = int(ii / (self.MAXPTS-1) * w)
                p.drawLine(x0, ty(pts[ii-1]), x1, ty(pts[ii]))
        draw_line(self._sp, '#ff9900', 1.5)
        draw_line(self._fb, '#44ddff', 1.5)
        # legend + values
        p.setFont(QFont('monospace', 8))
        p.setPen(QPen(QColor('#ff9900'))); p.drawText(6, 14, '─ Setpoint')
        p.setPen(QPen(QColor('#44ddff'))); p.drawText(6, 26, '─ Feedback')
        if self._sp and self._fb:
            sp_v, fb_v = self._sp[-1], self._fb[-1]
            err = (sp_v - fb_v + 180.0) % 360.0 - 180.0  # distancia angular más corta
            p.setPen(QPen(QColor('#aaaaaa')))
            p.drawText(w - 110, 14, f'SP={sp_v:7.2f}°')
            p.drawText(w - 110, 26, f'FB={fb_v:7.2f}°')
            ec = '#ff6b6b' if abs(err) > 2 else '#70e570'
            p.setPen(QPen(QColor(ec)))
            p.drawText(w - 110, 38, f'Err={err:+.2f}°')
        p.end()
