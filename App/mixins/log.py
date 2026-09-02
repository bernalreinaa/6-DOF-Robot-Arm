# -*- coding: utf-8 -*-
"""Pestaña de registro (Log) de eventos y errores del sistema, con historial persistente en disco."""
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

from app_paths import LOG_DIR


class LogMixin:
    """Ver Pestaña de registro (Log) de eventos y errores del sistema, con historial persistente en disco."""

    def _build_log_tab(self):
        """Pestaña de log de errores con historial persistente."""
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.setContentsMargins(3, 3, 3, 3)
        lay.setSpacing(2)

        # Toolbar
        tb = QHBoxLayout()
        lbl = QLabel("Errores y eventos del sistema")
        lbl.setStyleSheet("color:#aaa; font-size:9px;")
        tb.addWidget(lbl)
        tb.addStretch()
        btn_clear = QPushButton("🗑 Limpiar")
        btn_open  = QPushButton("📂 Abrir archivo")
        btn_clear.setStyleSheet("border-color:#ff6b6b; color:#ff6b6b; padding:2px 8px;")
        btn_open.setStyleSheet("border-color:#88aacc; color:#88aacc; padding:2px 8px;")
        btn_clear.clicked.connect(self._log_clear)
        btn_open.clicked.connect(self._log_open_file)
        tb.addWidget(btn_clear)
        tb.addWidget(btn_open)
        lay.addLayout(tb)

        self._log_widget = QTextEdit()
        self._log_widget.setReadOnly(True)
        self._log_widget.setStyleSheet(
            "background:#080c10; color:#cccccc; font-family:Consolas,monospace;"
            " font-size:9px; border:1px solid #222; border-radius:4px;")
        self._log_widget.setPlaceholderText("Sin eventos registrados.")
        lay.addWidget(self._log_widget)

        return w

    def _log_error(self, msg, level='ERROR'):
        """Registra un evento en el panel de log y en el archivo de log del día."""
        import datetime
        now = datetime.datetime.now()
        ts  = now.strftime('%H:%M:%S')
        date_str = now.strftime('%Y-%m-%d')
        colors = {'ERROR': '#ff6b6b', 'WARN': '#ffaa44', 'INFO': '#70e570', 'SERIAL': '#44aaff'}
        color  = colors.get(level, '#cccccc')
        line   = f"[{ts}] [{level}] {msg}"
        if self._log_widget is not None:
            self._log_widget.append(
                f'<span style="color:#555">[{ts}]</span> '
                f'<span style="color:{color};font-weight:bold">[{level}]</span> '
                f'<span style="color:#ddd">{msg}</span>')
            # Scroll al final
            sb = self._log_widget.verticalScrollBar()
            sb.setValue(sb.maximum())
        # Escribir al archivo
        try:
            log_path = os.path.join(LOG_DIR, f'errors_{date_str}.log')
            with open(log_path, 'a', encoding='utf-8') as f:
                f.write(line + '\n')
        except Exception:
            pass

    def _log_clear(self):
        if self._log_widget:
            self._log_widget.clear()

    def _log_open_file(self):
        import datetime, subprocess, platform
        date_str = datetime.datetime.now().strftime('%Y-%m-%d')
        log_path = os.path.join(LOG_DIR, f'errors_{date_str}.log')
        if not os.path.exists(log_path):
            QMessageBox.information(self, "Sin archivo", "No hay archivo de log para hoy.")
            return
        try:
            if platform.system() == 'Windows':
                os.startfile(log_path)
            else:
                subprocess.Popen(['xdg-open', log_path])
        except Exception as e:
            QMessageBox.warning(self, "Error", str(e))
