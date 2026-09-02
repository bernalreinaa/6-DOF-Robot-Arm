# -*- coding: utf-8 -*-
"""Punto de entrada de la aplicación: splash de arranque y bucle de Qt."""
import sys
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

from app_paths import BASE_DIR
from main_window import BrazoRobot


def main():
    app = QApplication(sys.argv)
    # Estilo global de los tooltips (QToolTip no hereda ningún estilo por
    # defecto en esta app oscura; sin esto, algunos temas de sistema pintan
    # el tooltip con texto oscuro sobre fondo oscuro — se ve como un
    # recuadro negro vacío, aunque el texto de ayuda SÍ está puesto).
    app.setStyleSheet(
        'QToolTip { background-color:#1c1f24; color:#e0e0e0;'
        ' border:1px solid #4a5568; padding:4px 6px; font-size:11px; }'
    )
    _base_font = app.font()
    _base_font.setPointSize(max(7, int(_base_font.pointSize() * 0.8)))
    app.setFont(_base_font)

    _splash_path = os.path.join(BASE_DIR, "brazo_robotico_2.png")
    splash_pix = QPixmap(_splash_path) if os.path.exists(_splash_path) \
        else QPixmap(1520, 722)
    if splash_pix.isNull():
        splash_pix.fill(Qt.darkGray)

    splash = QSplashScreen(splash_pix, Qt.WindowStaysOnTopHint)
    splash.setWindowFlag(Qt.FramelessWindowHint)
    splash.show()
    app.processEvents()   # pinta el splash ANTES de la carga bloqueante

    ventana = BrazoRobot()
    ventana.setWindowTitle("Brazo Robótico 6 GDL — Control y Simulación")
    ventana.resize(800, 480)
    ventana.setWindowFlag(Qt.FramelessWindowHint)
    ventana.showFullScreen()  # modo kiosco: pantalla táctil 800x480 dedicada

    # splash.finish cierra el splash en cuando ventana.show() se llame
    splash.finish(ventana)
    QTimer.singleShot(500, ventana.show)
    return app.exec_()

if __name__ == "__main__":
    sys.exit(main())
