# -*- coding: utf-8 -*-
"""
Rutas y constantes compartidas por toda la aplicación (App/): carpeta base,
carpetas de datos persistentes (rutinas, logs, backups, perfiles de
herramienta) y disponibilidad de pyserial. Se calcula una sola vez aquí y
el resto de módulos importan estas constantes en vez de recalcularlas.
"""
import sys
import os

# Carpeta base de la app: junto al .py cuando corre desde código fuente, o
# junto al ejecutable cuando está empaquetada con PyInstaller (modo --onedir:
# sys.executable vive en dist/BrazoRobot/, __file__ dentro de ese caso puede
# apuntar al bundle interno, no sirve). Sin esto, cosas como los meshes del
# brazo (P1.obj..P7.obj) o la imagen de splash solo se cargaban si la app se
# lanzaba con el directorio de trabajo puesto A MANO en la carpeta del
# proyecto — al empaquetar para Raspberry Pi (o lanzarla desde un acceso
# directo/otro cwd) fallaba en silencio y el brazo 3D salía sin piezas.
if getattr(sys, 'frozen', False):
    BASE_DIR = os.path.dirname(os.path.abspath(sys.executable))
else:
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# Carpeta donde se guardan las rutinas (junto al ejecutable/.py)
RUTINAS_DIR = os.path.join(BASE_DIR, 'rutinas')
os.makedirs(RUTINAS_DIR, exist_ok=True)
LOG_DIR = os.path.join(BASE_DIR, 'logs')
os.makedirs(LOG_DIR, exist_ok=True)
BACKUPS_DIR = os.path.join(BASE_DIR, 'backups')
os.makedirs(BACKUPS_DIR, exist_ok=True)
TOOLS_DIR = os.path.join(BASE_DIR, 'herramientas')
os.makedirs(TOOLS_DIR, exist_ok=True)
TOOLS_FILE = os.path.join(TOOLS_DIR, 'perfiles_tcp.json')

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
