# -*- coding: utf-8 -*-
"""
Punto de entrada histórico de la aplicación (variante para la pantalla
táctil de 800x480 de la Raspberry Pi 5, ver apartado 5.2 de la memoria).

Desde la refactorización a paquete (apartado 5.2), este fichero es solo un
punto de entrada fino: toda la lógica vive en los módulos de este mismo
directorio (app_paths.py, kinematics.py, widgets.py,
routine_editor_widgets.py, dialogs.py, main_window.py, mixins/*.py). Se
conserva este nombre de fichero, en vez de borrarlo, para no romper accesos
directos ni scripts de lanzamiento existentes que ya apunten a él.
"""
from app import main

if __name__ == "__main__":
    import sys
    sys.exit(main())
