# ══════════════════════════════════════════════════════════════════
# CINEMATICA_3D_VISUAL_py.py
# Variante de CINEMATICA_3D_VISUAL.py con la interfaz escalada para
# caber en una pantalla táctil de 800x480 (Raspberry Pi 5, panel DSI).
# Generada automáticamente a partir del archivo original — cualquier
# cambio de LÓGICA (no de tamaños de UI) debe replicarse a mano en
# ambos archivos.
# ══════════════════════════════════════════════════════════════════
import sys
import os
import numpy as np
from PyQt5.QtCore import QTimer
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
from pyvistaqt import QtInteractor
import pyvista as pv
import vtk
import json
from collections import deque

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
 
 
# ─────────────────────────────────────────────
#  Funciones matemáticas de transformación
# ─────────────────────────────────────────────
 
def transform_matrix_mdh(alpha, a, d, theta, offset):
    q = theta + offset
    T = np.array([
        [np.cos(q),               -np.sin(q),              0,            a             ],
        [np.sin(q)*np.cos(alpha),  np.cos(q)*np.cos(alpha), -np.sin(alpha), -d*np.sin(alpha)],
        [np.sin(q)*np.sin(alpha),  np.cos(q)*np.sin(alpha),  np.cos(alpha),  d*np.cos(alpha)],
        [0,                        0,                        0,            1             ]
    ])
    return T
 
 
def numpy_to_vtk_matrix(mat):
    vtk_mat = vtk.vtkMatrix4x4()
    for i in range(4):
        for j in range(4):
            vtk_mat.SetElement(i, j, mat[i, j])
    return vtk_mat
 
def translate(v):
    T = np.identity(4); T[0:3, 3] = v; return T
 
def rotate_x(theta):
    R = np.identity(4); c, s = np.cos(theta), np.sin(theta)
    R[1,1]=c; R[1,2]=-s; R[2,1]=s; R[2,2]=c; return R
 
def rotate_y(theta):
    R = np.identity(4); c, s = np.cos(theta), np.sin(theta)
    R[0,0]=c; R[0,2]=s; R[2,0]=-s; R[2,2]=c; return R
 
def rotate_z(theta):
    R = np.identity(4); c, s = np.cos(theta), np.sin(theta)
    R[0,0]=c; R[0,1]=-s; R[1,0]=s; R[1,1]=c; return R
 
 
# ─────────────────────────────────────────────
#  Parámetros MDH y constantes
# ─────────────────────────────────────────────
 
MDH_A      = np.array([0,        -0.064,   0.227,    0.051,    0,        0       ], dtype=float)
MDH_D      = np.array([0.2076,    0,        0,        0.247263, 0,       -0.05262 ], dtype=float)
MDH_ALPHA  = np.array([0,        -np.pi/2,  0,       -np.pi/2,  np.pi/2,  np.pi/2], dtype=float)
MDH_OFFSET = np.array([0,        -np.pi/2,  0,        0,         np.pi/2,  0      ], dtype=float)
 
# Rotación 180° en Z: alinea el frame base MDH con el frame de Fusion 360.
T_MDH_TO_FUSION = np.array([
    [-1,  0,  0,  0],
    [ 0, -1,  0,  0],
    [ 0,  0,  1,  0],
    [ 0,  0,  0,  1]
], dtype=float)

# Offset de herramienta (TCP) activo — transformación 4x4 (rotación + traslación
# en metros, misma escala que MDH_A/MDH_D) aplicada DESPUÉS del flange final del
# robot. Identidad = sin herramienta / offset nulo. Se actualiza con
# set_tool_offset() al cambiar de perfil de herramienta desde la GUI.
TOOL_OFFSET_T = np.eye(4)
 
 
# ─────────────────────────────────────────────
#  Cinemática Directa
# ─────────────────────────────────────────────
 
def cinematica_directa(thetas, rotation_signs_mdh=None):
    t = np.array(thetas, dtype=float)
    if rotation_signs_mdh is not None:
        t = t * np.array(rotation_signs_mdh, dtype=float)
    T_total = np.eye(4)
    for i in range(6):
        T_total = T_total @ transform_matrix_mdh(MDH_ALPHA[i], MDH_A[i], MDH_D[i], t[i], MDH_OFFSET[i])
    # Offset de herramienta (TCP) activo — ver TOOL_OFFSET_T / set_tool_offset().
    return T_total @ TOOL_OFFSET_T
 
 
def transformaciones_acumuladas(thetas, rotation_signs_mdh=None):
    t = np.array(thetas, dtype=float)
    if rotation_signs_mdh is not None:
        t = t * np.array(rotation_signs_mdh, dtype=float)
    T_total = np.eye(4)
    transforms = []
    for i in range(6):
        T_total = T_total @ transform_matrix_mdh(MDH_ALPHA[i], MDH_A[i], MDH_D[i], t[i], MDH_OFFSET[i])
        transforms.append(T_total.copy())
    return transforms
 
 
# ─────────────────────────────────────────────
#  Cinemática Inversa — Damped Least Squares
# ─────────────────────────────────────────────
 
def jacobiano_numerico(q, rotation_signs_mdh, delta=1e-7):
    """
    Jacobiano numérico de posición (3×6).
    Calcula ∂p/∂qᵢ por diferencias finitas.
    """
    J  = np.zeros((3, 6))
    p0 = (T_MDH_TO_FUSION @ cinematica_directa(q, rotation_signs_mdh))[:3, 3]
    for i in range(6):
        q_d    = q.copy()
        q_d[i] += delta
        p_d    = (T_MDH_TO_FUSION @ cinematica_directa(q_d, rotation_signs_mdh))[:3, 3]
        J[:, i] = (p_d - p0) / delta
    return J
 
 
def rpy_from_matrix(R):
    """
    Extrae ángulos RPY (Roll-Pitch-Yaw, convención ZYX) de una matriz de
    rotación 3×3.  Devuelve [roll, pitch, yaw] en radianes.
    """
    pitch = np.arcsin(np.clip(-R[2, 0], -1.0, 1.0))
    if abs(np.cos(pitch)) > 1e-6:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw  = np.arctan2(R[1, 0], R[0, 0])
    else:                               # gimbal lock (pitch ≈ ±90°)
        roll = 0.0
        yaw  = np.arctan2(-R[0, 1], R[1, 1])
    return np.array([roll, pitch, yaw])


def rpy_to_matrix(rpy):
    """
    Construye una matriz de rotación 3×3 desde RPY (ZYX):
    R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    """
    r, p, y = rpy
    Rx = np.array([[1,      0,       0     ],
                   [0,      np.cos(r), -np.sin(r)],
                   [0,      np.sin(r),  np.cos(r)]])
    Ry = np.array([[ np.cos(p), 0, np.sin(p)],
                   [ 0,         1, 0        ],
                   [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0],
                   [np.sin(y),  np.cos(y), 0],
                   [0,          0,          1]])
    return Rz @ Ry @ Rx


def set_tool_offset(dx_mm=0.0, dy_mm=0.0, dz_mm=0.0,
                     roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0):
    """
    Actualiza la transformación global TOOL_OFFSET_T (offset de TCP) aplicada
    tras el flange final en cinematica_directa(). dx/dy/dz en mm (se
    convierten a metros, misma escala que el resto de la cadena MDH).
    """
    global TOOL_OFFSET_T
    R = rpy_to_matrix(np.deg2rad([roll_deg, pitch_deg, yaw_deg]))
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3]  = np.array([dx_mm, dy_mm, dz_mm], dtype=float) / 1000.0
    TOOL_OFFSET_T = T


def ori_error_vec(R_target, R_current):
    """
    Error de orientación como vector de rotación (3,) en radianes.
    Usa el vector anti-simétrico de R_err = R_target @ R_current.T.
    Válido para errores grandes; linealización para DLS.
    """
    R_err = R_target @ R_current.T
    # Ángulo de la rotación residual
    cos_t = np.clip((np.trace(R_err) - 1.0) / 2.0, -1.0, 1.0)
    theta = np.arccos(cos_t)
    skew  = np.array([R_err[2,1]-R_err[1,2],
                      R_err[0,2]-R_err[2,0],
                      R_err[1,0]-R_err[0,1]])
    if abs(np.sin(theta)) > 1e-8:
        return (theta / (2.0 * np.sin(theta))) * skew
    return 0.5 * skew          # límite cuando θ→0


def jacobiano_numerico_6(q, rotation_signs_mdh, delta=1e-7):
    """
    Jacobiano numérico completo (6×6) posición + orientación.
      Filas 0-2: ∂p/∂qᵢ  [m/rad]
      Filas 3-5: eje de rotación diferencial ∂ω/∂qᵢ  [rad/rad]
    """
    J  = np.zeros((6, 6))
    T0 = T_MDH_TO_FUSION @ cinematica_directa(q, rotation_signs_mdh)
    p0 = T0[:3, 3]
    R0 = T0[:3, :3]
    for i in range(6):
        q_d     = q.copy()
        q_d[i] += delta
        T_d     = T_MDH_TO_FUSION @ cinematica_directa(q_d, rotation_signs_mdh)
        # Posición
        J[:3, i] = (T_d[:3, 3] - p0) / delta
        # Orientación: vector anti-simétrico de la rotación diferencial
        dR = T_d[:3, :3] @ R0.T
        J[3:, i] = np.array([dR[2,1]-dR[1,2],
                              dR[0,2]-dR[2,0],
                              dR[1,0]-dR[0,1]]) / (2.0 * delta)
    return J


# ─────────────────────────────────────────────
#  Zonas prohibidas por articulación (reflejan EXACTAMENTE las
#  constantes limit_inf_motor / limit_sup_motor de cada firmware
#  PID_NEMA17_ESP32_X). Si la IK genera una solución con alguna
#  articulación dentro de su zona prohibida, el firmware la
#  rechazará en silencio (el motor no se mueve) — por eso la IK debe
#  conocer estos límites y evitarlos, en vez de descubrirlo después
#  de mandar el setpoint y desincronizar la rutina.
#  None = sin zona prohibida (p.ej. J6).
# ─────────────────────────────────────────────
JOINT_FORBIDDEN_ZONES = {
    0: (90.0, 270.0),   # Articulación 1
    1: (20.0, 265.0),   # Articulación 2 (pasa por 0°: permitido 265°→360°/0°→20°)
    2: (71.0, 269.0),   # Articulación 3
    3: (91.0, 269.0),   # Articulación 4
    4: (180.0, 310.0),  # Articulación 5
    5: None,            # Articulación 6 (sin zona prohibida)
}

# Archivo donde se persisten los cambios que el usuario haga a las zonas
# prohibidas desde "Configuración de nodos" — sin esto, cualquier ajuste se
# perdería al cerrar la aplicación y volvería a los valores de fábrica de
# arriba en la siguiente sesión.
FORBIDDEN_ZONES_FILE = os.path.join(BASE_DIR, 'zonas_prohibidas.json')


def _load_forbidden_zones():
    """Carga las zonas prohibidas guardadas por el usuario (si las hay),
    sobrescribiendo los valores de fábrica de JOINT_FORBIDDEN_ZONES. Se
    llama una vez al importar el módulo."""
    if not os.path.exists(FORBIDDEN_ZONES_FILE):
        return
    try:
        with open(FORBIDDEN_ZONES_FILE, 'r', encoding='utf-8') as f:
            data = json.load(f)
        for k, v in data.items():
            JOINT_FORBIDDEN_ZONES[int(k)] = tuple(v) if v is not None else None
    except Exception as _e:
        print(f'[ZonasProhibidas] Error cargando {FORBIDDEN_ZONES_FILE}: {_e}')


def set_forbidden_zone(joint_idx, lo_deg, hi_deg):
    """Actualiza en caliente la zona prohibida de una articulación (la que
    usa la cinemática inversa para descartar soluciones) y la persiste a
    disco. lo_deg == hi_deg se interpreta como "sin zona prohibida" (None),
    igual que hace el firmware en isInForbiddenZone(). Debe llamarse SIEMPRE
    que se editen/lean los campos "Límite inferior/superior" de
    Configuración de nodos, para que la IK no quede desincronizada respecto
    a los límites reales que tiene cada ESP32 — ver el comentario sobre
    JOINT_FORBIDDEN_ZONES más arriba."""
    JOINT_FORBIDDEN_ZONES[joint_idx] = None if lo_deg == hi_deg else (float(lo_deg), float(hi_deg))
    try:
        with open(FORBIDDEN_ZONES_FILE, 'w', encoding='utf-8') as f:
            json.dump({str(k): (list(v) if v is not None else None)
                       for k, v in JOINT_FORBIDDEN_ZONES.items()}, f, indent=2)
    except Exception as _e:
        print(f'[ZonasProhibidas] Error guardando {FORBIDDEN_ZONES_FILE}: {_e}')


_load_forbidden_zones()


def _wrap360(angle_deg):
    a = angle_deg % 360.0
    if a < 0:
        a += 360.0
    return a


def _angular_diff(a, b):
    """Distancia angular con signo más corta de b a a (grados), teniendo en
    cuenta el wraparound 0/360 — p.ej. _angular_diff(2, 359) = 3, no -357."""
    return ((a - b + 180.0) % 360.0) - 180.0


def _angle_is_forbidden(angle_deg, zone):
    """Misma lógica que isInForbiddenZone() en el firmware."""
    if zone is None:
        return False
    lo, hi = zone
    if lo == hi:
        return False
    a = _wrap360(angle_deg)
    if lo < hi:
        return lo <= a <= hi
    return a >= lo or a <= hi


def _q_respects_joint_limits(q_deg):
    """True si ninguna articulación cae en su zona prohibida."""
    for idx, zone in JOINT_FORBIDDEN_ZONES.items():
        if _angle_is_forbidden(q_deg[idx], zone):
            return False
    return True


def _resolve_anim_q_end(q_start_deg, q_end_deg, n_samples=24):
    """
    Ajusta, por articulacion, el angulo final que usara la interpolacion
    lineal de la animacion en el visor 3D. La IK ya garantiza que el
    angulo final (modulo 360) no cae en zona prohibida, pero el CAMINO
    recto entre el angulo actual y ese final si puede cruzarla -- en
    particular al ir a home (0 grados) desde un angulo cercano a 360 por
    el lado "largo". Si existe una representacion equivalente del mismo
    angulo final (+-360 grados) cuyo camino recto NO cruza la zona
    prohibida, se usa esa en vez de la directa.
    """
    q_end_adj = np.array(q_end_deg, dtype=float).copy()
    for idx in range(len(q_end_adj)):
        zone = JOINT_FORBIDDEN_ZONES.get(idx)
        if zone is None:
            continue
        a0 = float(q_start_deg[idx])
        a1 = float(q_end_deg[idx])
        best = None
        for cand in (a1, a1 - 360.0, a1 + 360.0):
            crosses = False
            for k in range(n_samples + 1):
                t = k / n_samples
                a = a0 + t * (cand - a0)
                if _angle_is_forbidden(a, zone):
                    crosses = True
                    break
            dist = abs(cand - a0)
            if best is None:
                best = (cand, dist, crosses)
            else:
                _, best_dist, best_crosses = best
                if best_crosses and not crosses:
                    best = (cand, dist, crosses)
                elif crosses == best_crosses and dist < best_dist:
                    best = (cand, dist, crosses)
        if best is not None:
            q_end_adj[idx] = best[0]
    return q_end_adj


def _allowed_zone_center(zone):
    """Punto central del arco PERMITIDO (opuesto al centro de la zona
    prohibida, a 180°)."""
    lo, hi = zone
    return (((lo + hi) / 2.0) + 180.0) % 360.0


def _describe_ik_failure(q_deg):
    """Construye un detalle legible del intento de IK que NO convergió:
    el ángulo que necesitaría cada articulación (el mejor candidato
    encontrado) y si esa articulación queda fuera de su rango permitido.
    Sirve para que el log explique por qué la rutina no pudo llegar."""
    partes = []
    for idx in range(6):
        ang  = _wrap360(q_deg[idx])
        zone = JOINT_FORBIDDEN_ZONES.get(idx)
        fuera = _angle_is_forbidden(q_deg[idx], zone)
        marca = ' [FUERA DE RANGO]' if fuera else ''
        partes.append(f'Art.{idx+1}={ang:.1f}°{marca}')
    return ' | '.join(partes)


def _ik_dls_attempt(target_mm, q0_deg, rotation_signs_mdh,
                     target_rpy_deg=None,
                     max_iter=400, tol_mm=1.5, tol_deg=2.0,
                     lam=0.05, alpha=0.5, w_ori=0.5):
    """
    Un único intento de IK por Damped Least Squares, partiendo de q0_deg.
    Es un método local: puede quedar atrapado en mínimos locales o
    singularidades según el punto de partida. Ver ik_dls() para la
    versión con reintentos multi-arranque.
    """
    use_ori    = target_rpy_deg is not None
    target_pos = np.array(target_mm, dtype=float) / 1000.0
    q          = np.deg2rad(np.array(q0_deg, dtype=float))

    if use_ori:
        R_target = rpy_to_matrix(np.deg2rad(np.array(target_rpy_deg, dtype=float)))

    for _ in range(max_iter):
        T           = T_MDH_TO_FUSION @ cinematica_directa(q, rotation_signs_mdh)
        err_pos     = target_pos - T[:3, 3]
        err_pos_mm  = np.linalg.norm(err_pos) * 1000.0

        if use_ori:
            err_ori     = ori_error_vec(R_target, T[:3, :3])
            err_ori_deg = np.degrees(np.linalg.norm(err_ori))
            if err_pos_mm < tol_mm and err_ori_deg < tol_deg:
                return np.rad2deg(q), err_pos_mm, err_ori_deg, True

            J    = jacobiano_numerico_6(q, rotation_signs_mdh)
            err_v = np.concatenate([err_pos, w_ori * err_ori])
            J_w  = np.vstack([J[:3, :], w_ori * J[3:, :]])
            J_dls = J_w.T @ np.linalg.inv(J_w @ J_w.T + lam**2 * np.eye(6))
        else:
            if err_pos_mm < tol_mm:
                return np.rad2deg(q), err_pos_mm, 0.0, True

            J     = jacobiano_numerico(q, rotation_signs_mdh)
            err_v = err_pos
            J_dls = J.T @ np.linalg.inv(J @ J.T + lam**2 * np.eye(3))

        dq = alpha * J_dls @ err_v

        # Limitar paso máximo a 15° por iteración
        norm = np.linalg.norm(dq)
        if norm > np.radians(15):
            dq *= np.radians(15) / norm

        q += dq
        q  = np.clip(q, -2 * np.pi, 2 * np.pi)

    T          = T_MDH_TO_FUSION @ cinematica_directa(q, rotation_signs_mdh)
    err_pos_mm = np.linalg.norm(target_pos - T[:3, 3]) * 1000.0
    if use_ori:
        err_ori_deg = np.degrees(np.linalg.norm(ori_error_vec(R_target, T[:3, :3])))
        return np.rad2deg(q), err_pos_mm, err_ori_deg, False
    return np.rad2deg(q), err_pos_mm, 0.0, err_pos_mm < tol_mm


def ik_dls(target_mm, q0_deg, rotation_signs_mdh,
           target_rpy_deg=None,
           max_iter=400, tol_mm=1.5, tol_deg=2.0,
           lam=0.05, alpha=0.5, w_ori=0.5):
    """
    Cinemática inversa por Damped Least Squares, con reintentos
    multi-arranque.

    El método DLS es local: la convergencia depende del punto de
    partida (q0_deg). Si el objetivo es alcanzable pero la primera
    pasada (desde q0_deg) queda atrapada en un mínimo local o una
    singularidad, se reintenta desde varias semillas alternativas
    (home, y perturbaciones aleatorias de q0_deg) antes de declarar
    "sin solución". Esto evita falsos negativos cuando el robot ya
    ha demostrado (p.ej. moviéndolo manualmente) que la posición sí
    es alcanzable.

    Sin target_rpy_deg → IK de posición (3×6, igual que antes).
    Con target_rpy_deg → IK posición + orientación (6×6).

    Parámetros
    ----------
    target_mm      : [x, y, z] en mm
    q0_deg         : ángulos iniciales en grados
    target_rpy_deg : [roll, pitch, yaw] en grados  (None = solo posición)
    tol_mm         : tolerancia de posición en mm
    tol_deg        : tolerancia de orientación en grados (solo con target_rpy_deg)
    lam            : factor de amortiguación DLS
    alpha          : tamaño de paso
    w_ori          : peso de orientación vs posición

    Retorna
    -------
    q_deg       : ángulos solución en grados
    err_pos_mm  : error de posición final en mm
    err_ori_deg : error de orientación final en grados (0 si solo posición)
    converged   : True si ambos errores están dentro de tolerancia
    """
    kwargs = dict(target_rpy_deg=target_rpy_deg, max_iter=max_iter,
                  tol_mm=tol_mm, tol_deg=tol_deg, lam=lam, alpha=alpha,
                  w_ori=w_ori)

    def _ok(cand):
        """Una solución solo es válida si converge en tolerancia Y
        además ninguna articulación cae en su zona prohibida (si no,
        el firmware la rechazaría y la rutina se desincronizaría)."""
        return cand[3] and _q_respects_joint_limits(cand[0])

    q0_arr = np.array(q0_deg, dtype=float)

    # Semilla "centrada en zona permitida": para cada articulación con
    # zona prohibida, arrancamos en el centro de su arco permitido en
    # vez de en q0 (que puede caer dentro de la zona prohibida); el resto
    # de articulaciones mantiene el punto de partida original. Esto
    # aumenta mucho la probabilidad de que el DLS converja a una
    # solución que ya respeta los límites.
    centered_seed = q0_arr.copy()
    for idx, zone in JOINT_FORBIDDEN_ZONES.items():
        if zone is not None:
            centered_seed[idx] = _allowed_zone_center(zone)

    candidates_seeds = [q0_arr, centered_seed]

    best = None
    for seed in candidates_seeds:
        cand = _ik_dls_attempt(target_mm, seed, rotation_signs_mdh, **kwargs)
        if best is None or cand[1] < best[1]:
            best = cand
        if _ok(cand):
            return cand

    rng   = np.random.default_rng(12345)
    seeds = [np.zeros(6)]
    for _ in range(4):
        seeds.append(centered_seed + rng.uniform(-30.0, 30.0, size=6))
    for _ in range(4):
        seeds.append(q0_arr + rng.uniform(-90.0, 90.0, size=6))
    for _ in range(3):
        seeds.append(rng.uniform(-180.0, 180.0, size=6))

    for seed in seeds:
        cand = _ik_dls_attempt(target_mm, seed, rotation_signs_mdh, **kwargs)
        if _ok(cand):
            return cand
        if cand[1] < best[1]:
            best = cand

    # Ninguna semilla produjo una solución que respete a la vez la
    # tolerancia de posición/orientación y las zonas prohibidas: se
    # devuelve la mejor encontrada pero marcada como NO convergida
    # (False), para que quien llame reporte "IK sin solución" en vez
    # de mandar un setpoint que el firmware va a rechazar en silencio.
    if not _q_respects_joint_limits(best[0]):
        best = (best[0], best[1], best[2], False)

    return best


# ─────────────────────────────────────────────
#  Verificación home
# ─────────────────────────────────────────────
_T_home = cinematica_directa(np.zeros(6))
print("=== Home (todos a 0°) ===")
print(f"X: {_T_home[0,3]*1000:.3f} mm  (Esperado: 183.263)")
print(f"Y: {_T_home[1,3]*1000:.3f} mm  (Esperado:   0.000)")
print(f"Z: {_T_home[2,3]*1000:.3f} mm  (Esperado: 432.982)")
 



# ─────────────────────────────────────────────────────────────────────────────
#  Diagrama de flujo — vista de solo lectura de la rutina
# ─────────────────────────────────────────────────────────────────────────────

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


# ─────────────────────────────────────────────
#  Ventana de configuración de nodos
# ─────────────────────────────────────────────

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


class NodeConfigDialog(QDialog):
    """Ventana independiente para configurar los parametros PID y de velocidad
    de cada articulacion del brazo robotico.

    Formato enviado al ESP32:
        init[i]kp;ki;kd;max_vel;cruise_vel;approach_vel;min_vel;tol_deg;slow_zone_deg;approach_zone_deg;\\n
    donde i = 0-based (0..5), sin '=' tras el corchete.
    """

    # (nombre visible, clave interna, valor por defecto)
    PARAMS = [
        ("Kp",                "kp",                5.0  ),
        ("Ki",                "ki",                0.0  ),
        ("Kd",                "kd",                0.1  ),
        ("Vel. maxima",       "max_vel",           15.0 ),
        ("Vel. crucero",      "cruise_vel",         8.0 ),
        ("Vel. aproximacion", "approach_vel",       3.0 ),
        ("Vel. minima",       "min_vel",            1.0 ),
        ("Tolerancia (deg)",  "tol_deg",            1.0 ),
        ("Zona lenta (deg)",  "slow_zone_deg",     10.0 ),
        ("Zona aprox. (deg)", "approach_zone_deg",  5.0 ),
        ("Límite inferior (deg)", "limit_inf_motor", -180.0),
        ("Límite superior (deg)", "limit_sup_motor",  180.0)
    ]

    def __init__(self, main_window):
        super().__init__(main_window)
        self._main = main_window
        self.setWindowTitle("Configuracion de nodos — ESP32")
        self.setMinimumWidth(300)
        self.setStyleSheet(self._theme())
        self._build_ui()
        # Auto-cargar valores de la articulación 1 al abrir
        QTimer.singleShot(150, self._reload_config)
        # Debounce para auto-envío
        self._debounce = QTimer(self)
        self._debounce.setSingleShot(True)
        self._debounce.setInterval(600)
        self._debounce.timeout.connect(self._auto_send)
        # Muestreo de datos para la gráfica (100 ms)
        self._sample_timer = QTimer(self)
        self._sample_timer.setInterval(100)
        self._sample_timer.timeout.connect(self._sample_pid_data)
        self._sample_timer.start()
        self.finished.connect(self._sample_timer.stop)
        # Conectar spinboxes al debounce
        for spin in self._spin_boxes.values():
            spin.valueChanged.connect(self._on_spin_changed)
        # Limpiar gráfica al cambiar articulación
        self.joint_combo.currentIndexChanged.connect(self._resp_graph.clear_data)

    # ── Construccion de la UI ──────────────────────────────────────────────

    def _build_ui(self):
        from PyQt5.QtWidgets import QDoubleSpinBox
        root = QVBoxLayout(self)
        root.setSpacing(6)
        root.setContentsMargins(8, 8, 8, 8)

        # Titulo
        title = QLabel("CONFIGURACION DE NODO / ARTICULACION")
        title.setStyleSheet(
            "font-size:12px; font-weight:bold; color:#ff9900; padding:4px;"
        )
        title.setAlignment(Qt.AlignCenter)
        root.addWidget(title)

        # Selector de articulacion
        sel_row = QHBoxLayout()
        lbl_sel = QLabel("Articulacion:")
        lbl_sel.setStyleSheet("color:#ff9900; font-weight:bold;")
        sel_row.addWidget(lbl_sel)
        self.joint_combo = QComboBox()
        for j in range(1, 7):
            self.joint_combo.addItem(f"Articulacion {j}  (nodo {j})")
        self.joint_combo.setFixedWidth(132)
        self.joint_combo.currentIndexChanged.connect(self._reload_config)
        sel_row.addWidget(self.joint_combo)
        sel_row.addStretch()
        root.addLayout(sel_row)

        # Separador
        sep = QFrame(); sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet("background:#ff9900; max-height:1px;")
        root.addWidget(sep)

        # Grid de parametros — 2 columnas
        grid = QGridLayout()
        grid.setHorizontalSpacing(24)
        grid.setVerticalSpacing(8)
        self._spin_boxes = {}

        for idx, (label_txt, key, default) in enumerate(self.PARAMS):
            col = (idx % 2) * 2   # columnas 0 o 2
            row = idx // 2

            lbl = QLabel(label_txt)
            lbl.setStyleSheet("color:#e0e0e0; font-size:9px; font-weight:normal;")

            spin = QDoubleSpinBox()
            spin.setDecimals(4)
            spin.setRange(-9999.0, 9999.0)
            spin.setSingleStep(0.1)
            spin.setValue(default)
            spin.setFixedWidth(78)
            spin.setStyleSheet(
                "QDoubleSpinBox { background:#2b2e38; border:1.2px solid #ff9900;"
                " border-radius:5px; color:#e0e0e0; padding:3px; }"
                "QDoubleSpinBox::up-button, QDoubleSpinBox::down-button"
                " { width:18px; background:#31364a; border-radius:3px; }"
            )
            if key in ('limit_inf_motor', 'limit_sup_motor'):
                tip = ('Zona prohibida para la cinemática inversa (IK): estos mismos '
                       'valores son los que descarta ik_dls() al buscar solución, y los '
                       'que "Leer desde ESP32"/"Enviar configuración" sincronizan '
                       'automáticamente con el firmware de este nodo. Iguales entre sí '
                       '= sin zona prohibida.')
                spin.setToolTip(tip)
                lbl.setToolTip(tip)
            self._spin_boxes[key] = spin
            grid.addWidget(lbl,  row, col)
            grid.addWidget(spin, row, col + 1)

        root.addLayout(grid)

        zona_hint = QLabel(
            '⚠ Los límites inferior/superior definen también la zona prohibida que '
            'usa la cinemática inversa (IK) del brazo — se sincronizan solos al leer '
            'o enviar la configuración de este nodo.')
        zona_hint.setWordWrap(True)
        zona_hint.setStyleSheet('color:#ffcc66; font-size:8.5px; font-style:italic;')
        root.addWidget(zona_hint)

        # Separador
        sep2 = QFrame(); sep2.setFrameShape(QFrame.HLine)
        sep2.setStyleSheet("background:#ff9900; max-height:1px;")
        root.addWidget(sep2)

        # Fila de botones + estado
        btn_row = QHBoxLayout()
        self._status_lbl = QLabel("")
        self._status_lbl.setStyleSheet("font-size:9px; font-weight:bold;")
        btn_row.addWidget(self._status_lbl)
        btn_row.addStretch()

        btn_reload = QPushButton("Leer desde ESP32")
        btn_reload.setStyleSheet(
            "border:1.5px solid #44bbff; color:#44bbff; font-weight:bold;"
            " border-radius:8px; padding:6px 16px;"
        )
        btn_reload.clicked.connect(self._reload_config)
        btn_row.addWidget(btn_reload)

        btn_send = QPushButton("Enviar configuracion")
        btn_send.setStyleSheet(
            "border:1.5px solid #ff9900; color:#ff9900; font-weight:bold;"
            " border-radius:8px; padding:6px 16px;"
        )
        btn_send.clicked.connect(self._send_config)
        btn_row.addWidget(btn_send)

        btn_zone = QPushButton("Aplicar zona a IK")
        btn_zone.setToolTip(
            'Actualiza la zona prohibida de esta articulación en la cinemática '
            'inversa con los valores de Límite inferior/superior de arriba, sin '
            'necesidad de conexión serie (útil para editar rutinas sin el robot '
            'conectado).')
        btn_zone.setStyleSheet(
            "border:1.5px solid #66cc88; color:#66cc88; font-weight:bold;"
            " border-radius:8px; padding:6px 16px;"
        )
        btn_zone.clicked.connect(self._apply_zone_only)
        btn_row.addWidget(btn_zone)

        self._auto_chk = QCheckBox("Auto-enviar")
        self._auto_chk.setStyleSheet("color:#ff9900; font-weight:bold;")
        self._auto_chk.setToolTip(
            "Envía automáticamente al ESP32 cuando cambias un valor")
        btn_row.addWidget(self._auto_chk)

        btn_close = QPushButton("Cerrar")
        btn_close.setStyleSheet(
            "border:1.5px solid #888; color:#888; border-radius:8px; padding:6px 14px;"
        )
        btn_close.clicked.connect(self.reject)
        btn_row.addWidget(btn_close)

        root.addLayout(btn_row)

        # ── Gráfica de respuesta ──────────────────────────────────────────
        sep3 = QFrame(); sep3.setFrameShape(QFrame.HLine)
        sep3.setStyleSheet("background:#333; max-height:1px;")
        root.addWidget(sep3)
        lbl_graph = QLabel("Respuesta articulación seleccionada")
        lbl_graph.setStyleSheet(
            "font-size:9px; color:#aaa; background:transparent;")
        root.addWidget(lbl_graph)
        self._resp_graph = ResponseGraph(self)
        root.addWidget(self._resp_graph)

    # ── Envio al ESP32 ─────────────────────────────────────────────────────

    def _send_config(self):
        sp = self._main.serial_port
        if not (sp and sp.is_open):
            self._set_status("Sin conexion serie activa", error=True)
            return

        joint_idx = self.joint_combo.currentIndex()   # 0-based

        # Orden exacto que espera procesarComandoInit en el ESP32
        orden = ["kp", "ki", "kd",
                 "max_vel", "cruise_vel", "approach_vel", "min_vel",
                 "tol_deg", "slow_zone_deg", "approach_zone_deg", "limit_inf_motor", "limit_sup_motor"]
        values = [self._spin_boxes[k].value() for k in orden]

        # Formato: init[i]val0;val1;...val9;\n  (i=0-based, sin '=' tras ']')
        params_str = ";".join(f"{v:.4f}" for v in values) + ";"
        cmd = f"init[{joint_idx + 1}]{params_str}\n"

        # Mantener la IK (JOINT_FORBIDDEN_ZONES) sincronizada con lo que se
        # manda al firmware — ver comentario junto a set_forbidden_zone().
        set_forbidden_zone(joint_idx,
                            self._spin_boxes['limit_inf_motor'].value(),
                            self._spin_boxes['limit_sup_motor'].value())

        try:
            sp.write(cmd.encode("ascii"))
            self._set_status(f"Enviado a Articulacion {joint_idx + 1}  OK", error=False)
            print(f"[NodeConfig] {cmd.strip()}")
        except Exception as e:
            self._set_status(f"Error: {e}", error=True)
            print(f"[NodeConfig] Error al enviar: {e}")

    def _apply_zone_only(self):
        """Aplica la zona prohibida a la IK sin necesidad de conexión serie
        (a diferencia de _send_config, que requiere el puerto abierto)."""
        joint_idx = self.joint_combo.currentIndex()
        lo = self._spin_boxes['limit_inf_motor'].value()
        hi = self._spin_boxes['limit_sup_motor'].value()
        set_forbidden_zone(joint_idx, lo, hi)
        if lo == hi:
            self._set_status(f"Articulación {joint_idx + 1}: sin zona prohibida (IK)", error=False)
        else:
            self._set_status(
                f"Articulación {joint_idx + 1}: zona prohibida IK = {lo:.1f}°–{hi:.1f}°",
                error=False)

    def _reload_config(self):
        """Solicita al ESP32 la configuracion actual de un nodo y rellena los campos.

        El ESP32 responde (entre otras lineas de angulos) con:
            reload[1]=3.0;0.2;0.0;2000.0;1000.0;600.0;20.0;0.5;5.0;15.0;
        Como el ESP32 sigue emitiendo angulos[...] cada ~100 ms, hay que leer
        en bucle hasta encontrar la linea que empiece por 'reload['.
        """
        sp = self._main.serial_port
        if not (sp and sp.is_open):
            self._set_status("Sin conexion serie activa", error=True)
            return

        joint_idx = self.joint_combo.currentIndex()   # 0-based en Python
        cmd = f"reload[{joint_idx + 1}]\n"            # 1-based para el ESP32

        # Orden exacto en que el ESP32 construye el mensaje
        orden = ["kp", "ki", "kd",
                 "max_vel", "cruise_vel", "approach_vel", "min_vel",
                 "tol_deg", "slow_zone_deg", "approach_zone_deg", "limit_inf_motor", "limit_sup_motor"]

        try:
            # 1) Pausar el timer de lectura continua para no competir por el buffer
            self._main._serial_rx_timer.stop()

            # 2) Enviar solicitud
            sp.write(cmd.encode("ascii"))
            self._set_status(
                f"Esperando respuesta de Articulacion {joint_idx + 1}...", error=False
            )
            print(f"[NodeReload TX] {cmd.strip()}")

            # 3) Leer líneas hasta encontrar la que empieza por 'reload['
            #    Máximo 30 intentos × timeout 1 s = 30 s de espera máxima
            reload_line = None
            for intento in range(30):
                raw  = sp.readline()
                line = raw.decode("ascii", errors="ignore").strip()
                print(f"[NodeReload RX #{intento}] '{line}'")
                if line.startswith("reload["):
                    reload_line = line
                    break

            if reload_line is None:
                self._set_status("Sin respuesta reload del ESP32 (timeout)", error=True)
                return

            # 4) Parsear formato: reload[i]=v0;v1;v2;...v9;
            params_str = reload_line.split("=", 1)[1]
            tokens = [t.strip() for t in params_str.split(";") if t.strip()]

            if len(tokens) < len(orden):
                self._set_status(
                    f"Respuesta incompleta ({len(tokens)}/{len(orden)} parametros)",
                    error=True
                )
                return

            # 5) Actualizar cada spin-box con su valor posicional
            for i, key in enumerate(orden):
                try:
                    self._spin_boxes[key].setValue(float(tokens[i]))
                except (ValueError, KeyError):
                    print(f"[NodeReload] Error al parsear '{key}' = '{tokens[i]}'")

            # Sincronizar la IK con los límites REALES que acaba de reportar
            # el firmware (ver comentario junto a set_forbidden_zone()).
            set_forbidden_zone(joint_idx,
                                self._spin_boxes['limit_inf_motor'].value(),
                                self._spin_boxes['limit_sup_motor'].value())

            self._set_status(
                f"Articulacion {joint_idx + 1} cargada correctamente  OK", error=False
            )

        except Exception as e:
            self._set_status(f"Error: {e}", error=True)
            print(f"[NodeReload] Error: {e}")

        finally:
            # 6) Reanudar siempre el timer, tanto si hubo éxito como si no
            self._main._serial_rx_timer.start()




    def _on_spin_changed(self):
        if self._auto_chk.isChecked():
            self._debounce.start()

    def _auto_send(self):
        if self._auto_chk.isChecked():
            self._send_config()

    def _sample_pid_data(self):
        idx = self.joint_combo.currentIndex()
        fb  = self._main.real_angles_feedback
        sp  = self._main._joint_setpoints
        if fb and sp and idx < len(fb) and idx < len(sp):
            self._resp_graph.push(sp[idx], fb[idx])

    def _set_status(self, msg, error=False):
        color = "#ff6b6b" if error else "#70e570"
        self._status_lbl.setStyleSheet(
            f"font-size:9px; font-weight:bold; color:{color};"
        )
        self._status_lbl.setText(msg)

    # ── Tema oscuro ────────────────────────────────────────────────────────

    def _theme(self):
        return """
        QDialog     { background:#23262b; color:#e0e0e0;
                      font-family:'Segoe UI',Arial; font-size:10px; }
        QLabel      { color:#93ffa8; font-weight:600; }
        QComboBox   { background:#2b2e38; border:1.2px solid #ff9900;
                      border-radius:5px; color:#e0e0e0; padding:4px; }
        QPushButton { background-color:#31364a; border:1.2px solid #5dd095;
                      border-radius:8px; padding:5px 10px;
                      color:#70e570; font-weight:bold; }
        QPushButton:hover { background-color:#70e570; color:#23262b; }
        QDoubleSpinBox { background:#2b2e38; border:1.2px solid #ff9900;
                         border-radius:5px; color:#e0e0e0; padding:3px; }
        """


class CintaConfigDialog(QDialog):
    """Ventana pequeña para configurar la cinta transportadora: distancia
    umbral de detección del HC-SR04 (enviada por Serial como "cintadist=N.N",
    reenviada por el central a la cinta por ESP-NOW) y una lectura en vivo
    del estado que reporta la propia cinta (objeto detectado / distancia
    medida, ver BrazoRobot._parse_cinta_estado()). El arranque/paro con
    velocidad se maneja desde el bloque 'Cinta transportadora' del editor de
    rutinas, no desde aquí -- esto es solo configuración + monitor."""

    def __init__(self, main_window):
        super().__init__(main_window)
        self._main = main_window
        self.setWindowTitle("Cinta transportadora")
        self.setMinimumWidth(260)
        self.setStyleSheet("""
            QDialog     { background:#23262b; color:#e0e0e0;
                          font-family:'Segoe UI',Arial; font-size:10px; }
            QLabel      { color:#93ffa8; font-weight:600; }
            QPushButton { background-color:#31364a; border:1.2px solid #00b894;
                          border-radius:8px; padding:5px 10px;
                          color:#00e0ac; font-weight:bold; }
            QPushButton:hover { background-color:#00b894; color:#23262b; }
            QDoubleSpinBox { background:#2b2e38; border:1.2px solid #00b894;
                             border-radius:5px; color:#e0e0e0; padding:3px; }
        """)
        from PyQt5.QtWidgets import QDoubleSpinBox
        root = QVBoxLayout(self)
        root.setSpacing(8)
        root.setContentsMargins(12, 12, 12, 12)

        title = QLabel("CINTA TRANSPORTADORA")
        title.setStyleSheet("font-size:12px; font-weight:bold; color:#00e0ac; padding:4px;")
        title.setAlignment(Qt.AlignCenter)
        root.addWidget(title)

        dist_row = QHBoxLayout()
        dist_row.addWidget(QLabel("Distancia de detección (cm):"))
        self.dist_spin = QDoubleSpinBox()
        self.dist_spin.setRange(1.0, 100.0)
        self.dist_spin.setSingleStep(0.5)
        self.dist_spin.setDecimals(1)
        self.dist_spin.setValue(getattr(self._main, 'cinta_distancia_umbral_cm', 4.0))
        self.dist_spin.setFixedWidth(74)
        self.dist_spin.setToolTip(
            'Si el HC-SR04 mide una pieza a esta distancia o menos, la cinta se '
            'para automáticamente (hasta que la pieza se aleje).')
        dist_row.addWidget(self.dist_spin)
        dist_row.addStretch()
        root.addLayout(dist_row)

        btn_send = QPushButton("Enviar distancia")
        btn_send.clicked.connect(self._enviar_distancia)
        root.addWidget(btn_send)

        # Estado en vivo, refrescado cada 300 ms desde lo último que
        # reportó el central por Serial (objetoCinta=/distCinta=).
        self.lbl_estado = QLabel("Objeto: —")
        self.lbl_estado.setStyleSheet("color:#e0e0e0; font-weight:normal;")
        root.addWidget(self.lbl_estado)
        self.lbl_dist_medida = QLabel("Distancia medida: —")
        self.lbl_dist_medida.setStyleSheet("color:#e0e0e0; font-weight:normal;")
        root.addWidget(self.lbl_dist_medida)

        self._refresh_timer = QTimer(self)
        self._refresh_timer.setInterval(300)
        self._refresh_timer.timeout.connect(self._refresh_estado)
        self._refresh_timer.start()
        self.finished.connect(self._refresh_timer.stop)
        self._refresh_estado()

    def _enviar_distancia(self):
        self._main.send_cinta_distancia(self.dist_spin.value())

    def _refresh_estado(self):
        det = getattr(self._main, 'cinta_objeto_detectado', False)
        d = getattr(self._main, 'cinta_distancia_cm', -1.0)
        self.lbl_estado.setText(f"Objeto: {'🟢 DETECTADO' if det else '⚪ no detectado'}")
        self.lbl_dist_medida.setText(
            "Distancia medida: —" if d < 0 else f"Distancia medida: {d:.1f} cm")


class ToolProfileDialog(QDialog):
    """Gestion de perfiles de herramienta (offset TCP) -- pinza, ventosa, etc."""

    def __init__(self, main_window):
        super().__init__(main_window)
        self._main = main_window
        self.setWindowTitle("Perfiles de Herramienta (TCP)")
        self.setMinimumWidth(216)
        self.setStyleSheet(
            "QDialog { background:#1c1f24; color:#ddd; }"
            "QLabel { color:#ccc; }"
            "QLineEdit { background:#0d1014; color:#e0e0e0; border:1px solid #3a3f4a;"
            " border-radius:3px; padding:3px; }"
            "QComboBox { background:#0d1014; color:#ddd; border:1px solid #3a3f4a;"
            " border-radius:3px; padding:3px; }"
            "QPushButton { border:1px solid #26C6DA; color:#26C6DA; background:transparent;"
            " border-radius:4px; padding:4px 10px; }"
            "QPushButton:hover { background:#0d2a3d; }"
        )
        vl = QVBoxLayout(self)

        hint = QLabel(
            "El offset se aplica desde el flange (extremo del robot) hasta la\n"
            "punta real de la herramienta. Util al cambiar pinza/ventosa/otra.")
        hint.setStyleSheet("color:#888; font-size:8px;")
        vl.addWidget(hint)

        row1 = QHBoxLayout()
        row1.addWidget(QLabel("Perfil:"))
        self._combo = QComboBox()
        self._combo.addItems(sorted(self._main._tool_profiles.keys()))
        idx = self._combo.findText(self._main._active_tool_profile)
        if idx >= 0:
            self._combo.setCurrentIndex(idx)
        row1.addWidget(self._combo, 1)
        btn_new = QPushButton("+ Nuevo")
        btn_del = QPushButton("X Eliminar")
        row1.addWidget(btn_new)
        row1.addWidget(btn_del)
        vl.addLayout(row1)

        grid = QGridLayout()
        self._fields = {}
        labels = [('dx', 'X offset (mm)'), ('dy', 'Y offset (mm)'), ('dz', 'Z offset (mm)'),
                  ('roll', 'Roll (grados)'), ('pitch', 'Pitch (grados)'), ('yaw', 'Yaw (grados)')]
        for i, (key, label) in enumerate(labels):
            grid.addWidget(QLabel(label), i, 0)
            e = QLineEdit('0')
            grid.addWidget(e, i, 1)
            self._fields[key] = e
        vl.addLayout(grid)

        self._combo.currentTextChanged.connect(self._load_fields)
        self._load_fields(self._combo.currentText())

        btn_new.clicked.connect(self._new_profile)
        btn_del.clicked.connect(self._delete_profile)

        row2 = QHBoxLayout()
        btn_apply = QPushButton("OK Aplicar y guardar")
        btn_apply.setStyleSheet(
            "border-color:#70e570; color:#70e570; font-weight:bold;")
        btn_apply.clicked.connect(self._apply_and_save)
        btn_close = QPushButton("Cerrar")
        btn_close.clicked.connect(self.accept)
        row2.addWidget(btn_apply)
        row2.addStretch()
        row2.addWidget(btn_close)
        vl.addLayout(row2)

    def _load_fields(self, name):
        p = self._main._tool_profiles.get(name, {})
        for key, e in self._fields.items():
            e.setText(str(p.get(key, 0)))

    def _new_profile(self):
        from PyQt5.QtWidgets import QInputDialog
        name, ok = QInputDialog.getText(self, "Nuevo perfil", "Nombre del perfil:")
        name = name.strip()
        if not ok or not name:
            return
        if name in self._main._tool_profiles:
            QMessageBox.warning(self, "Ya existe", "Ya hay un perfil con ese nombre.")
            return
        self._main._tool_profiles[name] = {'dx': 0, 'dy': 0, 'dz': 0,
                                            'roll': 0, 'pitch': 0, 'yaw': 0}
        self._combo.addItem(name)
        self._combo.setCurrentText(name)

    def _delete_profile(self):
        name = self._combo.currentText()
        if name == 'Sin offset':
            QMessageBox.warning(self, "No permitido",
                                 "El perfil 'Sin offset' no se puede eliminar.")
            return
        if name not in self._main._tool_profiles:
            return
        self._main._tool_profiles.pop(name, None)
        idx = self._combo.findText(name)
        if idx >= 0:
            self._combo.removeItem(idx)

    def _apply_and_save(self):
        name = self._combo.currentText()
        try:
            vals = {k: float(e.text() or 0) for k, e in self._fields.items()}
        except ValueError:
            QMessageBox.warning(self, "Valor invalido", "Revisa los campos numericos.")
            return
        self._main._tool_profiles[name] = vals
        self._main._active_tool_profile = name
        self._main._apply_active_tool_profile()
        self._main._save_tool_profiles()
        self._main.update_xyz_display()
        self._main.update_3d_visualization()
        QMessageBox.information(self, "Aplicado",
                                 f"Perfil '{name}' aplicado y guardado.")



# ─────────────────────────────────────────────
#  Ventana principal
# ─────────────────────────────────────────────

class BrazoRobot(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Simulación brazo robótico STEP")
        self.setStyleSheet(self.dark_theme())
 
        self.angles = [0.0] * 6
        self.serial_port = None   # objeto serial.Serial activo

        # Ghost arm
        self.ghost_actors        = []
        self._ghost_visible      = False

        # Programa por bloques — motor de ejecución
        self._prog_running    = False
        self._prog_paused     = False   # True = congelada, resumible con Run
        self._prog_gen        = None
        self._prog_state      = 'needs_next'   # 'needs_next'|'moving'|'waiting'|'settling'
        self._prog_highlighted_widget = None    # BlockWidget resaltado como "en curso"
        self._prog_anim_step  = 0
        self._prog_anim_total = 0
        self._prog_anim_q_start = None
        self._prog_anim_q_end   = None
        # Ángulos objetivo (0-360, ya resueltos/esquivando zona prohibida)
        # del bloque de movimiento en curso, usados por _prog_settle_tick()
        # para comprobar la llegada real. Independiente de self.angles
        # porque, desde que el setpoint real se manda al empezar el bloque
        # (ver _prog_start_move), self.angles puede seguir cambiando por la
        # animación local del visor 3D mientras el robot real ya llegó.
        self._prog_settle_target = None
        self._prog_wait_ticks   = 0
        # Timeout de seguridad del estado 'settling' (segundos): si tras
        # mandar un setpoint el robot no confirma "llegada estable" antes de
        # este tiempo, se fuerza el avance al siguiente bloque avisando en el
        # log — ver _prog_tick(), estado 'settling'. Editable desde la
        # pestaña Rutinas (self._prog_timeout_spin).
        self._prog_settle_timeout_s = 30.0
        self._prog_timer = QTimer(self)
        self._prog_timer.setInterval(25)
        self._prog_timer.timeout.connect(self._prog_tick)

        # Tolerancia angular para considerar que un movimiento de rutina
        # "llegó" al objetivo (grados). Con 1.5° (el MOVEMENT_TOL del firmware,
        # pensado solo para la baliza) el error cartesiano rondaba 8-13 mm.
        # Con 0.5° bajó a 2-4 mm... pero SOLO en los puntos donde el PID puede
        # realmente asentarse por debajo de 0.5°; en los demás (esquina1,
        # esquina2, HOME) el robot nunca llega a estar tan quieto y cada
        # movimiento agota el timeout de 30 s esperando algo que el propio PID
        # no está configurado para dar (su zona muerta real, tol_deg, es mayor
        # que 0.5° para esas articulaciones/poses). 1.0° es el punto medio:
        # sigue mejorando bastante sobre 1.5° sin forzar timeout en casi todos
        # los movimientos. Si algún punto concreto sigue dando timeout con
        # esto, el límite ya no está aquí sino en tol_deg de ese PID (Config.
        # de nodos) — apretar más este valor solo quemará más tiempo sin
        # ganar precisión real. Editable desde la pestaña Rutinas
        # (self._prog_tol_spin).
        self._PROG_MOVEMENT_TOL_DEG = 1.0

        # Ventana de estabilidad (segundos) del estado 'settling': tiempo
        # mínimo seguido con max_delta<0.5° Y dentro de _PROG_MOVEMENT_TOL_DEG
        # antes de dar el bloque por "llegado" (ver _prog_tick()). Por
        # defecto 0.5 s = 20 ticks de 25 ms. Editable desde la pestaña
        # Rutinas (self._prog_stable_spin).
        self._prog_stable_window_s = 0.5

        # Destino (XYZ mm + nombre de variable si aplica) del último bloque de
        # movimiento lanzado por la rutina; usado por _prog_log_arrival() para
        # comparar contra dónde se detuvo realmente el robot.
        self._prog_target_xyz = None
        self._prog_target_var = None
        self._prog_current_vel_pct = 100.0  # % de velocidad del bloque move/move_ori en curso

        # Variables de posición nombradas { nombre: {x,y,z,traj} }
        self._position_vars = {}
        # Setpoints enviados (para gráfica PID)
        self._joint_setpoints = [0.0] * 6
        # Traza 3D de trayectoria
        self._traj_tracing = False
        self._traj_pts     = []
        self._traj_actor   = None
        self._preview_actor = None   # preview de rutina en 3D
        self._workspace_actor = None  # envolvente de trabajo 3D
        self._workspace_visible = False
        # Pila de subrutinas en ejecución (anti-recursión)
        self._prog_call_stack = set()
        # Log de errores
        self._log_widget = None   # QTextEdit, asignado en _build_log_tab
        # Historial de posiciones (últimas 20)
        self._pos_history = deque(maxlen=20)
        self._pos_hist_rows_layout = None  # asignado en _build_rutinas_tab

        # Animación ghost
        self._ghost_anim_q_start = None   # np.array de ángulos de inicio
        self._ghost_anim_q_end   = None   # np.array de ángulos objetivo
        self._ghost_anim_step    = 0
        self._ghost_anim_total   = 0
        self._ghost_anim_err_pos = 0.0
        self._ghost_anim_err_ori = 0.0
        self._ghost_timer        = QTimer(self)
        self._ghost_timer.timeout.connect(self._animate_ghost_step)

        # Signos visuales (calibrados para el 3D)
        self.rotation_signs_visual = [1, 1, -1, -1, 1, 1]
 
        # Signos MDH (calibrados con el panel de diagnóstico)
        self.rotation_signs_mdh = [1, 1, 1, 1, -1, 1]
 
        self.sliders               = []
        self.labels                = []
        self.real_angle_labels     = []
        self.real_angles_feedback  = [0.0] * 6   # ángulos reales recibidos del ESP32
        self._joint_offsets        = [0.0] * 6   # offsets de calibración (°)
        self._joint_raw_feedback   = [0.0] * 6   # lectura bruta antes de offset
        self.inputs                = []
        self.connection_indicators = []
        self.sign_buttons_mdh      = []
        self.enable_buttons        = []
        self._enable_states        = [False] * 6   # False = habilitado (motor activo)
        self._viz_real             = False
        self._free_move            = False
        self._dry_run_mode         = False   # modo simulacion: no envia nada al hardware
        self._teach_mode_active    = False   # modo Teach: capturar puntos a mano

        # Cinta transportadora (ESP32-C3 propio, ver central "cinta="/"cintadist=").
        # Estado recibido del central por Serial (ver _parse_cinta_estado);
        # cinta_objeto_detectado lo usa la condición "Pieza (cinta)" de las
        # rutinas (ver _eval_prog_cond). cinta_distancia_umbral_cm es lo
        # último que la propia app mandó con "cintadist=" (para mostrarlo en
        # la UI de configuración sin depender de que el hardware lo confirme).
        self.cinta_objeto_detectado    = False
        self.cinta_distancia_cm        = -1.0    # última distancia MEDIDA por el HC-SR04 (cm), -1 = sin lectura
        self.cinta_distancia_umbral_cm = 4.0      # umbral de detección configurado desde la app (cm)
        self._free_move_timer      = QTimer(self)
        self._free_move_timer.setInterval(150)
        self._free_move_timer.timeout.connect(self._free_move_tick)
        self._range_warning_active     = False
        self._range_warning_blink_timer = QTimer(self)
        self._range_warning_blink_timer.setInterval(400)
        self._range_warning_blink_timer.timeout.connect(self._range_warning_blink_tick)

        # Deshacer/Rehacer del editor de rutinas
        self._prog_undo_stack = []
        self._prog_redo_stack = []
        self._prog_undo_max   = 50

        # Perfiles de herramienta (TCP offset) -- carga el activo al iniciar
        self._tool_profiles       = {}
        self._active_tool_profile = 'Sin offset'
        self._load_tool_profiles()

 
        main_widget  = QWidget()
        self.setCentralWidget(main_widget)
        root_layout = QVBoxLayout(main_widget)
        root_layout.setSpacing(0)
        root_layout.setContentsMargins(4, 4, 4, 4)
 
        # ── Splitter vertical: arriba 3D+controles / abajo diagnóstico ───
        self._main_splitter = QSplitter(Qt.Vertical)
        self._main_splitter.setHandleWidth(7)
        self._main_splitter.setStyleSheet("""
            QSplitter::handle:vertical {
                background: qlineargradient(x1:0,y1:0,x2:0,y2:1,
                    stop:0 #23262b, stop:0.4 #f0a500, stop:0.6 #f0a500, stop:1 #23262b);
                border-radius: 3px; margin: 1px 40px;
            }
            QSplitter::handle:vertical:hover {
                background: qlineargradient(x1:0,y1:0,x2:0,y2:1,
                    stop:0 #23262b, stop:0.3 #ffd060, stop:0.7 #ffd060, stop:1 #23262b);
            }
        """)
        root_layout.addWidget(self._main_splitter)
 
        # ── Fila superior: 3D + controles (dentro del splitter) ──
        # Splitter horizontal: izquierda 3D / derecha Controles Manuales
        top_splitter = QSplitter(Qt.Horizontal)
        top_splitter.setHandleWidth(4)
        top_splitter.setStyleSheet("""
            QSplitter::handle {
                margin: 0px;
            }
            QSplitter::handle:horizontal {
                background: #2e3340;
                border-left:  1px solid #70e570;
                border-right: 1px solid #70e570;
                margin: 0px;
            }
            QSplitter::handle:horizontal:hover {
                background: #70e570;
                margin: 0px;
            }
        """)
        self._top_splitter = top_splitter

        left_frame  = QFrame()
        left_frame.setStyleSheet("QFrame { border: 2px solid #70e570; border-radius: 5px; }")
        left_layout = QVBoxLayout(left_frame)

        # Cabecera 3D con botón de colapsar panel derecho
        hdr_3d = QHBoxLayout()
        lbl_3d = QLabel("🖥️ VISUAL. 3D")
        lbl_3d.setStyleSheet("font-size:8px; font-weight:bold; color:#70e570; padding:1px;")
        lbl_3d.setAlignment(Qt.AlignCenter)
        self._ctrl_collapse_btn = QPushButton("▶")
        self._ctrl_collapse_btn.setToolTip("Plegar / desplegar Controles Manuales")
        self._ctrl_collapse_btn.setFixedSize(19, 14)
        self._ctrl_collapse_btn.setStyleSheet(
            "QPushButton { border:1px solid #70e570; color:#70e570; background:transparent;"
            " border-radius:4px; font-weight:bold; }"
            "QPushButton:hover { background:#70e570; color:#23262b; }"
        )
        self._ws_btn = QPushButton('🌐 Envolvente')
        self._ws_btn.setToolTip('Mostrar/ocultar envolvente de trabajo 3D')
        self._ws_btn.setCheckable(True)
        self._ws_btn.setStyleSheet(
            'QPushButton { border:1px solid #446688; color:#6699bb; background:transparent;'
            ' border-radius:4px; font-size:9px; padding:2px 6px; }'
            'QPushButton:checked { background:#224466; color:#88bbdd; border-color:#88bbdd; }'
            'QPushButton:hover { background:#1a3355; }')
        self._ws_btn.clicked.connect(self._toggle_workspace)

        self._viz_real_btn = QPushButton('👁 Vista App')
        self._viz_real_btn.setToolTip('Cambiar entre angulos de la app o reales del ESP32')
        self._viz_real_btn.setCheckable(True)
        self._viz_real_btn.setStyleSheet(
            'QPushButton { border:1px solid #886644; color:#bbaa77; background:transparent;'
            ' border-radius:4px; font-size:9px; padding:2px 6px; }'
            'QPushButton:checked { background:#443311; color:#ffcc66; border-color:#ffcc66; }'
            'QPushButton:hover { background:#332200; }')
        self._viz_real_btn.clicked.connect(self._toggle_viz_real)

        self._free_btn = QPushButton('🔓 Mover libre')
        self._free_btn.setToolTip('Deshabilita motores y sigue angulos reales en 3D')
        self._free_btn.setCheckable(True)
        self._free_btn.setStyleSheet(
            'QPushButton { border:1px solid #664488; color:#aa77cc; background:transparent;'
            ' border-radius:4px; font-size:9px; padding:2px 6px; }'
            'QPushButton:checked { background:#331144; color:#dd99ff; border-color:#dd99ff; }'
            'QPushButton:hover { background:#220033; }')
        self._free_btn.clicked.connect(self._toggle_free_move)

        self._tcp_btn = QPushButton('TCP')
        self._tcp_btn.setToolTip('Perfiles de herramienta (offset TCP: pinza, ventosa, etc.)')
        self._tcp_btn.setStyleSheet(
            'QPushButton { border:1px solid #26C6DA; color:#26C6DA; background:transparent;'
            ' border-radius:4px; font-size:9px; padding:2px 6px; }'
            'QPushButton:hover { background:#0d2a3d; }')
        self._tcp_btn.clicked.connect(self._open_tool_profile_dialog)

        self._reset_view_btn = QPushButton('🎥')
        self._reset_view_btn.setToolTip('Recuperar la vista de cámara con la que arranca la app')
        self._reset_view_btn.setStyleSheet(
            'QPushButton { border:1px solid #888888; color:#cccccc; background:transparent;'
            ' border-radius:4px; font-size:9px; padding:2px 6px; }'
            'QPushButton:hover { background:#333333; }')
        self._reset_view_btn.clicked.connect(self._reset_camera_view)

        # Botón de cierre visible: esta ventana arranca en modo kiosco (sin
        # bordes ni barra de título, ver main()) para la pantalla táctil de
        # 800x480, así que no hay "X" nativa — hace falta un botón propio.
        self._close_btn = QPushButton('✕')
        self._close_btn.setToolTip('Cerrar la aplicación')
        self._close_btn.setFixedSize(20, 16)
        self._close_btn.setStyleSheet(
            'QPushButton { border:1px solid #aa4444; color:#ff8888; background:transparent;'
            ' border-radius:4px; font-weight:bold; font-size:10px; }'
            'QPushButton:hover { background:#aa2222; color:white; }')
        self._close_btn.clicked.connect(self.close)

        hdr_3d.addWidget(lbl_3d, 1)
        hdr_3d.addWidget(self._viz_real_btn)
        hdr_3d.addWidget(self._free_btn)
        hdr_3d.addWidget(self._tcp_btn)
        hdr_3d.addWidget(self._ws_btn)
        hdr_3d.addWidget(self._reset_view_btn)
        hdr_3d.addWidget(self._ctrl_collapse_btn)
        hdr_3d.addWidget(self._close_btn)
        left_layout.addLayout(hdr_3d)

        self._range_warning_lbl = QLabel('')
        self._range_warning_lbl.setAlignment(Qt.AlignCenter)
        self._range_warning_lbl.setStyleSheet(
            'background:#cc2222; color:white; font-weight:bold; font-size:10px;'
            ' padding:4px; border-radius:3px;')
        self._range_warning_lbl.setVisible(False)
        left_layout.addWidget(self._range_warning_lbl)

        self.plotter_widget = QtInteractor()
        self.plotter_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        left_layout.addWidget(self.plotter_widget)

        right_frame  = QFrame()
        right_frame.setStyleSheet("QFrame { border: 2px solid #70e570; border-radius: 5px; }")
        right_layout = QVBoxLayout(right_frame)
        lbl_ctrl = QLabel("🎮 CONTROLES MANUALES")
        lbl_ctrl.setStyleSheet("font-size:9px; font-weight:bold; color:#70e570; padding:2px;")
        lbl_ctrl.setAlignment(Qt.AlignCenter)

        # Botón para ocultar/mostrar la vista 3D (simétrico al que oculta
        # este mismo panel desde la cabecera de la vista 3D) — en una
        # pantalla de 800x480 a veces interesa dar todo el ancho a los
        # controles y prescindir del render 3D.
        self._viz3d_collapse_btn = QPushButton("◀")
        self._viz3d_collapse_btn.setToolTip("Ocultar visualización 3D")
        self._viz3d_collapse_btn.setFixedSize(19, 14)
        self._viz3d_collapse_btn.setStyleSheet(
            "QPushButton { border:1px solid #70e570; color:#70e570; background:transparent;"
            " border-radius:4px; font-weight:bold; }"
            "QPushButton:hover { background:#70e570; color:#23262b; }"
        )
        hdr_ctrl = QHBoxLayout()
        hdr_ctrl.addWidget(self._viz3d_collapse_btn)
        hdr_ctrl.addWidget(lbl_ctrl, 1)
        right_layout.addLayout(hdr_ctrl)
        self._ctrl_frame = right_frame

        right_layout.addWidget(self._build_serial_panel())

        scroll_area   = QScrollArea(); scroll_area.setWidgetResizable(True)
        scroll_widget = QWidget()
        self.controls_layout = QVBoxLayout(scroll_widget)
        scroll_area.setWidget(scroll_widget)
        scroll_area.setStyleSheet("""
            QScrollArea { border:none; background:transparent; }
            QScrollBar:vertical { background:#23262b; width:14px; margin:16px 3px; border-radius:7px; }
            QScrollBar::handle:vertical { background:#70e570; border-radius:7px; min-height:20px; }
        """)
        right_layout.addWidget(scroll_area)

        # Lógica del botón colapsar / expandir
        self._ctrl_prev_sizes = [190, 590]
        def _toggle_ctrl_panel():
            if self._ctrl_frame.isVisible():
                self._ctrl_prev_sizes = self._top_splitter.sizes()
                self._ctrl_frame.setVisible(False)
                self._ctrl_collapse_btn.setText("◀")
                self._ctrl_collapse_btn.setToolTip("Mostrar Controles Manuales")
            else:
                self._ctrl_frame.setVisible(True)
                self._top_splitter.setSizes(self._ctrl_prev_sizes)
                self._ctrl_collapse_btn.setText("▶")
                self._ctrl_collapse_btn.setToolTip("Plegar Controles Manuales")
        self._ctrl_collapse_btn.clicked.connect(_toggle_ctrl_panel)

        # Lógica del botón ocultar / mostrar la vista 3D (mismo patrón que
        # el de arriba, pero sobre left_frame en vez de _ctrl_frame).
        self._viz3d_prev_sizes = [190, 590]
        def _toggle_viz3d_panel():
            if left_frame.isVisible():
                self._viz3d_prev_sizes = self._top_splitter.sizes()
                left_frame.setVisible(False)
                self._viz3d_collapse_btn.setText("▶")
                self._viz3d_collapse_btn.setToolTip("Mostrar visualización 3D")
            else:
                left_frame.setVisible(True)
                self._top_splitter.setSizes(self._viz3d_prev_sizes)
                self._viz3d_collapse_btn.setText("◀")
                self._viz3d_collapse_btn.setToolTip("Ocultar visualización 3D")
        self._viz3d_collapse_btn.clicked.connect(_toggle_viz3d_panel)

        top_splitter.addWidget(left_frame)
        top_splitter.addWidget(right_frame)
        # En la pantalla de 800x480 el panel de CONTROLES MANUALES (fila de
        # puerto serie, botones, etc.) necesita más ancho que la vista 3D
        # para no requerir scroll horizontal — al revés que en el monitor
        # de PC, donde la vista 3D es la protagonista.
        top_splitter.setSizes([190, 590])
        top_splitter.setStretchFactor(0, 1)
        top_splitter.setStretchFactor(1, 3)

        # ── Fila inferior: panel de diagnóstico ──────────
        self.diag_frame = self._build_diagnostic_panel()
        self._main_splitter.addWidget(top_splitter)
        self._main_splitter.addWidget(self.diag_frame)
        self._main_splitter.setSizes([650, 200])
        self.setup_manual_controls()
        self.setup_3d_visualization()
        self.update_xyz_display()
        self.update_3d_visualization()
        self._load_joint_offsets()   # cargar offsets de calibración si existen
 
    # ─────────────────────────────────────────
    #  Panel de diagnóstico / calibración
    # ─────────────────────────────────────────
 
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
            q = np.deg2rad(q_deg)
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


    def _build_serial_panel(self):
        frame = QFrame()
        frame.setStyleSheet(
            "QFrame { border: 2px solid #4488ff; border-radius: 5px; }"
        )
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(4, 2, 4, 2)
        layout.setSpacing(2)

        title = QLabel("COMUNICACION SERIE — ESP32")
        title.setStyleSheet(
            "font-size:10px; font-weight:bold; color:#4488ff; padding:2px;"
        )
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        row = QHBoxLayout()
        row.setSpacing(3)

        self.serial_led = QLabel()
        self.serial_led.setFixedSize(8, 8)
        self.serial_led.setStyleSheet("background:#ff6b6b; border-radius:7px;")
        row.addWidget(self.serial_led)

        self.serial_status_lbl = QLabel("Desconectado")
        self.serial_status_lbl.setStyleSheet("color:#ff6b6b; font-weight:bold;")
        row.addWidget(self.serial_status_lbl)

        row.addSpacing(10)

        self.com_combo = QComboBox()
        self.com_combo.setFixedWidth(66)
        row.addWidget(self.com_combo)

        self.baud_combo = QComboBox()
        for br in ["9600", "19200", "38400", "57600", "115200", "230400"]:
            self.baud_combo.addItem(br)
        self.baud_combo.setCurrentText("115200")
        self.baud_combo.setFixedWidth(51)
        row.addWidget(self.baud_combo)

        btn_scan = QPushButton("Escanear")
        btn_scan.setStyleSheet("border-color:#4488ff; color:#4488ff;")
        btn_scan.clicked.connect(self.scan_com_ports)
        row.addWidget(btn_scan)

        self.btn_connect = QPushButton("Conectar")
        self.btn_connect.setStyleSheet("border-color:#4488ff; color:#4488ff;")
        self.btn_connect.clicked.connect(self.toggle_serial)
        row.addWidget(self.btn_connect)

        btn_cfg = QPushButton("Configurar nodos")
        btn_cfg.setStyleSheet("border-color:#ff9900; color:#ff9900;")
        btn_cfg.clicked.connect(self.open_node_config)
        row.addWidget(btn_cfg)

        row.addStretch()
        layout.addLayout(row)

        # ── Fila 2: Backup / Restaurar / Calibrar ────────────────────
        row2 = QHBoxLayout()
        row2.setSpacing(3)
        btn_backup = QPushButton('💾 Backup')
        btn_backup.setToolTip('Guardar configuración completa')
        btn_backup.setStyleSheet('border-color:#44cc88; color:#44cc88;')
        btn_backup.clicked.connect(self._backup_config)
        row2.addWidget(btn_backup)

        btn_restore = QPushButton('📂 Restaurar')
        btn_restore.setToolTip('Restaurar configuración desde backup')
        btn_restore.setStyleSheet('border-color:#44cc88; color:#44cc88;')
        btn_restore.clicked.connect(self._restore_config)
        row2.addWidget(btn_restore)

        btn_cal = QPushButton('🔧 Calibrar')
        btn_cal.setToolTip('Asistente de calibración de offsets por articulación')
        btn_cal.setStyleSheet('border-color:#cc8844; color:#cc8844;')
        btn_cal.clicked.connect(self._open_calibration_wizard)
        row2.addWidget(btn_cal)

        btn_rst = QPushButton("Reset todos a 0°")
        btn_rst.setStyleSheet("border-color:#f0a500; color:#f0a500;")
        btn_rst.clicked.connect(self.reset_all)
        row2.addWidget(btn_rst)

        btn_cinta = QPushButton('📦 Cinta')
        btn_cinta.setToolTip('Configurar distancia de detección de la cinta transportadora')
        btn_cinta.setStyleSheet('border-color:#00b894; color:#00b894;')
        btn_cinta.clicked.connect(self.open_cinta_config)
        row2.addWidget(btn_cinta)

        row2.addStretch()
        layout.addLayout(row2)

        self.scan_com_ports()
        return frame

    def open_node_config(self):
        dlg = NodeConfigDialog(self)
        dlg.exec_()

    def open_cinta_config(self):
        dlg = CintaConfigDialog(self)
        dlg.exec_()

    def scan_com_ports(self):
        self.com_combo.clear()
        if not SERIAL_AVAILABLE:
            self.com_combo.addItem("pyserial no instalado")
            return
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.com_combo.addItem(p.device)
        if self.com_combo.count() == 0:
            self.com_combo.addItem("Sin puertos")

    def toggle_serial(self):
        if self.serial_port and self.serial_port.is_open:
            self._disconnect_serial()
        else:
            self._connect_serial()

    def _connect_serial(self):
        if not SERIAL_AVAILABLE:
            QMessageBox.warning(
                self, "pyserial no instalado",
                "Instala pyserial:\n  pip install pyserial"
            )
            return
        port = self.com_combo.currentText()
        if not port or port in ("Sin puertos", "pyserial no instalado"):
            QMessageBox.warning(self, "Puerto invalido", "Selecciona un puerto COM valido.")
            return
        baud = int(self.baud_combo.currentText())
        try:
            self.serial_port = serial.Serial(port, baud, timeout=1)
            self._set_serial_status(True, port)
        except Exception as e:
            QMessageBox.critical(self, "Error al conectar", str(e))
            self._disconnect_serial()

    def _disconnect_serial(self):
        self._log_error("Conexión serial cerrada / perdida", "SERIAL")
        if self.serial_port:
            try:
                self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None
        self._set_serial_status(False)

    def _set_serial_status(self, connected, port=""):
        if connected:
            self.serial_led.setStyleSheet("background:#70e570; border-radius:7px;")
            self.serial_status_lbl.setStyleSheet("color:#70e570; font-weight:bold;")
            self.serial_status_lbl.setText(f"Conectado — {port}")
            self.btn_connect.setText("Desconectar")
        else:
            self.serial_led.setStyleSheet("background:#ff6b6b; border-radius:7px;")
            self.serial_status_lbl.setStyleSheet("color:#ff6b6b; font-weight:bold;")
            self.serial_status_lbl.setText("Desconectado")
            self.btn_connect.setText("Conectar")

    def closeEvent(self, event):
        self._disconnect_serial()
        super().closeEvent(event)

    def keyPressEvent(self, event):
        # Esta ventana arranca en modo kiosco (sin bordes ni barra de
        # título, ver main()) para la pantalla táctil de 800x480, así que
        # no hay botón "X" para cerrarla. Con un teclado conectado (o por
        # SSH con xdotool): Esc cierra la app, F11 alterna pantalla
        # completa / ventana normal para depurar en el propio panel táctil.
        if event.key() == Qt.Key_Escape:
            self.close()
        elif event.key() == Qt.Key_F11:
            if self.isFullScreen():
                self.showNormal()
            else:
                self.showFullScreen()
        else:
            super().keyPressEvent(event)

    # ─────────────────────────────────────────
    #  Envio de setpoints al ESP32
    # ─────────────────────────────────────────

    def send_setpoints(self, angles, vel_pct=100.0):
        self._joint_setpoints = [a % 360 for a in angles]  # para gráfica PID (0-360, espacio calibrado)
        if getattr(self, '_dry_run_mode', False):
            return  # modo simulación: no enviar nada al hardware
        if not (self.serial_port and self.serial_port.is_open):
            return
        # "angles" (self.angles) vive en espacio CALIBRADO: cinemática, IK y
        # pantalla usan angulo_calibrado = angulo_raw - _joint_offsets[i] (ver
        # _parse_angulos). El firmware, en cambio, solo conoce su propio
        # angulo_raw (lo que reporta como "angulo[i]=..." y contra lo que
        # compara el setpoint). Si aquí se manda el valor calibrado tal cual,
        # el brazo se para sistemáticamente _joint_offsets[i]° desplazado del
        # objetivo que se ve en pantalla — justo el motivo de que "no se
        # acerque del todo" a las coordenadas indicadas en cuanto hay una
        # calibración (offsets != 0) guardada. Se compensa sumando el offset
        # de vuelta antes de enviar (con offsets todos a 0 no cambia nada).
        # Formato: SP[1]=10.00;SP[2]=20.00;...SP[6]=60.00;V=75;\n
        # "V=NN" es el % de velocidad (10-100) para este movimiento — el
        # central lo reenvía a las 6 articulaciones junto al setpoint, y
        # cada una escala su perfil de velocidad por ese factor. Se manda
        # siempre explícito (incluso a 100) para no depender de que el
        # central conserve el valor de un envío anterior.
        parts = [f"SP[{i+1}]={(angles[i] + self._joint_offsets[i]) % 360:.2f}" for i in range(6)]
        vel_pct = max(10.0, min(100.0, float(vel_pct)))
        parts.append(f"V={vel_pct:.0f}")
        cmd = ";".join(parts) + ";\n"
        try:
            self.serial_port.write(cmd.encode("ascii"))
            print(cmd)
        except Exception as e:
            self._log_error(f"Serial send_setpoints: {e}", "SERIAL")
            self._disconnect_serial()

    def _read_serial_angles(self):
        """Lee líneas pendientes del ESP32 y actualiza las etiquetas de ángulo real."""
        if not (self.serial_port and self.serial_port.is_open):
            return
        try:
            while self.serial_port.in_waiting:
                raw = self.serial_port.readline()
                line = raw.decode("ascii", errors="ignore").strip()
                if not line:
                    continue
                # Formato: angulo[1]=10.50;angulo[2]=20.30;...angulo[6]=60.10;
                # Formato: spmando[1]=45.00;  (setpoint enviado por el mando fisico)
                print(f"[Serial RX] {line}")
                self._parse_angulos(line)
                self._parse_sp_mando(line)
                self._parse_cinta_estado(line)
        except Exception as e:
            print(f"[Serial RX] Error: {e}")
            self._disconnect_serial()

    def _parse_angulos(self, line):
        """Parsea 'angulo[i]=val;' y actualiza real_angle_labels (i es 1-based)."""
        import re
        for m in re.finditer(r"angulo\[(\d+)\]=([-\d.]+)", line):
            idx = int(m.group(1)) - 1   # 0-based
            if 0 <= idx < 6:
                val = float(m.group(2))
                self._joint_raw_feedback[idx] = val
                val_cal = val - self._joint_offsets[idx]
                self.real_angle_labels[idx].setText(f"Real: {val_cal:.2f}°")
                self.real_angles_feedback[idx] = val_cal   # para el motor de rutinas

    def _parse_sp_mando(self, line):
        """Parsea 'spmando[i]=val;' — setpoint que el mando fisico acaba de
        mandar directamente al central (sin pasar por la app). El central lo
        reenvia al PC en cuanto llega. Sin esto, la app solo se enteraba del
        ANGULO REAL (via _parse_angulos, cada ~100 ms), pero self.angles
        (el objetivo que usa "Vista App"/la cinemática por defecto) se quedaba
        con el valor viejo — parecía que el mando "no llegaba" a la app.
        Sincroniza self.angles + sliders/inputs/labels + la vista 3D, sin
        volver a enviar el setpoint (ya está aplicado en el hardware)."""
        import re
        actualizado = False
        for m in re.finditer(r"spmando\[(\d+)\]=([-\d.]+)", line):
            idx = int(m.group(1)) - 1   # 0-based
            if 0 <= idx < 6:
                val = _wrap360(float(m.group(2)))
                self.angles[idx] = val
                self.sliders[idx].blockSignals(True)
                self.sliders[idx].setValue(int(val))
                self.sliders[idx].blockSignals(False)
                self.inputs[idx].setText(f"{val:.2f}")
                self.labels[idx].setText(f"Articulación {idx+1}: {val:.2f}°")
                # También el setpoint usado por la gráfica de ajuste PID
                # (Setpoint vs Feedback), que si no, se quedaba con el último
                # valor mandado desde la propia app y no reflejaba lo que
                # acaba de marcar el mando físico.
                if idx < len(self._joint_setpoints):
                    self._joint_setpoints[idx] = val
                actualizado = True
        if actualizado:
            self.update_xyz_display()
            self.update_3d_visualization()

    def _parse_cinta_estado(self, line):
        """Parsea 'objetoCinta=0|1;' y 'distCinta=val;' — estado de la cinta
        transportadora que el central reenvía desde la propia cinta (ver
        struct_message_cinta_estado en ambos firmwares). objeto_detectado lo
        usa la condición "Pieza (cinta)" de las rutinas (_eval_prog_cond)."""
        import re
        m = re.search(r"objetoCinta=([01]);", line)
        if m:
            self.cinta_objeto_detectado = (m.group(1) == '1')
        m = re.search(r"distCinta=(-?[\d.]+);", line)
        if m:
            try:
                self.cinta_distancia_cm = float(m.group(1))
            except ValueError:
                pass

    def send_reset(self, index):
        """Envía reset[i]=0.00 al ESP32 (index 0-based, igual que el firmware)."""
        if not (self.serial_port and self.serial_port.is_open):
            return
        cmd = f"reset[{index}]=0.00\n"
        try:
            self.serial_port.write(cmd.encode("ascii"))
        except Exception as e:
            print(f"[Serial] Error al enviar reset: {e}")
            self._disconnect_serial()

    def send_cinta_cmd(self, arranque, vel_pct=None):
        """Arranca/para la cinta transportadora: 'cinta=1;vel=NN' o 'cinta=0'.
        vel_pct None al parar (el central conserva la última velocidad
        conocida; no hace falta reenviarla)."""
        if self._dry_run_mode or not (self.serial_port and self.serial_port.is_open):
            return
        if arranque:
            v = max(0.0, min(100.0, float(vel_pct if vel_pct is not None else 50)))
            cmd = f"cinta=1;vel={v:.0f}\n"
        else:
            cmd = "cinta=0\n"
        try:
            self.serial_port.write(cmd.encode("ascii"))
        except Exception as e:
            print(f"[Serial] Error al enviar comando de cinta: {e}")
            self._disconnect_serial()

    def send_cinta_distancia(self, distancia_cm):
        """Cambia la distancia umbral de detección de la cinta ('cintadist=N.N'),
        reenviada por el central al ESP32-C3 de la cinta."""
        self.cinta_distancia_umbral_cm = float(distancia_cm)
        if self._dry_run_mode or not (self.serial_port and self.serial_port.is_open):
            return
        cmd = f"cintadist={float(distancia_cm):.1f}\n"
        try:
            self.serial_port.write(cmd.encode("ascii"))
        except Exception as e:
            print(f"[Serial] Error al enviar distancia de la cinta: {e}")
            self._disconnect_serial()

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

    def dark_theme(self):
        return """
        QWidget { background:#23262b; color:#E0E0E0;
                  font-family:'Segoe UI',Arial; font-size:10px; }
        QPushButton { background-color:#31364a; border:1.2px solid #5dd095;
                      border-radius:8px; padding:5px 10px;
                      color:#70e570; font-weight:bold; }
        QPushButton:hover { background-color:#70e570; color:#23262b; }
        QLineEdit, QComboBox, QTextEdit { background:#2b2e38; border:1.2px solid #5dd095;
                      border-radius:5px; color:#e0e0e0; padding:4px; }
        QLabel { color:#93ffa8; font-weight:600; }
        QSlider::groove:horizontal { height:8px; background:#393e4c; border-radius:4px; }
        QSlider::handle:horizontal { background:#70e570; border-radius:8px;
                      height:16px; width:16px; margin:-4px 0; }
        QTextEdit { background:#181c1f; color:#97fa97;
                    font-family:'Consolas',monospace; border-radius:6px; }
        """


# ─────────────────────────────────────────────
#  Splash + main
# ─────────────────────────────────────────────

class LoadingSignals(QObject):
    progress_updated = pyqtSignal(int)
    message_updated  = pyqtSignal(str)


class CalibrationWizard(QDialog):
    """Asistente paso a paso para calibrar el offset de cada articulación."""

    def __init__(self, robot):
        super().__init__(robot)
        self.robot = robot
        self.step  = 0          # 0-5 = joints, 6 = summary
        self.setWindowTitle('🔧 Asistente de Calibración')
        self.setMinimumWidth(288)
        self.setStyleSheet(robot.dark_theme())
        self._pending_offsets = list(robot._joint_offsets)
        self._build_ui()
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh_live)
        self._timer.start(150)
        self._refresh_step()

    def _build_ui(self):
        vlay = QVBoxLayout(self)
        vlay.setSpacing(6)

        # Progress bar
        from PyQt5.QtWidgets import QProgressBar
        self._progress = QProgressBar()
        self._progress.setRange(0, 6)
        self._progress.setValue(0)
        self._progress.setTextVisible(True)
        self._progress.setStyleSheet(
            'QProgressBar { border:1px solid #446644; border-radius:4px; background:#0d1a0d; color:#70e570; }'
            'QProgressBar::chunk { background:#336633; border-radius:3px; }')
        vlay.addWidget(self._progress)

        # Step title
        self._title_lbl = QLabel()
        self._title_lbl.setStyleSheet('font-size:11px; font-weight:bold; color:#88dd88;')
        vlay.addWidget(self._title_lbl)

        # Instructions
        self._instr_lbl = QLabel()
        self._instr_lbl.setWordWrap(True)
        self._instr_lbl.setStyleSheet('color:#aaccaa; font-size:9px;')
        vlay.addWidget(self._instr_lbl)

        # Live angle display
        self._live_lbl = QLabel('─')
        self._live_lbl.setStyleSheet(
            'font-size:17px; font-weight:bold; color:#ffcc44;'
            ' background:#0d1a0d; border:1px solid #336633;'
            ' border-radius:4px; padding:6px 12px;')
        self._live_lbl.setAlignment(Qt.AlignCenter)
        vlay.addWidget(self._live_lbl)

        # Current offset display
        self._offset_lbl = QLabel()
        self._offset_lbl.setStyleSheet('color:#556655; font-size:9px;')
        self._offset_lbl.setAlignment(Qt.AlignCenter)
        vlay.addWidget(self._offset_lbl)

        # Set zero button
        self._set_zero_btn = QPushButton('📍 Fijar como 0° (usar lectura actual)')
        self._set_zero_btn.setStyleSheet(
            'QPushButton { border:2px solid #44aa44; color:#44ff44;'
            ' background:#0d1a0d; border-radius:6px; padding:6px; font-weight:bold; }'
            'QPushButton:hover { background:#1a3a1a; }')
        self._set_zero_btn.clicked.connect(self._set_zero)
        vlay.addWidget(self._set_zero_btn)

        # Reset offset button
        btn_reset = QPushButton('↺ Resetear offset a 0')
        btn_reset.setStyleSheet(
            'QPushButton { border:1px solid #446644; color:#669966;'
            ' background:transparent; border-radius:4px; padding:4px; }'
            'QPushButton:hover { background:#1a2a1a; }')
        btn_reset.clicked.connect(self._reset_offset)
        vlay.addWidget(btn_reset)

        # Navigation
        nav = QHBoxLayout()
        self._prev_btn = QPushButton('◀ Anterior')
        self._prev_btn.clicked.connect(self._prev_step)
        self._next_btn = QPushButton('Siguiente ▶')
        self._next_btn.setStyleSheet(
            'QPushButton { border:1px solid #44aa44; color:#44cc44;'
            ' background:#0d1a0d; border-radius:5px; padding:4px 10px; }')
        self._next_btn.clicked.connect(self._next_step)
        self._save_btn = QPushButton('💾 Guardar y cerrar')
        self._save_btn.setStyleSheet(
            'QPushButton { border:2px solid #44aa44; color:#44ff44;'
            ' background:#0d1a0d; border-radius:5px; padding:4px 10px; font-weight:bold; }')
        self._save_btn.clicked.connect(self._save_and_close)
        self._save_btn.setVisible(False)
        nav.addWidget(self._prev_btn)
        nav.addStretch()
        nav.addWidget(self._next_btn)
        nav.addWidget(self._save_btn)
        vlay.addLayout(nav)

    def _refresh_step(self):
        self._progress.setValue(self.step)
        is_summary = (self.step == 6)
        self._set_zero_btn.setVisible(not is_summary)
        self._live_lbl.setVisible(not is_summary)
        self._next_btn.setVisible(not is_summary)
        self._save_btn.setVisible(is_summary)
        self._prev_btn.setEnabled(self.step > 0)

        if is_summary:
            self._title_lbl.setText('✅ Resumen de calibración')
            lines = []
            for i, off in enumerate(self._pending_offsets):
                lines.append(f'  J{i+1}:  offset = {off:+.2f}°')
            self._instr_lbl.setText(
                'Offsets a aplicar:\n' + '\n'.join(lines) +
                '\n\nPulsa "Guardar y cerrar" para aplicar y persistir.')
            self._offset_lbl.setText('')
        else:
            j = self.step + 1
            self._title_lbl.setText(f'Articulación {j}  (J{j})')
            self._instr_lbl.setText(
                f'1. Mueve físicamente J{j} a su posición de 0° (home mecánico).\n'
                f'2. Brazo conectado por serial para ver la lectura.\n'
                f'3. Pulsa "Fijar como 0" cuando el encoder esté en posición.')
            self._offset_lbl.setText(
                f'Offset actual J{j}: {self._pending_offsets[self.step]:+.2f}°')

    def _refresh_live(self):
        if self.step >= 6:
            return
        raw = self.robot._joint_raw_feedback[self.step]
        cal = raw - self._pending_offsets[self.step]
        self._live_lbl.setText(
            f'J{self.step+1}  raw: {raw:.2f}°   →   calibrado: {cal:.2f}°')

    def _set_zero(self):
        if self.step >= 6:
            return
        raw = self.robot._joint_raw_feedback[self.step]
        self._pending_offsets[self.step] = raw
        self._offset_lbl.setText(
            f'Offset J{self.step+1} fijado: {raw:+.2f}°  ✓')
        self._offset_lbl.setStyleSheet('color:#44ff44; font-size:9px;')

    def _reset_offset(self):
        if self.step >= 6:
            return
        self._pending_offsets[self.step] = 0.0
        self._offset_lbl.setText(f'Offset J{self.step+1}: 0.00° (reseteado)')
        self._offset_lbl.setStyleSheet('color:#556655; font-size:9px;')

    def _prev_step(self):
        if self.step > 0:
            self.step -= 1
            self._refresh_step()

    def _next_step(self):
        if self.step < 6:
            self.step += 1
            self._refresh_step()

    def _save_and_close(self):
        self._timer.stop()
        for i, off in enumerate(self._pending_offsets):
            self.robot._joint_offsets[i] = off
        self.robot._save_joint_offsets()
        self.accept()

    def closeEvent(self, event):
        self._timer.stop()
        super().closeEvent(event)



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
