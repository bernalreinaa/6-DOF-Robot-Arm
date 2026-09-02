# -*- coding: utf-8 -*-
"""
Cinemática directa/inversa (convención MDH), utilidades matriciales y zonas
articulares prohibidas. Ningún nombre de aquí depende de PyQt: es el módulo
"puro" de matemáticas del robot, reutilizable fuera de la GUI (por ejemplo
en el propio apartado 3.5 de la memoria, que reproduce estos cálculos).
"""
import os
import json
import numpy as np
import vtk

from app_paths import BASE_DIR

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
