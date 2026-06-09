"""
Utilidades de procesamiento de LaserScan para navegacion reactiva (solo NumPy).

CONVENCION-AGNOSTICO: la seleccion de sectores se hace por el ANGULO REAL de
cada beam (normalizado a [-pi, pi]), NO por indices calculados con
round((ang - angle_min)/inc). Asi funciona sin importar si el LIDAR publica:
  - angle_min en -pi..pi  o  0..2pi
  - angle_increment positivo o negativo
Esto elimina los errores de indexado/clamp que hacian que el sector "derecho"
colapsara al frente cuando el scan no era -pi..pi.

Convencion del robot (REP-103): angulo 0 = adelante, + = izquierda (CCW),
- = derecha (CW).
"""

import math
import numpy as np


def _norm(a):
    """Normaliza angulo(s) a [-pi, pi]. Acepta escalar o np.ndarray."""
    return np.arctan2(np.sin(a), np.cos(a))


def clean_ranges(ranges, range_min, range_max):
    """Sustituye lecturas invalidas por range_max (= 'libre', sin obstaculo).

    Invalidas: NaN, +/-inf, 0.0 y cualquier valor < range_min. El RPLidar
    reporta 0.0 cuando no hay eco; tratarlo como obstaculo a 0 m causaba
    frenados/transiciones espurias. Aqui se trata como 'sin obstaculo'.
    """
    r = np.asarray(ranges, dtype=np.float32)
    if not (math.isfinite(range_max) and range_max > 0.0):
        finite = r[np.isfinite(r)]
        range_max = float(finite.max()) if finite.size else 10.0
    rmin = float(range_min) if math.isfinite(range_min) and range_min > 0.0 else 0.0
    bad = ~np.isfinite(r) | (r <= 0.0) | (r < rmin)
    return np.where(bad, np.float32(range_max), r)


def beam_angles(angle_min, angle_increment, n, yaw_offset=0.0):
    """Angulo (normalizado a [-pi, pi]) de cada uno de los n beams.

    yaw_offset compensa el montaje del LIDAR. Si el cero fisico del lidar NO
    apunta al frente del robot, mide a que angulo (en el frame del lidar)
    aparece un objeto que esta justo ENFRENTE y pasalo como yaw_offset: se
    resta para que 'adelante' vuelva a ser el angulo 0. Default 0.0 (cero del
    lidar = frente, como en el Puzzlebot con rpy 0 0 0).
    """
    idx = np.arange(int(n), dtype=np.float64)
    return _norm(angle_min + idx * angle_increment - yaw_offset)


def sector_min(ranges, angles, lo, hi):
    """Minimo de los ranges cuyo angulo (normalizado) cae en [lo, hi] rad.

    Maneja wrap alrededor de +/-pi: si tras normalizar lo > hi, el sector cruza
    pi (p.ej. lo=2.9, hi=-2.9 cubre la parte trasera). Devuelve inf si no hay
    ningun beam en el sector.
    """
    if ranges is None or angles is None:
        return float('inf')
    r = np.asarray(ranges)
    a = np.asarray(angles)
    if r.size == 0 or a.size != r.size:
        return float('inf')
    lo = float(_norm(lo))
    hi = float(_norm(hi))
    if lo <= hi:
        mask = (a >= lo) & (a <= hi)
    else:
        mask = (a >= lo) | (a <= hi)   # el sector cruza +/-pi
    if not np.any(mask):
        return float('inf')
    return float(np.min(r[mask]))
