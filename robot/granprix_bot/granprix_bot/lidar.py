"""Traduccion de ``/scan`` (LaserScan, LiDAR MS200) a distancia minima
por zona angular (frente/derecha/izquierda/atras) en el marco del
robot. Calibracion tomada de ``CALIBRACION_LIDAR_VISION.md``
(``front_offset_deg``, ``invert_left_right``, ventanas por zona).

No usa mensaje custom (a diferencia de ``LidarZones.msg`` del proyecto
del robot) para que este paquete sea un solo ``ament_python`` sin
dependencias de interfaces personalizadas -- mas simple de compilar.
Convencion de angulo en el marco del robot: 0=frente, +90=izquierda,
-90=derecha (igual que la doc).
"""

import math

from .geometry_utils import normalize_angle
from .robot_state import GIRO_DERECHA, GIRO_IZQUIERDA, GIRO_180

INF = math.inf


def _en_ventana(angulo_deg: float, ventana_deg) -> bool:
    """True si angulo_deg cae dentro de [min, max] (con wraparound
    correcto, ej. ventana [165, 195] cruzando 180 funciona bien)."""
    mn, mx = ventana_deg
    centro = (mn + mx) / 2.0
    ancho = mx - mn
    diff = (angulo_deg - centro + 180.0) % 360.0 - 180.0
    return abs(diff) <= ancho / 2.0


def calcular_zonas(scan, front_offset_deg: float, invert_left_right: bool,
                    ventanas_deg: dict, max_range_m: float) -> dict:
    """``scan``: sensor_msgs/LaserScan (o None). ``ventanas_deg``: dict
    nombre_zona -> [min_deg, max_deg]. Devuelve dict nombre_zona ->
    distancia minima en metros (``math.inf`` si no hay lectura valida
    en esa zona)."""
    zonas = {nombre: INF for nombre in ventanas_deg}
    if scan is None or not scan.ranges:
        return zonas

    offset_rad = math.radians(front_offset_deg)
    rango_max = min(scan.range_max, max_range_m)

    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r <= scan.range_min or r > rango_max:
            continue
        raw_angle = scan.angle_min + i * scan.angle_increment
        angulo = normalize_angle(raw_angle - offset_rad)
        if invert_left_right:
            angulo = -angulo
        angulo_deg = math.degrees(angulo)

        for nombre, ventana in ventanas_deg.items():
            if _en_ventana(angulo_deg, ventana) and r < zonas[nombre]:
                zonas[nombre] = r

    return zonas


def puntos_robot(scan, front_offset_deg: float, invert_left_right: bool,
                  rango_max_m: float, max_puntos: int = 360) -> list:
    """Devuelve una lista de ``[x, y]`` (metros, marco del robot: x=frente,
    y=izquierda) para dibujar el barrido crudo del LiDAR -- usado solo
    por ``web_dashboard_node`` (radar en vivo), mismo offset/inversion
    que ``calcular_zonas`` para que lo dibujado coincida con las zonas
    que usan explorer/speedrun. Decimado a ``max_puntos`` como mucho."""
    if scan is None or not scan.ranges:
        return []

    n = len(scan.ranges)
    paso = max(1, n // max_puntos)
    offset_rad = math.radians(front_offset_deg)
    rango_max = min(scan.range_max, rango_max_m)

    puntos = []
    for i in range(0, n, paso):
        r = scan.ranges[i]
        if not math.isfinite(r) or r <= scan.range_min or r > rango_max:
            continue
        raw_angle = scan.angle_min + i * scan.angle_increment
        angulo = normalize_angle(raw_angle - offset_rad)
        if invert_left_right:
            angulo = -angulo
        puntos.append([round(r * math.cos(angulo), 3), round(r * math.sin(angulo), 3)])
    return puntos


# Zonas relativas -> direccion absoluta segun heading actual del robot
# (N/E/S/O). "front" pasa a ser la direccion del heading; "right" es
# heading girado -90 (derecha); "left" +90; "back" +180. Reusa las
# tablas de robot_state.py (no una copia local) para que el sensado de
# muros y la logica de giro nunca queden desincronizados.


def direcciones_absolutas(heading: str) -> dict:
    """Devuelve {'front': dir_abs, 'right': dir_abs, 'left': dir_abs,
    'back': dir_abs} para el heading actual (N/E/S/O)."""
    return {
        "front": heading,
        "right": GIRO_DERECHA[heading],
        "left": GIRO_IZQUIERDA[heading],
        "back": GIRO_180[heading],
    }
