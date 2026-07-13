"""Funciones geometricas compartidas: cuaterniones y angulos.

Mismas formulas que ``logica_pared_derecha_robot.md`` /
``AVANCE_Y_GIRO_CALIBRADO.md`` usan para leer el yaw de ``/odom_raw``.
"""

import math


def yaw_from_quaternion(q) -> float:
    """Extrae el yaw (rotacion sobre Z) en radianes de un quaternion.

    ``q`` es cualquier objeto con atributos x, y, z, w
    (geometry_msgs/Quaternion).
    """
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    """Normaliza un angulo en radianes al rango (-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def angle_diff(target: float, current: float) -> float:
    """Diferencia angular mas corta target - current, en (-pi, pi]."""
    return normalize_angle(target - current)
