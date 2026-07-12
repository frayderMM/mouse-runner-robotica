"""Intento 2: ruta optima sobre el mapa descubierto + plan de ejecucion.

- BFS sobre los muros conocidos del intento 1 -> ruta minima garantizada
  (garantizada porque la fase B del explorador verifico cada celda de la ruta).
- Compresion: pasos consecutivos en la misma direccion -> un solo segmento.
- Traduccion a comandos fisicos: cm, ticks de encoder y grados de giro,
  parametrizable con las medidas reales de tu robot.
"""
from .maze import nombre

FLECHA = {(0, 1): "N", (1, 0): "E", (0, -1): "S", (-1, 0): "O"}
GLIFO = {"N": "arriba", "E": "derecha", "S": "abajo", "O": "izquierda"}

# Angulo de giro segun direccion actual -> nueva (positivo = antihorario)
_ANG = {"N": 90, "E": 0, "S": -90, "O": 180}


def comprimir(ruta):
    """[(c1),(c2),...] -> [{'desde','hasta','dir','celdas'}]"""
    segs = []
    i = 0
    while i < len(ruta) - 1:
        dx = ruta[i + 1][0] - ruta[i][0]
        dy = ruta[i + 1][1] - ruta[i][1]
        j = i
        while j < len(ruta) - 1 and (ruta[j + 1][0] - ruta[j][0],
                                     ruta[j + 1][1] - ruta[j][1]) == (dx, dy):
            j += 1
        segs.append({"desde": ruta[i], "hasta": ruta[j],
                     "dir": FLECHA[(dx, dy)], "celdas": j - i})
        i = j
    return segs


def giro_relativo(dir_actual, dir_nueva):
    """Devuelve grados a girar (izquierda positivo): 0, 90, -90 o 180."""
    delta = (_ANG[dir_nueva] - _ANG[dir_actual]) % 360
    if delta == 270:
        delta = -90
    if delta == 180:
        delta = 180  # media vuelta
    return delta


class RobotConfig:
    def __init__(self, celda_cm=30.0, diametro_rueda_cm=6.5,
                 track_width_cm=15.0, ticks_por_vuelta=1560,
                 vel_exploracion_ms=0.15, vel_crucero_ms=0.5):
        self.celda_cm = celda_cm
        self.diam = diametro_rueda_cm
        self.track = track_width_cm
        self.ticks = ticks_por_vuelta
        self.vel_exp = vel_exploracion_ms
        self.vel_run = vel_crucero_ms
        self.circunf = 3.141592653589793 * diametro_rueda_cm

    def ticks_avance(self, celdas):
        cm = celdas * self.celda_cm
        return cm, round(cm / self.circunf * self.ticks)

    def ticks_giro(self, grados):
        # giro sobre el eje: cada rueda recorre un arco de radio track/2
        arco = 3.141592653589793 * self.track * abs(grados) / 360.0
        return arco, round(arco / self.circunf * self.ticks)


def plan_de_ejecucion(ruta, cfg, orientacion_inicial="N"):
    """Convierte la ruta en la lista de comandos fisicos del speed run."""
    segs = comprimir(ruta)
    plan = []
    ori = orientacion_inicial
    for k, s in enumerate(segs, 1):
        g = giro_relativo(ori, s["dir"])
        if g != 0:
            arco, tk = cfg.ticks_giro(g)
            lado = "izquierda" if g > 0 else "derecha"
            if abs(g) == 180:
                lado = "media vuelta"
            plan.append({"cmd": "GIRAR", "grados": g, "lado": lado,
                         "arco_cm": round(arco, 1), "ticks": tk})
            ori = s["dir"]
        cm, tk = cfg.ticks_avance(s["celdas"])
        plan.append({"cmd": "AVANZAR", "seg": k, "celdas": s["celdas"],
                     "dir": s["dir"], "cm": round(cm, 1), "ticks": tk,
                     "desde": nombre(s["desde"]), "hasta": nombre(s["hasta"])})
    return segs, plan


def tiempo_estimado(plan, cfg, t_giro_s=1.2, t_parada_s=0.4):
    """Estimacion simple: rectas a velocidad crucero + costo fijo por giro."""
    t = 0.0
    for p in plan:
        if p["cmd"] == "AVANZAR":
            t += (p["cm"] / 100.0) / cfg.vel_run + t_parada_s
        else:
            t += t_giro_s
    return t


def tiempo_exploracion(n_movs, cfg, t_sensado_s=0.5):
    """Exploracion: cada celda es acelerar-frenar-sensar."""
    por_celda = (cfg.celda_cm / 100.0) / cfg.vel_exp + t_sensado_s
    return n_movs * por_celda
