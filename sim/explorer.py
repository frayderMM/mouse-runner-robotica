"""Intento 1: exploracion tipo micromouse con flood fill.

El robot solo conoce los muros de las celdas que ya piso (LiDAR de celda actual).
Donde no tiene informacion, asume que NO hay muro (optimista).
En cada celda va al vecino accesible con menor distancia flood a la meta.
Desempate segun ORDEN_DESEMPATE (por defecto N > E > S > O).

Al llegar a la meta, fase B: verifica que la ruta candidata optima (BFS sobre
lo conocido) pase 100% por celdas ya sensadas; si no, va a sensarlas.
Asi la ruta del intento 2 queda GARANTIZADA como optima.
"""
from .maze import wall, nombre, DIRS, ORDEN_DESEMPATE

INF = 10 ** 9


class Explorer:
    def __init__(self, maze, lidar_alcance=1):
        """lidar_alcance=1: solo la celda actual.
        lidar_alcance>1: tambien sensa muros de celdas en linea recta
        hasta esa distancia (simula un LiDAR de mayor rango)."""
        self.mz = maze
        self.alcance = lidar_alcance
        self.conocidos = set()      # muros descubiertos
        self.sensadas = set()       # celdas cuyas 4 paredes ya son conocidas
        self.log = []               # registro celda por celda

    def _sense_celda(self, c):
        nuevos = []
        for n, d in self.mz.vecinos(c):
            w = wall(c, n)
            if w in self.mz.walls and w not in self.conocidos:
                self.conocidos.add(w)
                nuevos.append(d)
        self.sensadas.add(c)
        return nuevos

    def sense(self, c):
        """Sensa la celda actual y, si alcance>1, celdas visibles en linea recta."""
        nuevos = self._sense_celda(c)
        for d, (dx, dy) in DIRS.items():
            cur = c
            for _ in range(self.alcance - 1):
                nxt = (cur[0] + dx, cur[1] + dy)
                if not self.mz.dentro(nxt) or wall(cur, nxt) in self.mz.walls:
                    break  # el rayo choca con un muro (que ya quedo registrado)
                self._sense_celda(nxt)
                cur = nxt
        return nuevos

    def paso(self, pos, targets):
        """Un paso de decision flood fill. Devuelve (siguiente, detalle_log)."""
        dist = self.mz.flood(targets, self.conocidos)
        opciones = []
        for n, d in self.mz.vecinos(pos):
            if wall(pos, n) not in self.conocidos:
                opciones.append((dist.get(n, INF), ORDEN_DESEMPATE.index(d), d, n))
        opciones.sort()
        _, _, d_elegida, siguiente = opciones[0]
        detalle = {
            "en": pos,
            "opciones": [(d, n, dv) for dv, _, d, n in sorted(opciones)],
            "va": (d_elegida, siguiente),
        }
        return siguiente, detalle

    def explorar(self, max_pasos=2000):
        mz = self.mz
        pos = mz.start
        ruta = [pos]
        muros_iniciales = self.sense(pos)
        self.log.append({"en": pos, "muros_nuevos": muros_iniciales,
                         "opciones": [], "va": None, "fase": "A"})

        # Fase A: llegar a la meta
        while pos != mz.goal and len(ruta) < max_pasos:
            pos, detalle = self.paso(pos, [mz.goal])
            ruta.append(pos)
            detalle["muros_nuevos"] = self.sense(pos)
            detalle["fase"] = "A"
            self.log.append(detalle)
        fase_a = len(ruta) - 1

        # Fase B: verificar ruta candidata
        while len(ruta) < max_pasos:
            candidata = mz.bfs(mz.start, mz.goal, walls=self.conocidos)
            pendientes = [c for c in candidata if c not in self.sensadas]
            if not pendientes:
                break
            pos, detalle = self.paso(pos, pendientes)
            ruta.append(pos)
            detalle["muros_nuevos"] = self.sense(pos)
            detalle["fase"] = "B"
            self.log.append(detalle)

        return {
            "ruta": ruta,
            "fase_a": fase_a,
            "fase_b": len(ruta) - 1 - fase_a,
            "sensadas": self.sensadas,
            "conocidos": self.conocidos,
        }
