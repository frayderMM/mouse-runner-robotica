"""Modelo de laberinto para el robot real -- copia deliberada (no import
cruzado de paquete) de la logica de ``simulador_pista/sim/maze.py``, para
que este paquete ROS2 sea autocontenido y compilable en el robot sin
depender de que ``simulador_pista`` este instalado ahi.

A diferencia del simulador (que carga los muros de un .txt), aca el
laberinto arranca SIN muros conocidos -- los va llenando
``ExplorerNode`` a medida que el LiDAR real los detecta. Las
dimensiones (cols, rows) SI se conocen de antemano (regla de la
competencia, como en micromouse), solo los muros internos son
desconocidos al empezar.

Convencion de coordenadas: celda = (col, fila), (0,0) = esquina
inferior izquierda, igual que ``pistas/pista_ejemplo.txt``. A1 = (0,0).
"""
from collections import deque

DIRS = {"N": (0, 1), "E": (1, 0), "S": (0, -1), "O": (-1, 0)}
ORDEN_DESEMPATE_DEFAULT = ["N", "E", "S", "O"]


def wall(a, b):
    return frozenset([a, b])


def nombre(c):
    """(0,0) -> 'A1'"""
    return chr(ord("A") + c[0]) + str(c[1] + 1)


def celda(s):
    """'A1' -> (0,0). Acepta 'a1', 'L8', etc."""
    s = s.strip().upper()
    return (ord(s[0]) - ord("A"), int(s[1:]) - 1)


class Maze:
    def __init__(self, cols, rows, walls=None, start=None, goal=None):
        self.cols, self.rows = cols, rows
        self.walls = set(walls) if walls else set()
        self.start, self.goal = start, goal

    def dentro(self, c):
        return 0 <= c[0] < self.cols and 0 <= c[1] < self.rows

    def vecinos(self, c):
        for d, (dx, dy) in DIRS.items():
            n = (c[0] + dx, c[1] + dy)
            if self.dentro(n):
                yield n, d

    def hay_muro(self, a, b):
        return wall(a, b) in self.walls

    def bfs(self, start, goal, walls=None):
        """Camino mas corto. walls=None usa self.walls; si se pasa un
        set, se usa ese (mapa parcial descubierto por el robot)."""
        W = self.walls if walls is None else walls
        prev = {start: None}
        q = deque([start])
        while q:
            c = q.popleft()
            if c == goal:
                break
            for n, _ in self.vecinos(c):
                if wall(c, n) not in W and n not in prev:
                    prev[n] = c
                    q.append(n)
        if goal not in prev:
            return None
        path = [goal]
        while prev[path[-1]] is not None:
            path.append(prev[path[-1]])
        return path[::-1]

    def flood(self, targets, walls):
        """Distancia BFS desde targets a cada celda, usando solo `walls`
        como muros conocidos (el resto se asume libre -- optimista,
        igual que el explorador del simulador)."""
        dist = {t: 0 for t in targets}
        q = deque(targets)
        while q:
            c = q.popleft()
            for n, _ in self.vecinos(c):
                if wall(c, n) not in walls and n not in dist:
                    dist[n] = dist[c] + 1
                    q.append(n)
        return dist
