"""Modelo del laberinto y parser del formato ASCII.

Formato de pista (celdas de 3 chars de ancho x 2 de alto):

    +--+--+--+
    |     |M |
    +  +--+  +
    |I |     |
    +--+--+--+

  '--' entre dos '+'  -> muro horizontal
  '|'                 -> muro vertical
  espacio             -> paso libre
  'I' dentro de una celda -> inicio   (opcional, se puede pasar por CLI)
  'M' dentro de una celda -> meta     (opcional, se puede pasar por CLI)

Convencion de coordenadas: celda = (col, fila), con (0,0) abajo-izquierda.
A1 = (0,0). Notacion "A1", "L8", etc. soportada para entrada/salida.
"""
from collections import deque

DIRS = {"N": (0, 1), "E": (1, 0), "S": (0, -1), "O": (-1, 0)}
ORDEN_DESEMPATE = ["N", "E", "S", "O"]  # cambia el orden y cambia la exploracion


def wall(a, b):
    return frozenset([a, b])


def nombre(c):
    """(0,0) -> 'A1'"""
    return chr(ord("A") + c[0]) + str(c[1] + 1)


def celda(s):
    """'A1' -> (0,0). Acepta 'a1', 'B12', etc."""
    s = s.strip().upper()
    return (ord(s[0]) - ord("A"), int(s[1:]) - 1)


class Maze:
    def __init__(self, cols, rows, walls, start=None, goal=None):
        self.cols, self.rows = cols, rows
        self.walls = set(walls)
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
        """Camino mas corto. walls=None usa los muros reales del laberinto;
        si se pasa un set, se usa ese (mapa parcial del robot)."""
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
        """Distancia BFS desde targets a cada celda, usando solo `walls`."""
        dist = {t: 0 for t in targets}
        q = deque(targets)
        while q:
            c = q.popleft()
            for n, _ in self.vecinos(c):
                if wall(c, n) not in walls and n not in dist:
                    dist[n] = dist[c] + 1
                    q.append(n)
        return dist

    def validar(self):
        errores = []
        if self.start is None:
            errores.append("No hay inicio: marca 'I' en la pista o usa --inicio A1")
        if self.goal is None:
            errores.append("No hay meta: marca 'M' en la pista o usa --meta L8")
        if self.start and not self.dentro(self.start):
            errores.append(f"Inicio {nombre(self.start)} fuera de la grilla")
        if self.goal and not self.dentro(self.goal):
            errores.append(f"Meta {nombre(self.goal)} fuera de la grilla")
        if not errores and self.bfs(self.start, self.goal) is None:
            errores.append("La pista no tiene solucion: no existe camino inicio->meta")
        return errores


def parse_maze(texto):
    lineas = [l.rstrip("\n") for l in texto.strip("\n").split("\n")]
    rows = (len(lineas) - 1) // 2
    cols = (max(len(l) for l in lineas) - 1) // 3
    # normalizar ancho de lineas
    ancho = cols * 3 + 1
    lineas = [l.ljust(ancho) for l in lineas]

    walls, start, goal = set(), None, None
    for r in range(rows):          # r=0 es la fila superior del texto
        y = rows - 1 - r           # fila logica (0 abajo)
        fila_celdas = lineas[r * 2 + 1]
        fila_bajo = lineas[r * 2 + 2]
        for c in range(cols):
            interior = fila_celdas[c * 3 + 1: c * 3 + 3]
            if "I" in interior:
                start = (c, y)
            if "M" in interior:
                goal = (c, y)
            # muro al este de (c,y)
            if c < cols - 1 and fila_celdas[c * 3 + 3] == "|":
                walls.add(wall((c, y), (c + 1, y)))
            # muro al sur de (c,y)
            if y > 0 and fila_bajo[c * 3 + 1] == "-":
                walls.add(wall((c, y), (c, y - 1)))
    return Maze(cols, rows, walls, start, goal)


def cargar(ruta):
    with open(ruta, encoding="utf-8") as f:
        return parse_maze(f.read())
