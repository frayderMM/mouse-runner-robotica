"""Dibujo ASCII de la pista con un recorrido encima.

Simbolos dentro de las celdas:
  I  inicio      M  meta
  ^ > v <        direccion del paso por esa celda (ultimo paso si se repite)
  *              celda visitada mas de una vez (retroceso)
"""
from .maze import wall

FLECHA = {(0, 1): "^", (1, 0): ">", (0, -1): "v", (-1, 0): "<"}


def dibujar(mz, ruta=None, mostrar_solo_conocidos=None):
    """Devuelve el laberinto como string. Si mostrar_solo_conocidos es un set
    de muros, dibuja solo esos (vista del robot); si es None, dibuja todo."""
    W = mz.walls if mostrar_solo_conocidos is None else mostrar_solo_conocidos

    marca = {}
    repetida = set()
    if ruta:
        vistos = set()
        for i, c in enumerate(ruta):
            if c in vistos:
                repetida.add(c)
            vistos.add(c)
            if i < len(ruta) - 1:
                d = (ruta[i + 1][0] - c[0], ruta[i + 1][1] - c[1])
                marca[c] = FLECHA[d]
        marca[mz.start] = "I"
        marca[mz.goal] = "M"

    out = []
    # encabezado de columnas
    out.append("    " + "".join(f" {chr(ord('A')+c)} " for c in range(mz.cols)))
    for r in range(mz.rows - 1, -1, -1):
        # linea de muros superiores
        top = "   +"
        for c in range(mz.cols):
            arriba = r == mz.rows - 1 or wall((c, r), (c, r + 1)) in W
            top += ("--" if arriba else "  ") + "+"
        out.append(top)
        # linea de celdas
        mid = f"{r+1:>2} |" if (mz.cols and True) else "|"
        fila = f"{r+1:>2} "
        fila += "|"
        for c in range(mz.cols):
            s = marca.get((c, r), " ")
            extra = "*" if (c, r) in repetida and s not in "IM" else " "
            fila += f"{s}{extra}"
            der = c == mz.cols - 1 or wall((c, r), (c + 1, r)) in W
            fila += "|" if der else " "
        out.append(fila)
    # borde inferior
    bot = "   +"
    for c in range(mz.cols):
        bot += "--+"
    out.append(bot)
    return "\n".join(out)
