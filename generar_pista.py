from collections import Counter
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


ARROW_SYMBOLS = {"^", "v", "<", ">"}


def leer_mapa_ascii(ruta_txt: str):
    """
    Lee un laberinto ASCII con este estilo:

    +--+--+--+
    |I    >  |
    +  +--+  +
    |   ^  M |
    +--+--+--+

    Soporta:
    - Paredes horizontales con --, == o __
    - Paredes verticales con |
    - Inicio: I
    - Meta: M
    - Recorrido por celda con flechas: ^ v < >

    Devuelve un diccionario con la geometría y el contenido.
    """
    ruta = Path(ruta_txt)

    if not ruta.exists():
        raise FileNotFoundError(f"No se encontró el archivo: {ruta.resolve()}")

    lineas = ruta.read_text(encoding="utf-8").splitlines()

    while lineas and not lineas[0].strip():
        lineas.pop(0)

    while lineas and not lineas[-1].strip():
        lineas.pop()

    if len(lineas) < 3:
        raise ValueError("El mapa ASCII no tiene suficientes líneas.")

    primera_horizontal = lineas[0]
    if not primera_horizontal.startswith("+"):
        raise ValueError("La primera línea debe empezar con '+'.")

    columnas = primera_horizontal.count("+") - 1
    filas = (len(lineas) - 1) // 2

    if columnas <= 0 or filas <= 0:
        raise ValueError("No se pudo determinar el tamaño del laberinto.")

    ancho_celda_ascii = 3
    ancho_total = columnas * ancho_celda_ascii + 1
    # Normaliza el ancho de todas las líneas para evitar IndexError con
    # pistas que tengan espacios finales recortados por el editor.
    lineas = [linea.ljust(ancho_total) for linea in lineas]

    horizontales = [[False for _ in range(columnas)] for _ in range(filas + 1)]
    verticales = [[False for _ in range(columnas + 1)] for _ in range(filas)]
    contenido_celdas = [["" for _ in range(columnas)] for _ in range(filas)]
    flechas = [[None for _ in range(columnas)] for _ in range(filas)]

    inicio = None
    meta = None

    for fila_ascii in range(filas + 1):
        indice_linea = fila_ascii * 2
        linea_horizontal = lineas[indice_linea]

        for columna in range(columnas):
            inicio_segmento = columna * ancho_celda_ascii + 1
            fin_segmento = inicio_segmento + 2
            segmento = linea_horizontal[inicio_segmento:fin_segmento]

            horizontales[fila_ascii][columna] = (
                "-" in segmento or "=" in segmento or "_" in segmento
            )

        if fila_ascii == filas:
            continue

        linea_vertical = lineas[indice_linea + 1]

        for borde in range(columnas + 1):
            posicion = borde * ancho_celda_ascii
            caracter = linea_vertical[posicion] if posicion < len(linea_vertical) else " "
            verticales[fila_ascii][borde] = caracter in "|┃│"

        for columna in range(columnas):
            x0 = columna * ancho_celda_ascii + 1
            x1 = x0 + 2
            contenido = linea_vertical[x0:x1]
            contenido_celdas[fila_ascii][columna] = contenido

            if "I" in contenido:
                inicio = (fila_ascii, columna)

            if "M" in contenido:
                meta = (fila_ascii, columna)

            for ch in contenido:
                if ch in ARROW_SYMBOLS:
                    flechas[fila_ascii][columna] = ch
                    break

    return {
        "filas": filas,
        "columnas": columnas,
        "horizontales": horizontales,
        "verticales": verticales,
        "inicio": inicio,
        "meta": meta,
        "contenido_celdas": contenido_celdas,
        "flechas": flechas,
        "ruta_ascii": None,
    }


def datos_desde_maze(mz, ruta=None):
    """
    Construye el mismo diccionario de geometría que `leer_mapa_ascii`, pero a
    partir de un `sim.maze.Maze` ya cargado (usado por main.py), en vez de
    releer un .txt con flechas incrustadas.

    Si se pasa `ruta` (lista de celdas (col, fila) como las que devuelven
    `Maze.bfs` o `Explorer.explorar`), se calculan las flechas del recorrido
    celda por celda.
    """
    from sim.maze import wall

    filas, columnas = mz.rows, mz.cols

    horizontales = [[False] * columnas for _ in range(filas + 1)]
    verticales = [[False] * (columnas + 1) for _ in range(filas)]

    # Borde exterior: siempre cerrado.
    for columna in range(columnas):
        horizontales[0][columna] = True
        horizontales[filas][columna] = True
    for fila_ascii in range(filas):
        verticales[fila_ascii][0] = True
        verticales[fila_ascii][columnas] = True

    for col in range(columnas):
        for fila_logica in range(filas):
            fila_ascii = filas - 1 - fila_logica
            if fila_logica + 1 < filas and wall((col, fila_logica), (col, fila_logica + 1)) in mz.walls:
                horizontales[fila_ascii][col] = True
            if col + 1 < columnas and wall((col, fila_logica), (col + 1, fila_logica)) in mz.walls:
                verticales[fila_ascii][col + 1] = True

    inicio = None
    if mz.start is not None:
        col, fila_logica = mz.start
        inicio = (filas - 1 - fila_logica, col)

    meta = None
    if mz.goal is not None:
        col, fila_logica = mz.goal
        meta = (filas - 1 - fila_logica, col)

    ruta_ascii = None
    if ruta:
        ruta_ascii = [(filas - 1 - fila_logica, col) for col, fila_logica in ruta]

    return {
        "filas": filas,
        "columnas": columnas,
        "horizontales": horizontales,
        "verticales": verticales,
        "inicio": inicio,
        "meta": meta,
        "flechas": [[None] * columnas for _ in range(filas)],
        "ruta_ascii": ruta_ascii,
    }


def _dibujar_recorrido(ax, filas, ruta_ascii):
    """
    Dibuja el recorrido como una línea continua sobre los centros de celda,
    coloreada en degradé (inicio oscuro -> meta claro) para que se note el
    orden de avance, con flechas de dirección espaciadas y un círculo hueco
    sobre las celdas por las que el robot pasó más de una vez (retrocesos).
    """
    if not ruta_ascii or len(ruta_ascii) < 2:
        return

    puntos = [(col + 0.5, filas - fila - 0.5) for fila, col in ruta_ascii]
    n_tramos = len(puntos) - 1
    cmap = plt.get_cmap("plasma")

    for i in range(n_tramos):
        (x0, y0), (x1, y1) = puntos[i], puntos[i + 1]
        color = cmap(i / max(n_tramos - 1, 1))
        ax.plot([x0, x1], [y0, y1], color=color, linewidth=3.2, alpha=0.6,
                solid_capstyle="round", zorder=3.5)

    # Flechas de direccion espaciadas para no saturar el dibujo
    paso = max(1, n_tramos // 25)
    for i in range(0, n_tramos, paso):
        (x0, y0), (x1, y1) = puntos[i], puntos[i + 1]
        xm, ym = (x0 + x1) / 2, (y0 + y1) / 2
        dx, dy = (x1 - x0) * 0.32, (y1 - y0) * 0.32
        color = cmap(i / max(n_tramos - 1, 1))
        ax.annotate(
            "", xy=(xm + dx, ym + dy), xytext=(xm - dx, ym - dy),
            arrowprops=dict(arrowstyle="-|>", color=color, lw=1.6),
            zorder=4,
        )

    # Celdas visitadas mas de una vez -> retroceso
    repetidas = {celda for celda, veces in Counter(ruta_ascii).items() if veces > 1}
    for fila, col in repetidas:
        x, y = col + 0.5, filas - fila - 0.5
        ax.scatter([x], [y], s=110, facecolors="none", edgecolors="#d62728",
                   linewidths=1.6, zorder=4.5)


def _dibujar_en_ejes(ax, datos, titulo, mostrar_nombres=True, mostrar_flechas=True):
    filas = datos["filas"]
    columnas = datos["columnas"]
    horizontales = datos["horizontales"]
    verticales = datos["verticales"]
    inicio = datos["inicio"]
    meta = datos["meta"]
    flechas = datos["flechas"]

    # Fondo
    ax.add_patch(Rectangle((0, 0), columnas, filas, facecolor="white", edgecolor="none"))

    # Cuadrícula tenue
    for x in range(columnas + 1):
        ax.plot([x, x], [0, filas], color="#dddddd", linewidth=0.6, zorder=0)
    for y in range(filas + 1):
        ax.plot([0, columnas], [y, y], color="#dddddd", linewidth=0.6, zorder=0)

    # Paredes horizontales -> negras
    for fila_borde in range(filas + 1):
        y = filas - fila_borde
        for columna in range(columnas):
            if horizontales[fila_borde][columna]:
                ax.plot(
                    [columna, columna + 1],
                    [y, y],
                    color="black",
                    linewidth=5,
                    solid_capstyle="butt",
                    zorder=3,
                )

    # Paredes verticales -> negras
    for fila in range(filas):
        y_superior = filas - fila
        y_inferior = y_superior - 1
        for borde in range(columnas + 1):
            if verticales[fila][borde]:
                ax.plot(
                    [borde, borde],
                    [y_inferior, y_superior],
                    color="black",
                    linewidth=5,
                    solid_capstyle="butt",
                    zorder=3,
                )

    # Nombres de celdas
    if mostrar_nombres:
        for fila in range(filas):
            numero_fila = filas - fila
            for columna in range(columnas):
                nombre = f"{chr(65 + columna)}{numero_fila}"
                ax.text(
                    columna + 0.08,
                    filas - fila - 0.15,
                    nombre,
                    fontsize=8,
                    color="#666666",
                    alpha=0.7,
                    ha="left",
                    va="top",
                    zorder=1,
                )

    # Recorrido
    if mostrar_flechas:
        ruta_ascii = datos.get("ruta_ascii")
        if ruta_ascii:
            _dibujar_recorrido(ax, filas, ruta_ascii)
        else:
            # Sin orden de visita disponible (mapa ASCII con flechas ya
            # incrustadas): se dibuja una flecha suelta por celda.
            for fila in range(filas):
                for columna in range(columnas):
                    flecha = flechas[fila][columna]
                    if flecha:
                        x = columna + 0.5
                        y = filas - fila - 0.5
                        ax.text(
                            x,
                            y,
                            flecha,
                            fontsize=18,
                            fontweight="bold",
                            color="#1f77b4",
                            ha="center",
                            va="center",
                            zorder=4,
                        )

    # Inicio y meta
    for punto, color, letra in ((inicio, "#2ca02c", "I"), (meta, "#d62728", "M")):
        if punto is None:
            continue
        fila, columna = punto
        x = columna + 0.5
        y = filas - fila - 0.5

        ax.scatter([x], [y], s=220, color=color, edgecolors="white", linewidths=2, zorder=5)
        ax.text(x, y, letra, fontsize=11, fontweight="bold", color="white",
                ha="center", va="center", zorder=6)

    ax.set_title(titulo, fontsize=14, fontweight="bold")
    ax.set_xlim(-0.2, columnas + 0.2)
    ax.set_ylim(-0.2, filas + 0.2)
    ax.set_aspect("equal")
    ax.axis("off")


def dibujar_laberinto(
    datos,
    titulo="Pista generada desde ASCII",
    guardar_como="pista_generada.png",
    mostrar_nombres=True,
    mostrar_flechas=True,
    mostrar=True,
):
    fig, ax = plt.subplots(figsize=(14, 9))
    _dibujar_en_ejes(ax, datos, titulo, mostrar_nombres, mostrar_flechas)
    plt.tight_layout()

    if guardar_como:
        plt.savefig(guardar_como, dpi=220, bbox_inches="tight")
        print(f"Imagen guardada en: {Path(guardar_como).resolve()}")

    if mostrar:
        plt.show()
    else:
        plt.close(fig)


def generar_tres_vistas(
    mz,
    ruta1=None,
    ruta2=None,
    carpeta_salida="pistas",
    nombre_archivo="pistas_comparadas.png",
    mostrar=True,
):
    """
    Dibuja en una sola figura, lado a lado: la pista original (sin recorrido),
    el recorrido del intento 1 (exploración) y el recorrido del intento 2
    (speed run). Pensada para llamarse desde main.py con el `Maze` y las
    rutas ya calculadas.
    """
    carpeta = Path(carpeta_salida)
    carpeta.mkdir(parents=True, exist_ok=True)

    datos_original = datos_desde_maze(mz)
    datos_r1 = datos_desde_maze(mz, ruta1)
    datos_r2 = datos_desde_maze(mz, ruta2)

    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(24, 9))
    _dibujar_en_ejes(ax1, datos_original, "Pista original")
    _dibujar_en_ejes(ax2, datos_r1, "Intento 1 - Exploración")
    _dibujar_en_ejes(ax3, datos_r2, "Intento 2 - Speed run")
    plt.tight_layout()

    salida = carpeta / nombre_archivo
    fig.savefig(salida, dpi=200, bbox_inches="tight")
    print(f"Imagen guardada en: {salida.resolve()}")

    if mostrar:
        plt.show()
    else:
        plt.close(fig)

    return salida


def main():
    archivo_entrada = "pistas/pista_img.txt"
    archivo_salida = "pistas/pista_generada.png"

    datos = leer_mapa_ascii(archivo_entrada)

    print(f"Mapa leído correctamente: {datos['columnas']} columnas × {datos['filas']} filas")
    print(f"Inicio: {datos['inicio']}")
    print(f"Meta: {datos['meta']}")

    dibujar_laberinto(
        datos,
        titulo="Pista generada desde ASCII",
        guardar_como=archivo_salida,
        mostrar_nombres=True,
        mostrar_flechas=True,
    )


if __name__ == "__main__":
    main()
