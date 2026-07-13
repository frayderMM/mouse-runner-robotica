# Simulador de pista — dos intentos (exploración + speed run)

Simula la estrategia de dos intentos estilo micromouse:

- **Intento 1**: exploración con flood fill. El robot no conoce la pista; solo "ve"
  los muros de la celda donde está (LiDAR simulado) y decide en cada celda hacia
  dónde ir. Incluye fase B de verificación para garantizar que la ruta encontrada
  sea la óptima.
- **Intento 2**: BFS sobre el mapa descubierto, compresión en segmentos rectos y
  traducción a comandos físicos (cm, ticks de encoder, grados de giro).

Solo requiere **Python 3.8+**, sin dependencias externas.

## Uso rápido

```bash
python main.py pistas/pista_ejemplo.txt              # pista del examen 12x8
python main.py pistas/pista_chica.txt --detalle      # log celda por celda
python main.py pistas/mi_pista.txt --inicio A1 --meta L8
```

## Cómo escribir tu propia pista

Crea un `.txt` con este formato (cada celda mide 3 caracteres de ancho × 2 de alto):

```
+--+--+--+--+
|        |M |
+  +--+  +  +
|  |        |
+  +  +--+  +
|I    |     |
+--+--+--+--+
```

- `--` entre dos `+` = muro horizontal
- `|` = muro vertical
- espacio = paso libre
- `I` = inicio, `M` = meta (opcionales: puedes usar `--inicio B2 --meta D5`)
- El borde exterior debe estar cerrado (el validador te avisa si la pista no tiene solución)

Coordenadas: columnas A, B, C... de izquierda a derecha; filas 1, 2, 3... de abajo
hacia arriba (A1 = esquina inferior izquierda).

## Opciones

| Flag | Qué hace | Default |
|---|---|---|
| `--detalle` | Log celda por celda: qué ve el LiDAR, qué opciones evalúa y qué decide | off |
| `--lidar N` | Alcance del LiDAR en celdas (1 = solo la celda actual; 3 = ve muros a 3 celdas en línea recta) | 1 |
| `--inicio A1` / `--meta L8` | Sobrescriben las marcas I/M de la pista | — |
| `--celda-cm` | Tamaño de la celda real | 30 |
| `--rueda-cm` | Diámetro de rueda | 6.5 |
| `--track-cm` | Distancia entre ruedas | 15 |
| `--ticks` | Ticks de encoder por vuelta de rueda | 1560 |
| `--exportar mapa.yaml` | Guarda el mapa descubierto en formato 4 bits por celda (N/E/S/O) — el mismo que usarías en el robot real | — |
| `--exportar-plan plan.json` | Exporta el plan de ejecución (intento 2) en JSON, solo como referencia/comparación offline | — |

## Ejecución en el robot real — `robot/granprix_bot`

Este simulador tiene su contraparte en ROS2 dentro de `robot/granprix_bot/`:
un paquete `ament_python` que implementa **la misma lógica** (exploración con
flood fill celda por celda + speed run con BFS) para correr en el Yahboom
ROSMASTER R2 real, sobre la pista de `pistas/pista_ejemplo.txt` (12×8,
celdas de 30×30 cm). Construido desde cero en esta carpeta — no depende del
repo "Reto Final".

Ver `robot/README.md` para el detalle completo: cómo avanza celda por celda,
calibración usada (tomada de `AVANCE_Y_GIRO_CALIBRADO.md` y
`CALIBRACION_LIDAR_VISION.md`, en la raíz de este repo), y cómo compilar y
lanzar en el robot.

## Qué muestra la salida

1. **Intento 1**: movimientos totales (fase A + fase B), celdas sensadas, muros
   descubiertos, "costo de la ignorancia" (movimientos extra sobre el óptimo),
   tiempo estimado, y el dibujo de la pista con el recorrido (`*` = celda visitada
   más de una vez = retroceso).
2. **Intento 2**: ruta óptima con garantía (`optima garantizada: True` significa
   que la fase B verificó cada celda de la ruta), segmentos rectos y giros, y el
   **plan de ejecución físico**: cada `AVANZAR` con sus cm y ticks de encoder, cada
   `GIRAR` con su lado, grados y arco de rueda.

## Experimentos interesantes

```bash
# ¿Cuánto mejora un LiDAR de mayor alcance? (compara "costo de la ignorancia")
python main.py pistas/pista_ejemplo.txt --lidar 1
python main.py pistas/pista_ejemplo.txt --lidar 5

# Cambia el orden de desempate en sim/maze.py -> ORDEN_DESEMPATE = ["E","N","S","O"]
# y mira cómo cambia la exploración en la pista del examen (spoiler: 18 movs al primer intento)
```

## Estructura

```
simulador_pista/
├── main.py            # CLI y reportes
├── sim/
│   ├── maze.py        # modelo, parser ASCII, BFS, flood fill, validación
│   ├── explorer.py    # intento 1: flood fill + fase B de verificación
│   ├── speedrun.py    # intento 2: compresión de segmentos + comandos físicos
│   └── render.py      # dibujo ASCII de la pista con recorridos
└── pistas/
    ├── pista_ejemplo.txt   # la pista 12x8 del examen (BFS = 18 movimientos)
    └── pista_chica.txt     # 4x3 para probar rápido (activa la fase B)
```
