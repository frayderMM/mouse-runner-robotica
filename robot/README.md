# granprix_bot — el simulador corriendo en el robot real

Paquete ROS2 (`ament_python`) que implementa en el Yahboom ROSMASTER R2
la **misma lógica** que `simulador_pista` ya valida offline: intento 1
(exploración con flood fill, celda por celda) + intento 2 (speed run,
BFS sobre el mapa descubierto, tramos comprimidos). Construido desde
cero en esta carpeta — no depende ni modifica el repo "Reto Final".

Pista objetivo: `pistas/pista_ejemplo.txt` (12×8 celdas de 30×30 cm,
inicio **A1**, meta **L8**, óptimo conocido = 18 movimientos). El
robot debe reproducir el laberinto **exactamente** como está en ese
archivo.

---

## 1. Estructura

```text
robot/granprix_bot/
├── granprix_bot/
│   ├── maze_model.py     # Maze (bfs/flood), copia autocontenida de sim/maze.py
│   ├── geometry_utils.py # yaw de cuaternión, diferencia angular
│   ├── lidar.py          # /scan -> distancia mínima por zona (front/right/left/back)
│   ├── robot_state.py    # GridPose (celda+heading) + parámetros compartidos
│   ├── motion.py         # NodoRobotBase: primitivas avanzar/girar/pausa cerradas por odometría
│   ├── explorer_node.py  # Ronda 1: flood fill fase A + fase B, sensado real por LiDAR
│   └── speedrun_node.py  # Ronda 2: carga el mapa, BFS, comprime tramos, ejecuta rápido
├── config/granprix_bot_params.yaml
├── launch/explorar.launch.py
├── launch/speedrun.launch.py
├── package.xml / setup.py / setup.cfg
└── resource/granprix_bot
```

No usa mensajes personalizados (a diferencia del proyecto anterior,
que tenía un paquete `capytown_interfaces` aparte): `lidar.py` calcula
las zonas directamente de `sensor_msgs/LaserScan`, así que es un solo
paquete `ament_python`, sin pasos de compilación de mensajes
(`ament_cmake`) de por medio.

---

## 2. Cómo avanza el robot, celda por celda

Igual a como se describió: el robot arranca **orientado al norte**,
en el **centro de A1**. Cada celda mide **30×30 cm**.

### Ronda 1 — Exploración (`explorer_node.py`)

Por cada celda, en este orden:

1. **Llega** al centro de la celda (avance de 30 cm cerrado por
   odometría, ver sección 3).
2. **Se detiene 1 s** (`tiempo_pausa_antes_girar_s`, reusado como
   pausa de sensado).
3. **Sensa** con el LiDAR las 4 direcciones cardinales relativas a su
   heading actual (`lidar.py` + `robot_state.leer_muros_celda_actual`)
   y registra qué muros son nuevos.
4. **Decide** el siguiente movimiento con flood fill: distancia BFS a
   la meta usando *solo* los muros ya conocidos (una pared no sensada
   se asume abierta — optimista, igual que `sim/explorer.py`), con
   desempate N > E > S > O.
5. Si la dirección elegida no es la que ya mira, **gira** (arco lento
   cerrado por yaw, sección 3); si ya mira para allá, avanza directo.
6. Repite desde el paso 1.

Al llegar a la meta, entra en **fase B**: verifica que la ruta
candidata (BFS sobre lo ya conocido) pase 100% por celdas ya
sensadas; si falta alguna, sigue explorando hasta sensarla. Esto
**garantiza** que la ruta de la Ronda 2 sea la óptima real — misma
garantía que ya probaba `sim/explorer.py` offline.

Al terminar, guarda el mapa descubierto en
`~/capytown_resultados/mapa_descubierto.yaml` (mismo formato 4-bits
N/E/S/O que `main.py --exportar` del simulador).

### Ronda 2 — Speed run (`speedrun_node.py`)

Carga ese mapa, calcula el camino mínimo garantizado (BFS) y lo
**comprime** en tramos rectos (varias celdas seguidas = un solo
avance continuo, **sin pausa entre celda y celda** — a diferencia de
la exploración, acá el mapa ya se conoce, no hace falta sensar en
cada una). Antes de cada giro sí hay una pausa fija (mismo criterio
de "movimiento separado" que la exploración).

Probado offline (ver sección 6): sobre `pistas/pista_ejemplo.txt` da
exactamente el plan ya conocido — **4 tramos, 18 movimientos**:

| Tramo | Dirección | Celdas | Distancia |
|---|---|---:|---:|
| 1 | N | 5 | 150 cm |
| 2 | E | 4 | 120 cm |
| 3 | N | 2 | 60 cm |
| 4 | E | 7 | 210 cm |

---

## 3. Calibración usada (de los documentos en la raíz de `simulador_pista`)

### Avance y giro — `AVANCE_Y_GIRO_CALIBRADO.md`

Chasis **Ackermann**: no rota sobre su propio eje, así que "girar" es
un arco (avance lineal lento + velocidad angular máxima), no una
rotación en el sitio. Ambas primitivas cierran el lazo contra
`/odom_raw`, corregido con los factores de escala ya calibrados:

| Parámetro | Valor |
|---|---:|
| `factor_dist_odom` | 0.9474 |
| `factor_ang_odom` | 0.9899 |
| `velocidad_recta_mps` | 0.15 |
| `velocidad_giro_lineal_mps` | 0.06 |
| `velocidad_giro_angular_radps` | 0.6 |
| `angulo_giro_deg` | 90.0 |
| `margen_seguridad_giro_deg` | 60.0 (tope de seguridad, **relativo** al objetivo de cada giro: 90+60=150 para giros normales, 180+60=240 para ATRAS) |
| `margen_singularidad_atras_deg` | 4.0 (solo el giro ATRAS apunta a 180−4°, para no quedar justo en el punto de wraparound de ±180°) |

> **Corregido en revisión de código:** el giro ya no se detiene con
> una tolerancia restada del objetivo (antes todo giro de 90° quedaba
> sistemáticamente ~4° corto), y el tope de seguridad ahora es un
> margen relativo al objetivo de cada giro, no un ángulo absoluto fijo
> (antes 150° absoluto era **menor** que el objetivo real de un giro
> ATRAS de 180°, así que todo giro de 180° se cortaba en ~150° reales
> aunque el modelo lógico ya aplicara el giro completo). Ver
> `motion.py::_tick_giro`.

### LiDAR — `CALIBRACION_LIDAR_VISION.md`

| Parámetro | Valor |
|---|---:|
| `front_offset_deg` | 180.0 |
| `invert_left_right` | true |
| `front_window_deg` | [-15, 15] |
| `right_window_deg` | [-110, -70] |
| `left_window_deg` | [70, 110] |
| `back_window_deg` | [165, 195] |

**Nuevo, específico de este paquete** (no estaba en el proyecto
anterior porque ahí no se sensaba por celda): `umbral_pared_m: 0.20`
— si una zona mide menos que esto, se marca pared en esa dirección.
Sale de `celda_cm/2` (15 cm, el punto medio de una celda de 30 cm) +
5 cm de margen. **Verificar en pista real** antes de confiar en él:
si el LiDAR no está montado cerca del centro del robot, puede hacer
falta ajustarlo (ver nota equivalente en `CALIBRACION_LIDAR_VISION.md`
sobre la distancia objetivo de seguimiento de pared).

No usa seguimiento de pared por LiDAR durante el avance (no hay
`right_window_deg`/`left_window_deg` de por medio ahí) — la corrección
en línea recta es puramente por **odometría**, ver siguiente sección.

### Corrección en línea recta — `tipo_correccion` (`motion.py::_correccion_recta`)

Durante `AVANZAR` (tanto en exploración como en speed run), en vez de
avanzar "a ciegas", el robot corrige continuamente contra la
odometría, comparando su posición/yaw actuales contra los que tenía
al **empezar** ese avance (`_iniciar_avance` guarda `(x0,y0,yaw0)`):

```text
error_angular = angulo_faltante(yaw0, yaw_actual)   # objetivo - actual
error_lateral = proyeccion perpendicular de (x-x0, y-y0) sobre yaw0

correccion = kp_angulo * error_angular - kp_lateral * error_lateral
angular.z  = recortar(correccion, ±angular_max_correccion_radps)
linear.x   = velocidad_recta_mps   (sin cambios)
```

| Parámetro | Valor |
|---|---:|
| `tipo_correccion` | `angular_simple` (`ninguna` = avance ciego, comportamiento anterior) |
| `kp_angulo_recto` | 2.2 |
| `kp_lateral_recto` | 1.1 |
| `angular_max_correccion_radps` | 0.3 |

**Nota de signo (importante, ya verificada con simulación numérica
antes de subir esto al robot):** `error_angular` es "objetivo menos
actual" (`angle_diff(yaw0, yaw_actual)`), no al revés — con
`kp_angulo` positivo, esa es la convención que **cierra** el lazo. Se
probó explícitamente con el signo invertido (`actual - objetivo`) en
una simulación cinemática simple: el yaw crece sin parar hasta saturar
el límite angular en menos de 4 segundos (realimentación positiva,
mismo tipo de divergencia que ya advertía `CALIBRACION_LIDAR_VISION.md`
sección 1.4 para `right_line_angle_rad`). Con el signo actual
(`objetivo - actual`), la misma simulación converge en todos los
casos probados (error solo angular, solo lateral, ambos, y con
distintos headings iniciales) — el error angular se corrige en ~2s,
el lateral más lento (tramos largos lo corrigen mejor que los
cortos). **Aun así, esto es solo una simulación cinemática simple —
falta verificar en el robot real** (sección 5, orden de calibración):

- Si no corrige lo suficiente (sigue derivando): subir `kp_angulo_recto`
  y/o `kp_lateral_recto`.
- Si oscila de un lado a otro: bajar las ganancias, o bajar
  `angular_max_correccion_radps`.
- Si diverge (el error crece en vez de achicarse): **antes que nada**,
  sospechar del signo — revisar `_correccion_recta` en `motion.py`.

### Seguridad (igual en ambos nodos, cualquier estado)

| Parámetro | Valor |
|---|---:|
| `umbral_colision_m` | 0.10 |
| `tiempo_espera_obstaculo_s` | 2.0 |
| `max_intentos_obstaculo` | 5 (si el obstáculo sigue bloqueando tras 5 esperas de 2s = 10s, aborta la misión en vez de esperar para siempre en silencio) |

---

## 3.1 Correcciones tras revisión de código (2026-07)

Una revisión de código con 8 agentes en paralelo encontró varios
fallos reales antes de tocar el robot. Corregidos:

- **`explorer_node.py` — crash por `IndexError`/`TypeError`.** `_paso()`
  y la verificación de fase B asumían (como el simulador, que usa
  datos ground-truth) que siempre hay una dirección abierta y que el
  BFS siempre encuentra camino. Con sensado LiDAR real y ruidoso, esto
  puede no cumplirse. Ahora ambos casos se manejan con un aborto
  controlado (`_fallar()`: detiene el robot, guarda el mapa parcial
  para diagnóstico, no crashea el nodo).
- **`motion.py` — giro de 180° cortado en ~150°.** Ver sección 3,
  tabla de avance y giro.
- **`motion.py` — todo giro de 90° quedaba ~4° corto.** Idem.
- **`explorer_node.py` — se sensaba antes de la pausa de 1s, no
  después.** El orden ahora es literal: llega → pausa 1s → sensa →
  decide (antes sensaba apenas detectaba la llegada, con el chasis
  posiblemente todavía asentándose del frenado).
- **`speedrun_node.py` — recursos de ROS2 sin cerrar si el mapa falta.**
  `main()` ahora construye el nodo dentro del `try/finally`.
- **Duplicación de tablas de dirección.** `lidar.py` reusa las tablas
  de rotación de `robot_state.py` en vez de mantener una copia propia;
  `speedrun_node.py` deriva `_DIR_POR_DELTA` de `maze_model.DIRS` en
  vez de copiarlo a mano.
- **Espera infinita ante un obstáculo.** Ver `max_intentos_obstaculo`
  arriba.

---

## 4. Compilar y correr en el robot

Mismo flujo de siempre (PC → GitHub → robot), ver
`FLUJO_DE_TRABAJO.md` en la raíz de `simulador_pista`.

```bash
# En el robot, dentro del workspace ROS2:
cd /root/yahboomcar_ws/src
git clone https://github.com/frayderMM/mouse-runner-robotica.git simulador-pista
ln -s /root/yahboomcar_ws/src/simulador-pista/robot/granprix_bot /root/yahboomcar_ws/src/granprix_bot
cd /root/yahboomcar_ws
colcon build --packages-select granprix_bot
source install/setup.bash
```

> Se enlaza (`ln -s`) en vez de clonar directo dentro de `src/` porque
> el repo completo trae también `simulador_pista` (Python puro, no es
> un paquete ROS2) — así `colcon build` solo ve `robot/granprix_bot`
> como paquete.

### Ronda 1 — Exploración

```bash
ros2 launch granprix_bot explorar.launch.py
```

### Ronda 2 — Speed run (después de correr la Ronda 1 al menos una vez)

```bash
ros2 launch granprix_bot speedrun.launch.py
```

### Verificar en vivo

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /odom_raw
cat ~/capytown_resultados/mapa_descubierto.yaml   # despues de la Ronda 1
```

---

## 5. Orden de calibración recomendado en pista

1. **Factores de odometría** (`factor_dist_odom`, `factor_ang_odom`) —
   ya vienen con los valores de `AVANCE_Y_GIRO_CALIBRADO.md`, pero
   confirmar en este robot específico (sección 1 de ese documento)
   antes de correr nada más.
2. **Avance recto de una celda (30 cm)** — probar `explorer_node`
   parado en un pasillo recto y medir con cinta métrica si se
   detiene donde corresponde.
3. **Giro de 90°** — igual, medir con escuadra.
4. **Orientación del LiDAR** (`front_offset_deg` / `invert_left_right`)
   — usar `lidar_viz.py` si está disponible, o verificar con
   `ros2 topic echo /scan` que la distancia mínima baja al acercar un
   objeto al lado físico correcto.
5. **`umbral_pared_m`** — con el robot detenido en una celda con pared
   real de un solo lado, confirmar que solo esa dirección se marca
   como pared.
6. Recién ahí correr la Ronda 1 completa en la pista armada según
   `pistas/pista_ejemplo.txt`.

---

## 6. Validado offline antes de tocar el robot

La lógica de `maze_model.py` + `robot_state.py` (flood fill, fase B,
BFS, compresión de tramos) se probó con un harness que simula el
sensado leyendo `pistas/pista_ejemplo.txt` como si fuera el LiDAR real
— confirma que encuentra la meta, que la fase B garantiza la ruta
óptima (18 movimientos, misma ruta que ya daba `main.py`), y que la
ruta no choca contra ningún muro real. Esto **no reemplaza** probar en
el robot real (la parte de sensado real por umbral de distancia y las
primitivas de avance/giro por odometría no se pueden probar sin el
hardware), pero da confianza de que la parte de decisión/algoritmo
está bien portada.
