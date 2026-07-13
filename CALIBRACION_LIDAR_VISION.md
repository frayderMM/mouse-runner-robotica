# Calibración de LiDAR y visión (cámara)

Este documento junta la calibración de los dos sensores de percepción
del robot: el **LiDAR** (`lidar_processor_node`, zonas angulares y
distancia a pared) y la **cámara** (`stop_sign_detector_node`,
detección de PARE/META por color). No incluye lógica de control
(avance/giro) — ver `AVANCE_Y_GIRO_CALIBRADO.md` para eso.

---

## 1. LiDAR (`lidar_processor_node`)

Nodo puramente de traducción: lee `/scan` (LaserScan, LiDAR MS200) y
publica `/lidar_zones` con la distancia mínima por zona angular +
un ajuste de recta al lado derecho/izquierdo. **No decide nada** —
mantenerlo así facilita calibrar el montaje sin tocar la lógica de
control.

### 1.1 Orientación y sentido de montaje

El LiDAR puede estar montado con un offset de rotación y/o espejado
respecto al frame del robot. Se corrige con dos parámetros:

```yaml
lidar_processor:
  ros__parameters:
    front_offset_deg: 180.0
    invert_left_right: true
```

**Procedimiento (visual, con `lidar_viz.py`, raíz del repo, no
requiere `colcon build`):**

```bash
python3 lidar_viz.py
```

Dibuja en vivo los puntos crudos de `/scan` en el marco del robot
(frente arriba) con 4 sectores de color (verde=frente, rojo=derecha,
azul=izquierda, naranja=atrás) y la distancia mínima de cada uno.

1. Poner un objeto grande (caja/tablón, para evitar ambigüedad
   angular) en cada lado físico real del robot, uno a la vez, y
   confirmar en qué sector de color aparece la mancha de puntos.
   - Si aparece en el sector correcto → no tocar nada.
   - Si frente/atrás están cambiados → ajustar `front_offset_deg`
     (probar `180.0`).
   - Si izquierda/derecha quedan cambiadas → `invert_left_right: true`.
2. **No confiar en un solo lado a la vez.** Verificar izquierda y
   derecha por separado con un solo objeto genérico puede ser
   ambiguo (ruido del cuarto, objetos parecidos a ambos lados). La
   prueba confiable es poner **dos objetos con distancias distintas
   y reconocibles, uno en cada lado real, al mismo tiempo** (por
   ejemplo, algo a ~40 cm en la derecha real y algo a ~13 cm en la
   izquierda real) y comparar: `DERECHA` debe mostrar ~0.40 e
   `IZQUIERDA` ~0.13. Si salen cambiados, están invertidos.
3. Se puede probar sin editar el YAML todavía, pasando parámetros
   directo al script:
   ```bash
   python3 lidar_viz.py --ros-args -p front_offset_deg:=180.0 -p invert_left_right:=false
   ```
4. Confirmar con el nodo real ya compilado:
   ```bash
   ros2 run capytown_granprix lidar_processor_node
   ros2 topic echo /lidar_zones
   ```
   y verificar que `front`, `right`/`right_front`/`right_rear` y
   `left` bajan al acercar un objeto al lado físico correspondiente.

> **Diferencia estático vs. en movimiento (importante):** en este
> robot la prueba estática (robot detenido) dio `invert_left_right:
> false` como correcto, pero en movimiento real (con
> `wall_follower_node` corriendo) terminó siguiendo la pared
> izquierda en vez de la derecha, y se corrigió a
> `invert_left_right: true` por observación directa en movimiento.
> Si el comportamiento vuelve a verse invertido, repetir la prueba
> con el robot avanzando, no solo detenido.

### 1.2 Ventanas angulares (zonas de interés)

Ángulos en el marco del **robot** (0°=frente, +90°=izquierda,
−90°=derecha), aplicados **después** de `front_offset_deg` /
`invert_left_right`. Ajustar solo si el barrido del MS200 en este
robot no corresponde a estos valores tras aplicar el offset/inversión.

```yaml
front_window_deg: [-15.0, 15.0]        # cono frontal general
front_narrow_window_deg: [-5.0, 5.0]   # cono angosto, solo logica_dos_reglas
right_front_window_deg: [-75.0, -45.0] # S1, usado por ALINEAR
right_window_deg: [-110.0, -70.0]      # lado derecho (2 puntos)
right_rear_window_deg: [-135.0, -105.0]# S2, usado por ALINEAR
left_window_deg: [70.0, 110.0]         # lado izquierdo (2 puntos)
```

- `front_narrow_window_deg` es más angosto que `front_window_deg`
  porque un cono ancho puede agarrar una pared lateral vista en
  diagonal y confundirla con un obstáculo real al frente.
- `right_front`/`right_rear` (S1/S2) son los dos puntos que usa
  `ALINEAR` para comparar `right_front - right_rear` y corregir
  paralelismo tras un giro.

### 1.3 Ajuste de recta a la pared (método principal, `wall_follower`)

En vez de 2 puntos sueltos, se ajusta una recta por mínimos
cuadrados a **todos** los puntos del LiDAR dentro de la ventana
lateral — mucho más robusto al ruido, porque un solo punto malo pesa
poco entre decenas. Resultado publicado en `/lidar_zones`:
`right_line_angle_rad`, `right_line_distance_m`, `right_line_valid`
(y el espejo `left_line_*`).

```yaml
right_side_window_deg: [-110.0, -70.0]  # ventana para el ajuste derecho
left_side_window_deg: [70.0, 110.0]     # espejo, ajuste izquierdo
min_puntos_linea: 6
outlier_max_iter: 3      # rechazo iterativo de outliers (esquinas, etc.)
outlier_residuo_m: 0.03
right_wall_max_range_m: 0.50   # rango propio, corto, para no confundir
left_wall_max_range_m: 0.50    # con una pared lejana dentro de max_range_use_m
max_range_use_m: 4.0
```

- La ventana se angostó de −135°/−45° a −110°/−70°: una ventana más
  ancha alcanza más lejos hacia adelante/atrás y cerca de una esquina
  (pared perpendicular) puede agarrar puntos de esa otra pared,
  sesgando el ángulo hasta ~37° falsos (confirmado con una esquina
  real en `sim_local/`).
- `right_wall_max_range_m`/`left_wall_max_range_m` son mucho más
  cortos que `max_range_use_m` (4 m) a propósito: sin ese límite, el
  ajuste encuentra cualquier superficie dentro de 4 m (pared lejana,
  otro lado de un espacio abierto) y la reporta como "pared válida"
  aunque la pared realmente seguida (a ~12 cm) ya terminó.
- El rechazo iterativo de outliers (`outlier_max_iter`,
  `outlier_residuo_m`) es una segunda capa de defensa además de la
  ventana angosta: descarta puntos que no encajan bien en la recta
  (probablemente de otra superficie) y reajusta.

### 1.4 Nota de signo (si se tocan las ganancias que usan `right_line_angle_rad`)

Con la pared horizontal en el mundo y el robot con yaw θ respecto a
ella, `right_line_angle_rad ≈ -θ`. Para corregir θ→0 hace falta
`angular.z = +k·right_line_angle_rad`, **sin** signo negativo extra.
Con el signo invertido el lazo es de realimentación positiva y el
robot diverge (gira hasta ~90° y se sale del pasillo) en menos de 1
segundo — verificado con `sim_local/` antes de tocar el robot real.

---

## 2. Cámara — visión de PARE/META (`stop_sign_detector_node`)

Detecta señales por **color** (HSV): rojo para PARE, verde para
META. Publica dos booleanos continuos (`/pare_detectado`,
`/meta_detectado`) — true mientras la señal está confirmada en el
campo de visión; la lógica de "detenerse 3s" o "avanzar y terminar"
vive en `state_machine_node`, no aquí.

### 2.1 Región de interés (ROI) de la imagen

```yaml
roi_y_min_frac: 0.0
roi_y_max_frac: 1.0
```

Recorta el frame en `y` antes de procesar (fracción de la altura,
0.0-1.0). Por defecto usa el frame completo; útil para ignorar el
techo o el piso si la cámara ve zonas donde nunca puede aparecer un
cartel.

### 2.2 Detección de PARE (rojo)

```yaml
rango1_min: [0, 120, 70]
rango1_max: [10, 255, 255]
rango2_min: [170, 120, 70]
rango2_max: [180, 255, 255]

area_minima_px: 800.0
area_maxima_px: 60000.0
relacion_aspecto_min: 0.6
relacion_aspecto_max: 1.4
solidez_minima: 0.75
banda_central_frac: 0.20
frames_confirmacion: 3
frames_perdida: 5
```

- El **hue del rojo cruza 0/179** en el espacio HSV de OpenCV, por
  eso se usan dos rangos que se combinan con OR.
- **Solidez** = área del blob / área de su convex hull (~1 para un
  cartel compacto y sólido). Descarta reflejos alargados o ruido
  disperso que no tienen forma de cartel.
- **Banda central** (`banda_central_frac`): el PARE solo cuenta si su
  centro cae dentro de `banda_central_frac * ancho_frame` respecto al
  centro horizontal de la imagen — un cartel visto de refilón a un
  costado (que el robot no está mirando de frente) se ignora. La
  META **no** exige esto (puede aparecer más al costado según por
  dónde entra el robot al cartel final).
- **Confirmación por histéresis** (`frames_confirmacion` /
  `frames_perdida`): exige varios frames consecutivos con detección
  antes de marcar `confirmado=true`, y varios frames consecutivos sin
  detección antes de volver a `false` — evita parpadeos de un solo
  frame por ruido. Más alto = menos falsos positivos, más lento para
  reaccionar.

### 2.3 Detección de META (verde)

```yaml
meta_rango_min: [35, 40, 60]
meta_rango_max: [95, 255, 255]
meta_area_minima_px: 600.0
meta_area_maxima_px: 150000.0
meta_aspecto_min: 0.3
meta_aspecto_max: 3.0
meta_solidez_minima: 0.35
meta_frames_confirmacion: 3
meta_frames_perdida: 5
```

- Verde **opaco/apagado**: un solo rango de hue (no cruza el 0 como
  el rojo).
- `s_min` (segundo valor de `meta_rango_min`) más bajo que el rojo
  para agarrar un verde poco saturado.
- No exige banda central (ver nota arriba), y la tolerancia de
  aspecto/solidez es más laxa que PARE porque el cartel META puede
  verse más deformado según el ángulo de llegada.

### 2.4 Procedimiento de calibración

1. Colocar la señal de PARE (roja) frente a la cámara y correr:
   ```bash
   ros2 run capytown_granprix stop_sign_detector_node
   ros2 topic echo /pare_detectado
   ```
2. Si no detecta o detecta de más, ajustar `rango1_min/max`,
   `rango2_min/max` (segmentación HSV), `area_minima_px`,
   `area_maxima_px` y la relación de aspecto.
3. Activar `publicar_debug: true` y ver `/pare_detectado/debug_image`
   en RViz o `rqt_image_view` (el robot tiene TigerVNC con entorno
   gráfico) para ver el recuadro detectado en vivo — verde si está
   confirmado, naranja si hay un blob candidato aún sin confirmar.
4. Ajustar `frames_confirmacion` (más alto = menos falsos positivos,
   más lento para reaccionar) y `frames_perdida`.
5. Repetir el mismo procedimiento para META, con `meta_rango_min/max`
   y los parámetros `meta_*` equivalentes.

---

## 3. Orden recomendado de calibración

1. LiDAR — orientación de montaje (sección 1.1), primero y con el
   robot detenido y luego en movimiento.
2. LiDAR — ventanas angulares y ajuste de recta (secciones 1.2-1.3),
   solo si los valores por defecto no corresponden a este robot.
3. Cámara — PARE (sección 2.2), con el cartel real de la pista bajo
   la iluminación real (la luz ambiente cambia mucho la segmentación
   HSV).
4. Cámara — META (sección 2.3), mismo procedimiento con el cartel
   verde.
