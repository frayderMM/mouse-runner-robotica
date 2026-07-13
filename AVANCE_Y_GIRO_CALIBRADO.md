# Avance recto y giro calibrado

Este documento junta **solo** la lógica de movimiento base del robot:
avanzar en línea recta una distancia conocida y girar un ángulo
conocido (90°), ambos cerrados contra la **odometría** (`/odom_raw`),
sin seguimiento de pared ni corrección lateral por LiDAR. Es la
lógica que usan `AVANCE_GIRO_VACIO`, `AVANCE_META` y `GIRAR` en
`state_machine_node.py`.

Chasis: **Ackermann** (no puede rotar sobre su propio eje), por eso
un "giro" real es un arco de avance lento + dirección máxima, no una
rotación en el sitio.

---

## 1. Paso previo obligatorio: calibrar la escala del odómetro

El `/odom_raw` del ROSMASTER R2 sobreestima tanto distancia como
ángulo de forma **consistente** (no es ruido aleatorio, es un factor
de escala fijo). Si no se corrige esto primero, ni el avance recto ni
el giro de 90° van a dar resultados correctos aunque el control esté
bien ajustado.

### Procedimiento

1. Con el robot quieto, leer la pose:
   ```bash
   ros2 topic echo /odom_raw --once
   ```
2. Empujar el robot **a mano** una distancia real conocida en línea
   recta (por ejemplo 60 cm, medida con cinta métrica) y volver a
   leer. Calcular:
   ```text
   distancia_odom = sqrt((x2-x1)^2 + (y2-y1)^2)
   ```
3. Con el robot quieto de nuevo, anotar el quaternion de orientación,
   girarlo **a mano** un ángulo real conocido (90°, con una escuadra)
   sin trasladarlo, y volver a leer. El yaw (para un quaternion con
   x=y=0) es `yaw = 2*atan2(z, w)`; calcular:
   ```text
   angulo_odom = yaw2 - yaw1
   ```
4. Calcular los factores de corrección:
   ```text
   factor_dist_odom = distancia_real / distancia_odom
   factor_ang_odom  = angulo_real / angulo_odom
   ```
5. Poner esos valores en `granprix_params.yaml`, dentro de
   `state_machine`:
   ```yaml
   factor_dist_odom: 0.9474   # ejemplo: avance real 76 cm / odometro 78.3 cm
   factor_ang_odom: 0.9899    # ejemplo: giro real 90° / odometro 90.92°
   ```
6. Repetir la prueba 2-3 veces (avance y giro) para confirmar que el
   factor es estable; si varía mucho entre pruebas, sospechar de
   deslizamiento de ruedas más que de un error de escala fijo.

Estos factores se aplican una sola vez, apenas llega cada mensaje de
odometría (`_on_odom` en `state_machine_node.py`), así que tanto el
avance recto como el giro quedan corregidos automáticamente:

```python
def _on_odom(self, msg: Odometry):
    self._odom_x = msg.pose.pose.position.x * self._factor_dist_odom
    self._odom_y = msg.pose.pose.position.y * self._factor_dist_odom
    self._yaw = yaw_from_quaternion(msg.pose.pose.orientation) * self._factor_ang_odom
```

---

## 2. Avance recto calibrado (sin corrección lateral)

Patrón usado en `_handle_avance_giro_vacio` / `_handle_avance_meta`:
avanzar a velocidad fija, midiendo el desplazamiento real recorrido
con la odometría ya corregida (`_odom_x`, `_odom_y`), y detenerse
cuando se alcanza la distancia objetivo.

```python
def _handle_avance_recto(self):
    dx = self._odom_x - self._inicio_xy[0]
    dy = self._odom_y - self._inicio_xy[1]
    avance = math.hypot(dx, dy)

    if avance < self._distancia_objetivo:
        cmd = Twist()
        cmd.linear.x = self._velocidad_recta
        self._publish_twist(cmd)
        return

    self._publish_twist(Twist())  # llegó: se detiene
```

- `self._inicio_xy` se guarda **antes** de empezar a avanzar
  (`(self._odom_x, self._odom_y)` en ese instante).
- No hay corrección de ángulo ni de distancia lateral: es puramente
  "avanzar hasta haber recorrido X metros según odometría".

### Parámetros relevantes (`granprix_params.yaml`, sección `state_machine`)

| Parámetro | Valor | Uso |
|---|---|---|
| `velocidad_recta_mps` | `0.15` | Velocidad lineal fija durante el avance |
| `factor_dist_odom` | `0.9474` | Corrige la sobreestimación de distancia del odómetro |

---

## 3. Giro calibrado (90°, cerrado por yaw de odometría)

Patrón usado en `_handle_girar_dinamico`: gira un ángulo **fijo**
(90° por defecto) comparando el yaw actual contra el yaw guardado al
empezar el giro, con avance lineal lento + velocidad angular máxima
(arco, no rotación en el sitio). No depende del LiDAR en ningún
momento.

```python
def _handle_girar(self):
    angulo_girado = abs(angle_diff(self._yaw, self._yaw_inicio_giro))

    if angulo_girado >= self._angulo_giro_rad or angulo_girado >= self._angulo_maximo_giro_rad:
        self._publish_twist(Twist())  # completó el giro: se detiene
        return

    cmd = Twist()
    cmd.linear.x = self._v_giro_lineal
    cmd.angular.z = self._v_giro_angular if direccion == 'IZQUIERDA' else -self._v_giro_angular
    self._publish_twist(cmd)
```

- `self._yaw_inicio_giro` se guarda **antes** de empezar a girar
  (`self._yaw` en ese instante).
- `_angulo_maximo_giro_rad` es un tope de seguridad adicional (por si
  el odómetro se traba y nunca llega al objetivo).
- Para ATRAS (180°) se usa el mismo mecanismo, solo cambia el delta
  objetivo.

### Cálculo del ángulo objetivo

```python
def _compute_turn_target(self, yaw: float, direction: str) -> float:
    if direction == 'DERECHA':
        delta = -self._angulo_giro_rad
    elif direction == 'IZQUIERDA':
        delta = self._angulo_giro_rad
    elif direction == 'ATRAS':
        delta = math.pi
    return normalize_angle(yaw + delta)
```

### Parámetros relevantes (`granprix_params.yaml`, sección `state_machine`)

| Parámetro | Valor | Uso |
|---|---|---|
| `angulo_giro_deg` | `90.0` | Ángulo objetivo del giro (DERECHA/IZQUIERDA) |
| `angulo_maximo_giro_deg` | `150.0` | Tope de seguridad si el odómetro se traba |
| `velocidad_giro_lineal_mps` | `0.06` | Avance lineal durante el arco de giro |
| `velocidad_giro_angular_radps` | `0.6` | Velocidad angular durante el arco de giro (radio de giro = v/w) |
| `tolerancia_giro_deg` | `4.0` | Tolerancia usada en la variante de giro con lazo cerrado por error angular |
| `factor_ang_odom` | `0.9899` | Corrige la sobreestimación de ángulo del odómetro |

> Nota: `velocidad_giro_lineal_mps` y `velocidad_giro_angular_radps`
> determinan el radio del arco (`radio ≈ v / w`). Bajar la lineal o
> subir la angular cierra el radio (giro más "cerrado"), útil si hay
> poco espacio libre alrededor al girar.

---

## 4. Orden recomendado de calibración en pista

1. Calibrar `factor_dist_odom` y `factor_ang_odom` (sección 1) —
   sin esto, nada de lo siguiente es confiable.
2. Probar el avance recto solo (sección 2): mandar al robot a avanzar
   una distancia conocida (por ejemplo 60 cm) y medir con cinta
   métrica si se detiene donde corresponde. Ajustar `factor_dist_odom`
   si hay error sistemático.
3. Probar el giro solo (sección 3), en un espacio abierto: girar 90°
   y medir con una escuadra si terminó realmente a 90°. Ajustar
   `factor_ang_odom` si hay error sistemático.
4. Repetir cada prueba 2-3 veces para confirmar que el resultado es
   estable antes de dar por buena la calibración.
