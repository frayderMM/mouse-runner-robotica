# Ficha Técnica del Robot — Yahboom ROSMASTER R2 (Raspberry Pi 5)

Documento de referencia con las propiedades físicas y de hardware del robot,
independiente de cualquier proyecto o código. Sirve como punto de partida
para el Reto Final (nueva pista).

---

## 1. Identificación

| Propiedad | Valor |
|---|---|
| Modelo | Yahboom ROSMASTER R2 |
| Placa principal | Raspberry Pi 5 |
| Coprocesador | ESP32-S3 (microROS) vía placa de expansión |
| Chasis | Aleación de aluminio, estructura **Ackermann** (dirección tipo auto) |
| Sistema operativo / middleware | Ubuntu + ROS 2 Humble (dentro de contenedor Docker) |
| IP de red (modo actual) | `10.42.0.1` (usuario `root`) |

---

## 2. Chasis y movimiento

| Propiedad | Valor |
|---|---|
| Tipo de dirección | Ackermann (rueda interior y exterior giran en ángulos distintos) |
| Largo | 24 cm |
| Ancho | 16 cm |
| Llantas | Goma antideslizante tipo carreras (motor 520) |
| Velocidad máxima | 1.8 m/s |
| Barra anticolisión | Sí (parachoques delantero) |

### Motores (x2, tracción/dirección)

| Propiedad | Valor |
|---|---|
| Voltaje nominal | 12 V |
| Torque de arranque (stall) | 3.1 kgf·cm |
| Torque nominal | 2.2 kgf·cm |
| Velocidad antes de reducción | 11000 rpm |
| Relación de reducción | 1:19 |
| Velocidad después de reducción | ~550 ± 10 rpm |
| Potencia nominal | < 4 W |
| Corriente nominal / de arranque | 0.3 A / 3 A |
| Encoders | Sí, motores reductores con encoder (para odometría) |

---

## 3. Sensores

### LiDAR — Oradar/Yahboom MS200

| Propiedad | Valor |
|---|---|
| Tipo | TOF (time-of-flight), 360° |
| Rango | 0.03 m – 12 m |
| Resolución angular | 0.8° @ 10 Hz |
| Frecuencia de giro | 7–15 Hz (default 10 Hz) |
| Densidad de puntos | ~4500 puntos/segundo |
| Inmunidad a luz ambiental | Hasta 30,000 lux |
| Orientación del "frente" en este robot | Configurable por software (0° o 180° según montaje) |

### Cámara

| Propiedad | Valor |
|---|---|
| Tipo | Cámara con 2 grados de libertad (2DOF, pan-tilt) |
| Profundidad | Versión con cámara de profundidad disponible según kit |

### IMU

| Propiedad | Valor |
|---|---|
| Tipo | IMU de 6 ejes (acelerómetro + giroscopio) integrada en la placa de expansión ESP32 |

---

## 4. Alimentación

| Propiedad | Valor |
|---|---|
| Batería | Recargable, 7.4 V (Li-ion) |
| Alimentación de motores | 12 V (regulado desde batería) |

---

## 5. Control y conectividad

| Propiedad | Valor |
|---|---|
| Métodos de control soportados | App móvil, control remoto inalámbrico, teclado (teleop), ROS 2 |
| Middleware robótico | ROS 2 Humble |
| Comunicación placa-coprocesador | microROS sobre ESP32-S3 |
| Red | Wi-Fi (modo hotspot del robot, IP fija 10.42.0.1) |

---

## 6. Interfaz ROS 2 del robot (bringup / hardware)

Tópicos que expone el **stack base del robot** (driver + microROS), es decir,
lo que cualquier proyecto puede usar como entrada/salida — no son tópicos de
un proyecto en particular, sino la interfaz del hardware en sí:

| Tópico | Tipo | Origen | Dirección típica |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Driver LiDAR (MS200) | Publica el robot |
| `/odom_raw` | `nav_msgs/Odometry` | microROS (ESP32-S3, encoders), driver `yahboom_driver` | Publica el robot |
| `/cmd_vel` | `geometry_msgs/Twist` | — | Cualquier nodo escribe, el robot ejecuta |
| `/imu/data` | `sensor_msgs/Imu` | microROS (IMU 6 ejes) | Publica el robot *(nombre exacto a confirmar)* |
| `/camera/image_raw` (o similar) | `sensor_msgs/Image` | Driver de cámara | Publica el robot *(nombre exacto a confirmar)* |
| `/battery_state` | `sensor_msgs/BatteryState` | microROS | Publica el robot *(si el firmware lo expone)* |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | `robot_state_publisher` / driver | Publica el robot |

**Frames TF típicos:** `base_link` (marco del robot), `laser`/`laser_frame`
(marco del LiDAR), `odom` (marco de referencia fijo de odometría).

Paquete de arranque (bringup) usado en proyectos previos: `capytown_esan`
(`ros2 launch capytown_esan bringup.launch.py`) — es el que expone estos
tópicos antes de lanzar cualquier lógica propia.

**Comando para verificar en vivo** (siempre correr esto al llegar al robot,
la lista real puede variar según firmware/versión):
```bash
ros2 topic list
ros2 topic info /scan
ros2 topic info /odom_raw
```

> Solo `/scan`, `/odom_raw` y `/cmd_vel` están confirmados por uso directo en
> proyectos anteriores (RC2/RC3). Los demás (`/imu/data`, cámara,
> `/battery_state`) son los nombres típicos de un bringup ROSMASTER, pero
> deben confirmarse con `ros2 topic list` en este robot específico.

---

## Notas y verificación pendiente

- Los valores de motor, LiDAR y batería provienen de la ficha genérica del
  fabricante (Yahboom) para el modelo ROSMASTER R2 — **verificar en campo**
  cualquier valor crítico (velocidad máxima real, orientación del LiDAR,
  distancia mínima segura) antes de calibrar parámetros del Reto Final.
- Largo (24 cm) y ancho (16 cm) medidos directamente sobre el robot. Peso
  total y capacidad de carga no están documentados por el fabricante en la
  ficha pública consultada.
- El modelo exacto de LiDAR puede variar según la versión del kit adquirida
  (algunas versiones traen SLAM A1 / YDLidar en vez de MS200); este robot
  usa **MS200**, confirmado por el parámetro `lidar_front_deg` usado en
  proyectos anteriores.

**Fuentes:**
- https://category.yahboom.net/products/rosmaster-r2
- https://category.yahboom.net/products/ms200
- https://github.com/YahboomTechnology/ROSMASTER-R2
