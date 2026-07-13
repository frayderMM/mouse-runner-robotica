"""Nodo base con las primitivas de movimiento compartidas por
``explorer_node`` y ``speedrun_node``: avanzar una celda y girar 90/180,
ambos cerrados por odometria (``AVANCE_Y_GIRO_CALIBRADO.md``), mas la
lectura de LiDAR por zonas (``CALIBRACION_LIDAR_VISION.md``) y la
regla de seguridad de obstaculo al frente. Los estados de estas
primitivas son genericos (``_iniciar_avance`` / ``_tick_avance``,
``_iniciar_giro`` / ``_tick_giro``, ``_iniciar_pausa`` / ``_tick_pausa``)
para que la maquina de estados de alto nivel de cada nodo hijo solo se
preocupe de CUANDO llamarlas, no de COMO se ejecutan.

Chasis Ackermann: no puede rotar sobre su propio eje, por eso GIRAR es
un arco (avance lineal lento + velocidad angular maxima), no una
rotacion en el sitio -- igual que ``state_machine_node.py`` original.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from . import lidar
from .geometry_utils import angle_diff, yaw_from_quaternion
from .robot_state import GridPose, Parametros, declarar_parametros


class NodoRobotBase(Node):

    def __init__(self, nombre_nodo: str, overrides_params=None):
        super().__init__(nombre_nodo)
        declarar_parametros(self, overrides_params)
        self.p = Parametros(self)

        self.pose = GridPose.from_cell_name(self.p.celda_inicio, self.p.heading_inicial)

        self._odom_x = 0.0
        self._odom_y = 0.0
        self._yaw = 0.0
        self._odom_ready = False
        self._scan = None
        self._scan_ready = False

        self._avance_inicio_xy = (0.0, 0.0)
        self._yaw_inicio_avance = 0.0
        self._yaw_inicio_giro = 0.0
        self._pausa_inicio = None

        self._esperando_obstaculo = False
        self._espera_obstaculo_inicio = None

        self._cmd_pub = self.create_publisher(Twist, self.p.cmd_vel_topic, 10)

        self.create_subscription(Odometry, self.p.odom_topic, self._on_odom, 10)
        self.create_subscription(
            LaserScan, self.p.scan_topic, self._on_scan, QoSPresetProfiles.SENSOR_DATA.value
        )
        self.create_timer(1.0 / self.p.control_rate_hz, self._on_timer)

    # ------------------------------------------------------------------
    # Callbacks de sensores
    # ------------------------------------------------------------------
    def _on_odom(self, msg: Odometry):
        # Correccion de escala del odometro (AVANCE_Y_GIRO_CALIBRADO.md,
        # seccion 1): el ROSMASTER R2 sobreestima distancia y angulo de
        # forma consistente.
        self._odom_x = msg.pose.pose.position.x * self.p.factor_dist_odom
        self._odom_y = msg.pose.pose.position.y * self.p.factor_dist_odom
        self._yaw = yaw_from_quaternion(msg.pose.pose.orientation) * self.p.factor_ang_odom
        self._odom_ready = True

    def _on_scan(self, msg: LaserScan):
        self._scan = msg
        self._scan_ready = True

    @property
    def sensores_listos(self) -> bool:
        return self._odom_ready and self._scan_ready

    # ------------------------------------------------------------------
    # LiDAR por zonas (CALIBRACION_LIDAR_VISION.md)
    # ------------------------------------------------------------------
    def leer_zonas(self) -> dict:
        """{'front': m, 'right': m, 'left': m, 'back': m} relativas al
        frente actual del robot (no al heading absoluto)."""
        return lidar.calcular_zonas(
            self._scan, self.p.front_offset_deg, self.p.invert_left_right,
            self.p.ventanas_deg, self.p.max_range_use_m,
        )

    def leer_muros_celda_actual(self) -> set:
        """Traduce las zonas relativas del LiDAR a un set de direcciones
        absolutas (N/E/S/O) donde hay pared, usando el heading actual
        del robot para rotar frente/derecha/izquierda/atras a
        norte/este/sur/oeste."""
        zonas = self.leer_zonas()
        abs_dirs = lidar.direcciones_absolutas(self.pose.heading)
        muros = set()
        for zona_rel, dist in zonas.items():
            if dist < self.p.umbral_pared_m:
                muros.add(abs_dirs[zona_rel])
        return muros

    # ------------------------------------------------------------------
    # Seguridad: obstaculo al frente (activa en cualquier estado)
    # ------------------------------------------------------------------
    def _handle_obstaculo_frente(self) -> bool:
        zonas = self.leer_zonas()
        bloqueado = zonas['front'] < self.p.umbral_colision_m

        if self._esperando_obstaculo:
            if bloqueado:
                self._publish_twist(Twist())
                elapsed = (
                    self.get_clock().now() - self._espera_obstaculo_inicio
                ).nanoseconds / 1e9
                if elapsed >= self.p.tiempo_espera_obstaculo_s:
                    self._espera_obstaculo_inicio = self.get_clock().now()
                return True
            self._esperando_obstaculo = False
            return False

        if bloqueado:
            self._publish_twist(Twist())
            self.get_logger().warn(f'obstaculo a {zonas["front"]:.2f} m -- deteniendo y esperando')
            self._esperando_obstaculo = True
            self._espera_obstaculo_inicio = self.get_clock().now()
            return True

        return False

    # ------------------------------------------------------------------
    # Primitiva: AVANZAR una celda (cerrado por odometria; corrige
    # deriva angular/lateral en vivo si tipo_correccion='angular_simple')
    # ------------------------------------------------------------------
    def _iniciar_avance(self):
        self._avance_inicio_xy = (self._odom_x, self._odom_y)
        self._yaw_inicio_avance = self._yaw

    def _tick_avance(self, distancia_m: float = None) -> bool:
        """Publica el comando de avance y devuelve True cuando ya
        recorrio ``distancia_m`` (default: una celda, ``celda_cm``).
        Usado tal cual por ``explorer_node`` (siempre una celda) y por
        ``speedrun_node`` (distancia_m = tramo completo comprimido de
        varias celdas, para recorrerlo de un tiron)."""
        if distancia_m is None:
            distancia_m = self.p.celda_cm / 100.0

        dx = self._odom_x - self._avance_inicio_xy[0]
        dy = self._odom_y - self._avance_inicio_xy[1]
        avance = math.hypot(dx, dy)
        objetivo = distancia_m - self.p.margen_avance_m

        if avance >= objetivo:
            self._publish_twist(Twist())
            return True

        cmd = Twist()
        cmd.linear.x = self.p.velocidad_recta_mps
        cmd.angular.z = self._correccion_recta(dx, dy)
        self._publish_twist(cmd)
        return False

    def _correccion_recta(self, dx: float, dy: float) -> float:
        """tipo_correccion='angular_simple': corrige continuamente
        durante el avance, sumando error angular + error lateral
        respecto a la linea recta inicial (posicion y yaw guardados en
        ``_iniciar_avance``) -- a diferencia del avance ciego original
        (tipo_correccion='ninguna', sin correccion).

        error_angular: convencion "objetivo - actual" (angle_diff(yaw0,
        yaw_actual)), no "actual - objetivo" -- con kp_angulo POSITIVO,
        esta es la convencion que cierra el lazo (si el yaw actual se
        fue a la derecha del inicial, error_angular>0 empuja angular.z
        positivo = gira a la izquierda, de vuelta hacia yaw0). Con el
        signo invertido (actual-objetivo) el lazo seria de
        realimentacion POSITIVA y divergiria -- mismo tipo de error
        que advierte CALIBRACION_LIDAR_VISION.md seccion 1.4 para
        right_line_angle_rad, verificar en pista igual que ahi.

        error_lateral: cuanto se alejo, perpendicular a esa linea, de
        la trayectoria recta ideal (proyeccion de (dx,dy) sobre el eje
        perpendicular al yaw inicial, hacia la izquierda). Con
        kp_lateral RESTANDO: si se fue hacia la izquierda
        (error_lateral>0), angular.z se vuelve negativo (gira a la
        derecha) para volver al centro.

        correccion = kp_angulo*error_angular - kp_lateral*error_lateral,
        recortada a +/- angular_max_correccion_radps.

        Verificar siempre en un tramo largo que el robot vuelve a la
        linea recta, no que se aleja cada vez mas (si diverge, revisar
        el signo antes de subir las ganancias)."""
        if self.p.tipo_correccion != 'angular_simple':
            return 0.0

        yaw0 = self._yaw_inicio_avance
        error_angular = angle_diff(yaw0, self._yaw)
        error_lateral = dx * (-math.sin(yaw0)) + dy * math.cos(yaw0)

        correccion = (self.p.kp_angulo_recto * error_angular
                      - self.p.kp_lateral_recto * error_lateral)
        limite = self.p.angular_max_correccion_radps
        return max(-limite, min(limite, correccion))

    # ------------------------------------------------------------------
    # Primitiva: GIRAR (arco lento cerrado por yaw de odometria)
    # ------------------------------------------------------------------
    def _iniciar_giro(self):
        self._yaw_inicio_giro = self._yaw

    def _tick_giro(self, lado: str) -> bool:
        """lado in {'DERECHA', 'IZQUIERDA', 'ATRAS'}. Devuelve True
        cuando el giro objetivo (90 o 180) ya se completo."""
        if lado == 'ATRAS':
            objetivo_rad = math.pi
            izquierda = True  # convencion fija, ver nota en plan_executor_node.py
        else:
            objetivo_rad = self.p.angulo_giro_rad
            izquierda = (lado == 'IZQUIERDA')

        angulo_girado = abs(angle_diff(self._yaw, self._yaw_inicio_giro))

        if (angulo_girado >= objetivo_rad - self.p.tolerancia_giro_rad
                or angulo_girado >= self.p.angulo_maximo_giro_rad):
            self._publish_twist(Twist())
            return True

        cmd = Twist()
        cmd.linear.x = self.p.velocidad_giro_lineal_mps
        cmd.angular.z = self.p.velocidad_giro_angular_radps if izquierda else -self.p.velocidad_giro_angular_radps
        self._publish_twist(cmd)
        return False

    # ------------------------------------------------------------------
    # Primitiva: PAUSA fija (detenido N segundos)
    # ------------------------------------------------------------------
    def _iniciar_pausa(self):
        self._pausa_inicio = self.get_clock().now()

    def _tick_pausa(self, duracion_s: float) -> bool:
        self._publish_twist(Twist())
        elapsed = (self.get_clock().now() - self._pausa_inicio).nanoseconds / 1e9
        return elapsed >= duracion_s

    # ------------------------------------------------------------------
    def _publish_twist(self, cmd: Twist):
        self._cmd_pub.publish(cmd)

    def _on_timer(self):
        raise NotImplementedError
