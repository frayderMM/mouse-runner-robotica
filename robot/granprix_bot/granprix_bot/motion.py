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

import csv
import datetime
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, UInt16

from . import lidar
from .geometry_utils import angle_diff, normalize_angle, yaw_from_quaternion
from .robot_state import GIRO_180, GIRO_DERECHA, GIRO_IZQUIERDA, GridPose, Parametros, declarar_parametros

# Offset fijo (grados) del yaw absoluto de cada heading logico respecto
# al yaw de referencia de la mision (heading N = 0, el resto siguen el
# sentido de giro ya establecido: DERECHA resta 90 -- N->E->S->O->N).
_OFFSET_HEADING_DEG = {'N': 0.0, 'E': -90.0, 'S': 180.0, 'O': 90.0}
_DESTINO_GIRO = {'DERECHA': GIRO_DERECHA, 'IZQUIERDA': GIRO_IZQUIERDA, 'ATRAS': GIRO_180}


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
        # Yaw "cero" de la mision: se fija UNA sola vez, con la primera
        # lectura de /odom_raw (ver _on_odom) -- todos los objetivos de
        # AVANZAR y GIRAR se miden desde aca (mas los 90/180/270 grados
        # fijos de _OFFSET_HEADING_DEG segun el heading logico), no
        # desde el yaw que el robot tenia justo antes de CADA accion.
        # Asi un error de un tramo/giro no se convierte en el "cero"
        # del siguiente, evitando que se acumule sin limite. Si el
        # robot arranca la mision desalineado (no apuntando derecho al
        # pasillo), ese error queda fijo toda la corrida -- ver
        # README.md seccion de calibracion.
        self._yaw_referencia_mision = None

        self._avance_inicio_xy = (0.0, 0.0)
        self._yaw_inicio_giro = 0.0
        self._pausa_inicio = None
        self._muros_conteo = {}
        self._muros_muestras_tomadas = 0

        self._esperando_obstaculo = False
        self._espera_obstaculo_inicio = None
        self._intentos_obstaculo = 0
        self.obstaculo_abandonado = False

        self._cmd_pub = self.create_publisher(Twist, self.p.cmd_vel_topic, 10)
        self._buzzer_pub = self.create_publisher(UInt16, self.p.buzzer_topic, 10)
        self._event_pub = self.create_publisher(String, self.p.event_topic, 10)
        self._robot_state_pub = self.create_publisher(String, self.p.robot_state_topic, 10)
        self._csv_path = self._preparar_csv(nombre_nodo)

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
        if self._yaw_referencia_mision is None:
            # Primera lectura de odometria: define "heading N" para
            # toda la mision. Si el robot no esta bien alineado con el
            # pasillo en este instante, ese desalineamiento queda fijo
            # el resto de la corrida -- alinearlo a mano ANTES de
            # lanzar el nodo (ver README.md).
            self._yaw_referencia_mision = self._yaw

    def _yaw_objetivo(self, heading: str) -> float:
        """Yaw absoluto (radianes) que le corresponde al heading logico
        dado (N/E/S/O), fijo desde el arranque de la mision -- ver
        `_yaw_referencia_mision`."""
        if self._yaw_referencia_mision is None:
            return self._yaw  # todavia no llego ninguna lectura de odometria
        return normalize_angle(
            self._yaw_referencia_mision + math.radians(_OFFSET_HEADING_DEG[heading])
        )

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
        norte/este/sur/oeste. Lectura de UNA sola muestra -- para
        sensar una celda de verdad usar `_iniciar_validacion_muros` /
        `_tick_validacion_muros` (varias muestras con consenso, mas
        resistente al ruido de una lectura suelta)."""
        zonas = self.leer_zonas()
        abs_dirs = lidar.direcciones_absolutas(self.pose.heading)
        muros = set()
        for zona_rel, dist in zonas.items():
            if dist < self.p.umbral_pared_m:
                muros.add(abs_dirs[zona_rel])
        return muros

    # ------------------------------------------------------------------
    # Validacion del sensado por consenso: en vez de decidir "hay
    # pared" con una sola lectura del LiDAR, toma sensado_muestras
    # lecturas seguidas (con el robot quieto) y una zona solo se
    # confirma como pared si aparecio en al menos
    # sensado_consenso_minimo de ellas -- una lectura rara aislada
    # (ruido, reflejo) no alcanza para contradecir el resto.
    # ------------------------------------------------------------------
    def _iniciar_validacion_muros(self):
        self._muros_conteo = {z: 0 for z in self.p.ventanas_deg}
        self._muros_muestras_tomadas = 0

    def _tick_validacion_muros(self) -> bool:
        """Toma una muestra mas y devuelve True cuando ya se tomaron
        todas las ``sensado_muestras`` pedidas."""
        zonas = self.leer_zonas()
        for zona_rel, dist in zonas.items():
            if dist < self.p.umbral_pared_m:
                self._muros_conteo[zona_rel] += 1
        self._muros_muestras_tomadas += 1
        self._publish_twist(Twist())  # quieto mientras valida
        return self._muros_muestras_tomadas >= self.p.sensado_muestras

    def _muros_confirmados(self) -> set:
        """Traduce a direcciones absolutas (N/E/S/O) las zonas que
        alcanzaron el consenso minimo tras `_tick_validacion_muros`."""
        abs_dirs = lidar.direcciones_absolutas(self.pose.heading)
        return {
            abs_dirs[zona_rel] for zona_rel, cuenta in self._muros_conteo.items()
            if cuenta >= self.p.sensado_consenso_minimo
        }

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
                    self._intentos_obstaculo += 1
                    self.get_logger().warn(
                        f'obstaculo sigue bloqueando tras {self._intentos_obstaculo} '
                        f'intento(s) de espera de {self.p.tiempo_espera_obstaculo_s:.0f}s'
                    )
                    if self._intentos_obstaculo >= self.p.max_intentos_obstaculo:
                        self.get_logger().error(
                            'obstaculo persistente -- abortando mision (obstaculo_abandonado=True)'
                        )
                        self.obstaculo_abandonado = True
                    self._espera_obstaculo_inicio = self.get_clock().now()
                return True
            self._esperando_obstaculo = False
            self._intentos_obstaculo = 0
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

    def _tick_avance(self, distancia_m: float = None) -> bool:
        """Publica el comando de avance y devuelve True cuando ya
        recorrio ``distancia_m`` (default: una celda, ``celda_cm``) O
        cuando el LiDAR frontal ya ve una pared real a
        ``distancia_frenado_pared_frontal_m`` o menos -- lo que pase
        primero. Usado tal cual por ``explorer_node`` (siempre una
        celda) y por ``speedrun_node`` (distancia_m = tramo completo
        comprimido de varias celdas, para recorrerlo de un tiron).

        La odometria sola no alcanza: friccion/deslizamiento de
        ruedas hace que a veces el avance real quede corto o largo
        respecto a lo que reporta el odometro. La pared frontal
        detectada por LiDAR es una referencia fisica mas confiable que
        la odometria acumulada -- si ya esta ahi, se toma el avance
        como terminado (se frena y se cuenta la celda como recorrida)
        sin importar cuanto marcaba el odometro en ese momento, en vez
        de seguir empujando contra la pared o quedarse corto por
        exceso de confianza en el odometro."""
        if distancia_m is None:
            distancia_m = self.p.celda_cm / 100.0

        dx = self._odom_x - self._avance_inicio_xy[0]
        dy = self._odom_y - self._avance_inicio_xy[1]
        avance = math.hypot(dx, dy)
        objetivo = distancia_m - self.p.margen_avance_m

        frente_m = self.leer_zonas()['front']
        pared_frontal_cerca = frente_m <= self.p.distancia_frenado_pared_frontal_m

        if avance >= objetivo or pared_frontal_cerca:
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
        respecto a la linea recta ideal para el heading logico actual
        -- a diferencia del avance ciego original (tipo_correccion=
        'ninguna', sin correccion).

        yaw0 es el yaw ABSOLUTO objetivo (`_yaw_objetivo`), NO el yaw
        que el robot tenia al arrancar ESTE tramo -- eso permitia que
        un heading ya torcido (por ejemplo, si el robot arranca la
        mision no bien alineado) se sostuviera para siempre en vez de
        corregirse, y que el error de un tramo se le sumara al
        siguiente. Con el objetivo absoluto fijo, el error no se
        acumula tramo a tramo ni giro a giro (aunque si el robot
        arranca la mision desalineado, ese error inicial sigue
        presente en TODA la corrida -- esto no lo corrige, hace falta
        alinearlo bien a mano antes de lanzar, o agregar correccion
        contra la pared real por LiDAR si no alcanza).

        error_angular: convencion "objetivo - actual" (angle_diff(yaw0,
        yaw_actual)), no "actual - objetivo" -- con kp_angulo POSITIVO,
        esta es la convencion que cierra el lazo (si el yaw actual se
        fue a la derecha del objetivo, error_angular>0 empuja angular.z
        positivo = gira a la izquierda, de vuelta hacia yaw0). Con el
        signo invertido (actual-objetivo) el lazo seria de
        realimentacion POSITIVA y divergiria -- mismo tipo de error
        que advierte CALIBRACION_LIDAR_VISION.md seccion 1.4 para
        right_line_angle_rad, verificar en pista igual que ahi.

        error_lateral: cuanto se alejo, perpendicular a esa linea, de
        la trayectoria recta ideal (proyeccion de (dx,dy) sobre el eje
        perpendicular al yaw objetivo, hacia la izquierda). Con
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

        yaw0 = self._yaw_objetivo(self.pose.heading)
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
        cuando el giro ya alcanzo su objetivo.

        El objetivo es un yaw ABSOLUTO (`_yaw_objetivo` del heading de
        destino, calculado a partir de `self.pose.heading` -- pose
        TODAVIA no se actualiza hasta que este metodo devuelve True),
        no un delta relativo al yaw que tenia el robot al empezar ESTE
        giro. Antes, si un giro quedaba unos grados de mas o de menos,
        ese error se convertia en el "cero" del siguiente tramo recto
        y del siguiente giro, acumulandose sin limite vuelta tras
        vuelta -- con el objetivo absoluto fijo, el error de un giro
        no contamina a los que siguen.

        `ya_giro_lo_suficiente` exige que el robot haya rotado
        realmente casi todo el angulo nominal (90/180) antes de
        aceptar "ya estoy cerca del objetivo absoluto" -- sin esto, si
        ya habia error acumulado previo que por casualidad dejaba el
        yaw actual cerca del objetivo de destino, el giro podria
        cortarse casi sin haber girado, dejando al robot fuera de
        posicion (el giro tambien tiene que desplazarlo por el arco,
        no solo rotarlo).

        El tope de seguridad (`margen_seguridad_giro_rad`) sigue
        siendo relativo a cuanto se giro REALMENTE (no al objetivo
        absoluto) -- por si el odometro se traba y el yaw absoluto
        nunca converge."""
        heading_destino = _DESTINO_GIRO[lado][self.pose.heading]
        objetivo_abs = self._yaw_objetivo(heading_destino)
        objetivo_relativo_nominal = math.pi if lado == 'ATRAS' else self.p.angulo_giro_rad
        izquierda = True if lado == 'ATRAS' else (lado == 'IZQUIERDA')

        angulo_girado = abs(angle_diff(self._yaw, self._yaw_inicio_giro))
        error_al_objetivo = abs(angle_diff(objetivo_abs, self._yaw))
        tope_seguridad_rad = objetivo_relativo_nominal + self.p.margen_seguridad_giro_rad

        cerca_del_objetivo = error_al_objetivo <= self.p.tolerancia_giro_absoluto_rad
        ya_giro_lo_suficiente = angulo_girado >= objetivo_relativo_nominal - self.p.margen_giro_minimo_rad

        if (cerca_del_objetivo and ya_giro_lo_suficiente) or angulo_girado >= tope_seguridad_rad:
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

    def _emitir_pitido(self, duracion_ms: int = None):
        """Publica en /beep (std_msgs/UInt16, duracion en ms) -- mismo
        topico/tipo que usaba el proyecto original para el buzzer del
        robot (confirmado en pista con ``ros2 topic info /beep -v``,
        lo suscribe el driver base ``YB_Car_Node``). Default:
        ``pitido_giro_ms`` (pitido suave, mas corto que el de META)."""
        if duracion_ms is None:
            duracion_ms = self.p.pitido_giro_ms
        self._buzzer_pub.publish(UInt16(data=int(duracion_ms)))

    # ------------------------------------------------------------------
    # Telemetria: feedback en vivo (topicos) + registro en CSV (post-analisis)
    # ------------------------------------------------------------------
    def _preparar_csv(self, nombre_nodo: str) -> str:
        """Un archivo nuevo por corrida (con fecha/hora en el nombre,
        no se pisa entre corridas) en ``~/capytown_resultados/`` --
        misma carpeta que ya usaba el proyecto original para
        resultados. Devuelve la ruta ya con el encabezado escrito."""
        carpeta = os.path.expanduser('~/capytown_resultados')
        os.makedirs(carpeta, exist_ok=True)
        marca = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        ruta = os.path.join(carpeta, f'eventos_{nombre_nodo}_{marca}.csv')
        with open(ruta, 'w', encoding='utf-8', newline='') as f:
            csv.writer(f).writerow([
                't_s', 'nodo', 'evento', 'estado', 'celda', 'heading',
                'odom_x_m', 'odom_y_m', 'yaw_deg', 'detalle',
            ])
        self.get_logger().info(f'log de eventos: {ruta}')
        return ruta

    def _log_evento(self, tipo: str, **detalle):
        """Registra un evento de decision/transicion en 3 lugares a la
        vez, para poder corregir/ajustar despues:
        1. Log de consola (``ros2 launch`` / ``ros2 run`` lo muestra en vivo).
        2. Topico ``/robot_event`` (String) -- ``ros2 topic echo
           /robot_event`` para verlo en vivo sin mirar la consola.
        3. Fila en el CSV de esta corrida -- para graficar/analizar
           despues (Excel, pandas, lo que sea).

        ``detalle`` son pares clave=valor libres segun el evento (ej.
        ``_log_evento('DECISION', fase='A', elegida='N', hacia='A2')``)."""
        t = self.get_clock().now().nanoseconds / 1e9
        detalle_txt = ';'.join(f'{k}={v}' for k, v in detalle.items())

        self.get_logger().info(f'[{tipo}] {self.pose.cell_name} ({self.pose.heading}) {detalle_txt}')
        self._event_pub.publish(String(data=f'{tipo}|{self.pose.cell_name}|{self.pose.heading}|{detalle_txt}'))

        with open(self._csv_path, 'a', encoding='utf-8', newline='') as f:
            csv.writer(f).writerow([
                f'{t:.3f}', self.get_name(), tipo, self._state,
                self.pose.cell_name, self.pose.heading,
                f'{self._odom_x:.3f}', f'{self._odom_y:.3f}',
                f'{math.degrees(self._yaw):.1f}', detalle_txt,
            ])

    def _set_state(self, nuevo_estado: str):
        """Compartido por explorer_node y speedrun_node -- ademas de
        guardar el estado, lo publica en ``/robot_state`` (String) para
        poder ver en vivo en que estado esta el robot sin mirar logs."""
        self._state = nuevo_estado
        self._robot_state_pub.publish(String(data=nuevo_estado))
        self.get_logger().debug(f'-> {nuevo_estado}')

    def _on_timer(self):
        raise NotImplementedError
