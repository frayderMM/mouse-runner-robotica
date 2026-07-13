#!/usr/bin/env python3
"""Emisor de datos para el dashboard web -- corre en el ROBOT, sirve
JSON liviano por HTTP en ``GET /data`` (con CORS). No dibuja nada: el
render pesado (canvas) ocurre en ``web/index.html``, abierto en la
laptop, que hace polling a ``http://<IP-del-robot>:8080/data``. Mismo
patron que ``visualizador_web.py`` del proyecto de referencia
(``capytown_g0_granprix``), pero self-contained para este paquete y
con los 4 paneles pedidos:

1. Mapa -- pista de referencia (``pista_ejemplo_referencia.json``,
   generada offline por ``simulador_pista``) con las celdas ya
   visitadas pintadas y la ultima decision marcada con flecha.
2. Decisiones -- tabla en vivo de los eventos de ``/robot_event``
   (las mismas filas que van al CSV, ver ``motion.py::_log_evento``).
3. LiDAR -- radar con el barrido crudo de ``/scan`` + las 4 distancias
   de zona (front/right/left/back).
4. Odometria -- trayectoria continua (x,y) de ``/odom_raw`` (ya
   corregida por factor_dist_odom/factor_ang_odom) + velocidad actual
   de ``/cmd_vel``.

No usa mensajes/parametros de ``robot_state.py`` (ese es para
explorer/speedrun, que mueven al robot) -- este nodo solo escucha,
asi que declara sus propios parametros, mas simple y desacoplado.
"""

import json
import math
import threading
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from . import lidar
from .geometry_utils import yaw_from_quaternion

TRAY_MAX_PTS = 3000
DECISIONES_MAX = 300
VENTANAS_ZONAS = {
    'front': [-15.0, 15.0], 'right': [-110.0, -70.0],
    'left': [70.0, 110.0], 'back': [165.0, 195.0],
}
MAPA_DEFAULT = {'cols': 12, 'rows': 8, 'inicio': 'A1', 'meta': 'L8', 'muros': []}


class WebDashboardNode(Node):

    def __init__(self):
        super().__init__('web_dashboard')
        self._declarar_parametros()
        self._leer_parametros()

        self.lock = threading.Lock()
        self.estado = 'INICIAR'
        self.celda_actual = self.celda_inicio
        self.heading_actual = self.heading_inicial
        self.visitadas = {self.celda_inicio}
        self.decisiones = deque(maxlen=DECISIONES_MAX)
        self.ultima_decision = None
        # Se guarda aparte del deque (que se trunca a DECISIONES_MAX en
        # snapshot()) para que el "tiempo desde el inicio" en el
        # frontend no se recalcule mal si el operador conecta/reconecta
        # despues de que ya se acumularon mas de 100 eventos.
        self.t_inicio = None

        self.zonas = {z: math.inf for z in VENTANAS_ZONAS}
        self.lidar_pts = []

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.yaw = 0.0
        self.traj = deque(maxlen=TRAY_MAX_PTS)

        self.v_cmd = 0.0
        self.w_cmd = 0.0

        self.mapa = self._cargar_mapa_referencia()

        self.create_subscription(
            LaserScan, self.scan_topic, self._on_scan, QoSPresetProfiles.SENSOR_DATA.value
        )
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)
        self.create_subscription(Twist, self.cmd_vel_topic, self._on_cmd_vel, 10)
        self.create_subscription(String, self.event_topic, self._on_event, 10)
        self.create_subscription(String, self.robot_state_topic, self._on_robot_state, 10)

        self._arrancar_http()
        self.get_logger().info(
            f'web_dashboard activo en el puerto {self.puerto} -- '
            f'abrir web/index.html en la laptop y apuntar a la IP del robot'
        )

    # ------------------------------------------------------------------
    # Parametros (self-contained, no comparte robot_state.py con
    # explorer/speedrun -- este nodo solo escucha, no mueve al robot)
    # ------------------------------------------------------------------
    def _declarar_parametros(self):
        defaults = {
            'puerto': 8080,
            'scan_topic': '/scan',
            'odom_topic': '/odom_raw',
            'cmd_vel_topic': '/cmd_vel',
            'event_topic': '/robot_event',
            'robot_state_topic': '/robot_state',
            'front_offset_deg': 180.0,
            'invert_left_right': True,
            'max_range_use_m': 4.0,
            'factor_dist_odom': 0.9474,
            'factor_ang_odom': 0.9899,
            'celda_inicio': 'A1',
            'heading_inicial': 'N',
            # Ruta al JSON de la pista de referencia (generado offline
            # por simulador_pista, ver config/pista_ejemplo_referencia.json).
            # El launch file la resuelve al path instalado por defecto.
            'mapa_referencia': '',
        }
        for nombre_p, valor in defaults.items():
            self.declare_parameter(nombre_p, valor)

    def _leer_parametros(self):
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.puerto = int(g('puerto'))
        self.scan_topic = g('scan_topic')
        self.odom_topic = g('odom_topic')
        self.cmd_vel_topic = g('cmd_vel_topic')
        self.event_topic = g('event_topic')
        self.robot_state_topic = g('robot_state_topic')
        self.front_offset_deg = float(g('front_offset_deg'))
        self.invert_left_right = bool(g('invert_left_right'))
        self.max_range_use_m = float(g('max_range_use_m'))
        self.factor_dist_odom = float(g('factor_dist_odom'))
        self.factor_ang_odom = float(g('factor_ang_odom'))
        self.celda_inicio = str(g('celda_inicio'))
        self.heading_inicial = str(g('heading_inicial'))
        self.mapa_referencia_path = str(g('mapa_referencia'))

    def _cargar_mapa_referencia(self) -> dict:
        if not self.mapa_referencia_path:
            self.get_logger().warn('mapa_referencia vacio -- el panel de mapa queda sin paredes')
            return dict(MAPA_DEFAULT)
        try:
            with open(self.mapa_referencia_path, encoding='utf-8') as f:
                return json.load(f)
        except (OSError, json.JSONDecodeError) as e:
            # JSONDecodeError ademas de OSError: un JSON truncado/mal
            # editado (ej. a mitad de un git pull) no debe tumbar todo
            # el nodo al arrancar, igual que un archivo faltante.
            self.get_logger().warn(f'no se pudo cargar mapa_referencia ({self.mapa_referencia_path}): {e}')
            return dict(MAPA_DEFAULT)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _on_scan(self, msg: LaserScan):
        zonas = lidar.calcular_zonas(
            msg, self.front_offset_deg, self.invert_left_right, VENTANAS_ZONAS, self.max_range_use_m
        )
        pts = lidar.puntos_robot(msg, self.front_offset_deg, self.invert_left_right, self.max_range_use_m)
        with self.lock:
            self.zonas = zonas
            self.lidar_pts = pts

    def _on_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x * self.factor_dist_odom
        y = msg.pose.pose.position.y * self.factor_dist_odom
        yaw = yaw_from_quaternion(msg.pose.pose.orientation) * self.factor_ang_odom
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(yaw)):
            return  # muestra de odometria corrupta -- no la guardamos ni la mostramos
        with self.lock:
            self.odom_x, self.odom_y, self.yaw = x, y, yaw
            # Decimado simple: solo agrega si se movio >= 1cm desde el
            # ultimo punto guardado (evita miles de puntos casi iguales
            # mientras el robot esta detenido/pausado).
            if not self.traj or math.hypot(x - self.traj[-1][0], y - self.traj[-1][1]) >= 0.01:
                self.traj.append([round(x, 3), round(y, 3)])

    def _on_cmd_vel(self, msg: Twist):
        with self.lock:
            self.v_cmd = msg.linear.x
            self.w_cmd = msg.angular.z

    def _on_robot_state(self, msg: String):
        with self.lock:
            self.estado = msg.data

    def _on_event(self, msg: String):
        """Parsea el formato de motion.py::_log_evento:
        "tipo|celda|heading|clave1=valor1;clave2=valor2;..."."""
        partes = msg.data.split('|', 3)
        if len(partes) < 4:
            return
        tipo, celda, heading, detalle_txt = partes
        detalle = {}
        for par in detalle_txt.split(';'):
            if '=' in par:
                k, v = par.split('=', 1)
                detalle[k] = v

        t = self.get_clock().now().nanoseconds / 1e9
        with self.lock:
            if self.t_inicio is None:
                self.t_inicio = t
            self.celda_actual = celda
            self.heading_actual = heading
            self.visitadas.add(celda)
            self.decisiones.append({
                't': t, 'tipo': tipo, 'celda': celda, 'heading': heading, 'detalle': detalle,
            })
            if tipo in ('DECISION', 'DECISION_SEGMENTO'):
                self.ultima_decision = {
                    'celda': celda,
                    'elegida': detalle.get('elegida') or detalle.get('dir'),
                    'hacia': detalle.get('hacia'),
                }

    # ------------------------------------------------------------------
    # Snapshot para el frontend
    # ------------------------------------------------------------------
    def snapshot(self) -> dict:
        with self.lock:
            return {
                'estado': self.estado,
                'celda_actual': self.celda_actual,
                'heading_actual': self.heading_actual,
                'visitadas': sorted(self.visitadas),
                'ultima_decision': self.ultima_decision,
                't_inicio': self.t_inicio,
                'decisiones': list(self.decisiones)[-100:],
                'zonas': {k: (v if math.isfinite(v) else None) for k, v in self.zonas.items()},
                'lidar': self.lidar_pts,
                # isfinite: si /odom_raw publica alguna vez NaN/Infinity
                # (glitch del driver), json.dumps generaria el token
                # literal "NaN"/"Infinity" (JSON invalido) y el
                # fetch(...).json() del navegador rompe TODO el
                # payload, no solo este panel -- mejor null que tumbar
                # el dashboard entero.
                'pos': [round(self.odom_x, 3), round(self.odom_y, 3)]
                       if math.isfinite(self.odom_x) and math.isfinite(self.odom_y) else None,
                'yaw_deg': round(math.degrees(self.yaw), 1) if math.isfinite(self.yaw) else None,
                'traj': [p for p in self.traj if math.isfinite(p[0]) and math.isfinite(p[1])],
                'v_cmd': round(self.v_cmd, 3),
                'w_cmd': round(self.w_cmd, 3),
                'mapa': self.mapa,
            }

    # ------------------------------------------------------------------
    # Servidor HTTP (solo /data, con CORS) -- mismo patron liviano que
    # visualizador_web.py del proyecto de referencia.
    # ------------------------------------------------------------------
    def _arrancar_http(self):
        nodo = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass  # silenciar el log de cada request

            def _cors(self):
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Cache-Control', 'no-store')

            def do_GET(self):
                if self.path.startswith('/data'):
                    cuerpo = json.dumps(nodo.snapshot()).encode('utf-8')
                    content_type = 'application/json'
                else:
                    cuerpo = (b'web_dashboard_node activo. El frontend esta en la '
                              b'laptop (web/index.html). Datos en /data')
                    content_type = 'text/plain; charset=utf-8'
                self.send_response(200)
                self.send_header('Content-Type', content_type)
                self._cors()
                self.send_header('Content-Length', str(len(cuerpo)))
                self.end_headers()
                self.wfile.write(cuerpo)

        try:
            self._srv = ThreadingHTTPServer(('0.0.0.0', self.puerto), Handler)
        except OSError as e:
            # Puerto ya en uso (ej. un web_dashboard anterior que no
            # cerro bien tras un Ctrl+C) -- no tumbar el nodo entero
            # por esto, solo avisar y seguir sin servidor HTTP (las
            # suscripciones ROS2 igual quedan activas).
            self.get_logger().error(
                f'no se pudo abrir el puerto {self.puerto} ({e}) -- '
                f'el dashboard no va a responder /data, pero el nodo sigue vivo'
            )
            self._srv = None
            return
        threading.Thread(target=self._srv.serve_forever, daemon=True).start()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WebDashboardNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
