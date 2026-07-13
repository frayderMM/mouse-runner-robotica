"""Estado de celda/heading por conteo de movimientos (igual criterio
que ``grid_map.py`` del proyecto original: no hay localizacion
absoluta, se estima por avances y giros ejecutados) + declaracion y
lectura de los parametros ROS2 compartidos entre ``explorer_node`` y
``speedrun_node`` (para no duplicar la misma lista larga dos veces).

Convencion de coordenadas: igual que ``maze_model.py`` / pista_ejemplo.txt
-- (0,0) = A1 = esquina inferior izquierda, columnas A..L crecen al
ESTE, filas 1..8 crecen al NORTE.
"""

import math
from dataclasses import dataclass

from .maze_model import DIRS, celda, nombre

# Publicas (sin guion bajo) a proposito: lidar.py las reusa para
# direcciones_absolutas() en vez de mantener su propia copia -- un
# solo lugar para la convencion de rotacion N/E/S/O.
GIRO_DERECHA = {"N": "E", "E": "S", "S": "O", "O": "N"}
GIRO_IZQUIERDA = {"N": "O", "O": "S", "S": "E", "E": "N"}
GIRO_180 = {"N": "S", "S": "N", "E": "O", "O": "E"}


@dataclass
class GridPose:
    col: int
    row: int
    heading: str  # 'N' | 'E' | 'S' | 'O'

    @classmethod
    def from_cell_name(cls, name: str, heading: str):
        col, row = celda(name)
        return cls(col=col, row=row, heading=heading)

    @property
    def cell(self):
        return (self.col, self.row)

    @property
    def cell_name(self) -> str:
        return nombre(self.cell)

    def avanzar(self):
        dx, dy = DIRS[self.heading]
        self.col += dx
        self.row += dy

    def girar(self, lado: str):
        """lado in {'DERECHA', 'IZQUIERDA', 'ATRAS'}."""
        if lado == 'DERECHA':
            self.heading = GIRO_DERECHA[self.heading]
        elif lado == 'IZQUIERDA':
            self.heading = GIRO_IZQUIERDA[self.heading]
        elif lado == 'ATRAS':
            self.heading = GIRO_180[self.heading]
        else:
            raise ValueError(f'lado de giro desconocido: {lado}')


def lado_para_girar(heading_actual: str, direccion_deseada: str) -> str:
    """'NINGUNO' | 'DERECHA' | 'IZQUIERDA' | 'ATRAS' -- cuanto hay que
    girar para pasar de heading_actual a direccion_deseada."""
    if heading_actual == direccion_deseada:
        return 'NINGUNO'
    if GIRO_DERECHA[heading_actual] == direccion_deseada:
        return 'DERECHA'
    if GIRO_IZQUIERDA[heading_actual] == direccion_deseada:
        return 'IZQUIERDA'
    return 'ATRAS'


# ----------------------------------------------------------------------
# Parametros compartidos (calibracion tomada de AVANCE_Y_GIRO_CALIBRADO.md
# y CALIBRACION_LIDAR_VISION.md -- ver esos documentos para el detalle y
# el procedimiento de calibracion en pista).
# ----------------------------------------------------------------------

PARAMETROS_DEFAULT = {
    'scan_topic': '/scan',
    'odom_topic': '/odom_raw',
    'cmd_vel_topic': '/cmd_vel',
    # Buzzer: std_msgs/UInt16 con la duracion en ms -- confirmado en
    # pista en el proyecto original (`ros2 topic info /beep -v`, lo
    # suscribe el driver base YB_Car_Node).
    'buzzer_topic': '/beep',
    # Pitido suave y corto al decidir un giro (ver
    # motion.py::_emitir_pitido) -- mas corto que un pitido de alerta.
    'pitido_giro_ms': 120,
    # Telemetria (ver motion.py::_log_evento / _set_state): topico de
    # eventos de decision (String, formato "tipo|celda|heading|detalle")
    # y topico de estado actual de la maquina de estados (String).
    'event_topic': '/robot_event',
    'robot_state_topic': '/robot_state',
    'control_rate_hz': 20.0,

    # --- Pista (pistas/pista_ejemplo.txt) ---
    'celda_cm': 30.0,
    'num_columnas': 12,
    'num_filas': 8,
    'celda_inicio': 'A1',
    'celda_meta': 'L8',
    'heading_inicial': 'N',
    'orden_desempate': ['N', 'E', 'S', 'O'],
    'max_celdas_recorridas': 200,

    # --- LiDAR (CALIBRACION_LIDAR_VISION.md, seccion 1) ---
    'front_offset_deg': 180.0,
    'invert_left_right': True,
    'front_window_deg': [-15.0, 15.0],
    'right_window_deg': [-110.0, -70.0],
    'left_window_deg': [70.0, 110.0],
    'back_window_deg': [165.0, 195.0],
    'max_range_use_m': 4.0,
    # Umbral de "hay pared" al sensar una celda: celda_cm/2 (15cm para
    # celdas de 30cm, ver nota en README/FLUJO) + margen de seguridad.
    'umbral_pared_m': 0.20,
    # Validacion del sensado por consenso (ver motion.py::_tick_validacion_muros):
    # una sola lectura de LiDAR puede fallar por ruido o desalineacion,
    # asi que en vez de decidir con una sola muestra, se toman
    # sensado_muestras lecturas seguidas (con el robot quieto) y una
    # zona solo se confirma como pared si aparecio en al menos
    # sensado_consenso_minimo de ellas (mayoria, no una sola lectura
    # rara que contradiga el resto).
    'sensado_muestras': 5,
    'sensado_consenso_minimo': 3,

    # --- Avance y giro calibrados (AVANCE_Y_GIRO_CALIBRADO.md) ---
    'factor_dist_odom': 0.9474,
    'factor_ang_odom': 0.9899,
    'velocidad_recta_mps': 0.15,
    'margen_avance_m': 0.03,
    # Correccion durante el avance recto (ver motion.py::_correccion_recta):
    # 'ninguna' = avance ciego (sin correccion, comportamiento original);
    # 'angular_simple' = corrige error angular + deriva lateral respecto
    # a la linea recta inicial, con angular.z continuo.
    'tipo_correccion': 'angular_simple',
    'kp_angulo_recto': 2.2,
    'kp_lateral_recto': 1.1,
    'angular_max_correccion_radps': 0.3,
    'velocidad_giro_lineal_mps': 0.06,
    'velocidad_giro_angular_radps': 0.6,
    'angulo_giro_deg': 90.0,
    # Tope de seguridad EXPRESADO COMO MARGEN sobre el objetivo de cada
    # giro (no como angulo absoluto): un giro de 90 topea a 90+60=150,
    # uno de 180 (ATRAS) topea a 180+60=240 -- antes era un unico valor
    # absoluto (150) que quedaba POR DEBAJO del objetivo real de un
    # giro ATRAS (180), cortando todo giro de 180 en ~150 grados reales
    # aunque el modelo logico (GridPose) ya hubiera aplicado el giro
    # completo. Ver motion.py::_tick_giro.
    'margen_seguridad_giro_deg': 60.0,
    # Pequeno margen numerico (no de "aceptar giro corto"): un giro
    # ATRAS objetivo exactamente 180 esta justo en el punto donde
    # angle_diff() da la vuelta (rango (-180, 180]), asi que se apunta
    # a un poco menos para evitar quedar oscilando justo en ese punto
    # por precision de punto flotante. NO se usa para restar del
    # objetivo de los giros de 90 (eso era el bug: todo giro quedaba
    # sistematicamente ~4 grados corto).
    'margen_singularidad_atras_deg': 4.0,
    'tiempo_pausa_antes_girar_s': 1.0,

    # --- Seguridad (identica a state_machine_node del proyecto original) ---
    'umbral_colision_m': 0.10,
    'tiempo_espera_obstaculo_s': 2.0,
    # Si el obstaculo al frente sigue bloqueando despues de esta
    # cantidad de ciclos de espera (cada uno de tiempo_espera_obstaculo_s),
    # se aborta la mision en vez de esperar para siempre en silencio.
    'max_intentos_obstaculo': 5,

    # --- Salida ---
    'mapa_salida': '~/capytown_resultados/mapa_descubierto.yaml',
}


def declarar_parametros(node, overrides=None):
    valores = dict(PARAMETROS_DEFAULT)
    if overrides:
        valores.update(overrides)
    for nombre_p, valor in valores.items():
        node.declare_parameter(nombre_p, valor)


class Parametros:
    """Lee todos los parametros compartidos de un nodo ya inicializado
    y los deja como atributos tipados, listos para usar."""

    def __init__(self, node):
        g = lambda n: node.get_parameter(n).value  # noqa: E731

        self.scan_topic = g('scan_topic')
        self.odom_topic = g('odom_topic')
        self.cmd_vel_topic = g('cmd_vel_topic')
        self.buzzer_topic = g('buzzer_topic')
        self.pitido_giro_ms = int(g('pitido_giro_ms'))
        self.event_topic = g('event_topic')
        self.robot_state_topic = g('robot_state_topic')
        self.control_rate_hz = float(g('control_rate_hz'))

        self.celda_cm = float(g('celda_cm'))
        self.num_columnas = int(g('num_columnas'))
        self.num_filas = int(g('num_filas'))
        self.celda_inicio = str(g('celda_inicio'))
        self.celda_meta = str(g('celda_meta'))
        self.heading_inicial = str(g('heading_inicial'))
        self.orden_desempate = list(g('orden_desempate'))
        self.max_celdas_recorridas = int(g('max_celdas_recorridas'))

        self.front_offset_deg = float(g('front_offset_deg'))
        self.invert_left_right = bool(g('invert_left_right'))
        self.ventanas_deg = {
            'front': list(g('front_window_deg')),
            'right': list(g('right_window_deg')),
            'left': list(g('left_window_deg')),
            'back': list(g('back_window_deg')),
        }
        self.max_range_use_m = float(g('max_range_use_m'))
        self.umbral_pared_m = float(g('umbral_pared_m'))
        self.sensado_muestras = int(g('sensado_muestras'))
        self.sensado_consenso_minimo = int(g('sensado_consenso_minimo'))

        self.factor_dist_odom = float(g('factor_dist_odom'))
        self.factor_ang_odom = float(g('factor_ang_odom'))
        self.velocidad_recta_mps = float(g('velocidad_recta_mps'))
        self.margen_avance_m = float(g('margen_avance_m'))
        self.tipo_correccion = str(g('tipo_correccion'))
        self.kp_angulo_recto = float(g('kp_angulo_recto'))
        self.kp_lateral_recto = float(g('kp_lateral_recto'))
        self.angular_max_correccion_radps = float(g('angular_max_correccion_radps'))
        self.velocidad_giro_lineal_mps = float(g('velocidad_giro_lineal_mps'))
        self.velocidad_giro_angular_radps = float(g('velocidad_giro_angular_radps'))
        self.angulo_giro_rad = math.radians(float(g('angulo_giro_deg')))
        self.margen_seguridad_giro_rad = math.radians(float(g('margen_seguridad_giro_deg')))
        self.margen_singularidad_atras_rad = math.radians(float(g('margen_singularidad_atras_deg')))
        self.tiempo_pausa_antes_girar_s = float(g('tiempo_pausa_antes_girar_s'))

        self.umbral_colision_m = float(g('umbral_colision_m'))
        self.tiempo_espera_obstaculo_s = float(g('tiempo_espera_obstaculo_s'))
        self.max_intentos_obstaculo = int(g('max_intentos_obstaculo'))

        self.mapa_salida = str(g('mapa_salida'))
