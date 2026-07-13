#!/usr/bin/env python3
"""Intento 2 (speed run) en el robot real -- puerto de
``simulador_pista/sim/speedrun.py``: carga el mapa que dejo
``explorer_node`` (``mapa_entrada``, mismo formato 4-bits que genera
``_guardar_mapa`` ahi), calcula el camino minimo garantizado (BFS) y lo
comprime en tramos rectos (varias celdas seguidas = un solo AVANZAR
continuo, sin pausa entre celda y celda -- a diferencia de la
exploracion, aca el mapa ya se conoce, no hace falta sensar).

Antes de cada giro hay una pausa fija (mismo criterio que
``AVANCE_Y_GIRO_CALIBRADO.md``: separa visualmente el frenado del
inicio del arco). El giro en si es el mismo arco lento cerrado por yaw
de odometria (chasis Ackermann, no rota sobre su eje).
"""

import os

import rclpy
from geometry_msgs.msg import Twist

from .maze_model import Maze, celda, nombre, wall
from .motion import NodoRobotBase
from .robot_state import lado_para_girar

_DIR_POR_DELTA = {(0, 1): 'N', (1, 0): 'E', (0, -1): 'S', (-1, 0): 'O'}


class SpeedrunNode(NodoRobotBase):

    def __init__(self):
        super().__init__('speedrun', overrides_params={'mapa_entrada': '~/capytown_resultados/mapa_descubierto.yaml'})

        self.mz = Maze(self.p.num_columnas, self.p.num_filas,
                        start=celda(self.p.celda_inicio), goal=celda(self.p.celda_meta))
        self.mz.walls = self._cargar_mapa(str(self.get_parameter('mapa_entrada').value))

        ruta = self.mz.bfs(self.mz.start, self.mz.goal)
        if ruta is None:
            raise RuntimeError(
                f'no hay camino {nombre(self.mz.start)} -> {nombre(self.mz.goal)} '
                f'en el mapa cargado -- revisar mapa_entrada o volver a correr explorer_node'
            )
        self.segmentos = self._comprimir(ruta)
        self._seg_idx = 0
        self._lado_giro_pendiente = None
        self._terminado = False
        self._state = 'INICIAR'

        self.get_logger().info(
            f'speedrun listo: ruta de {len(ruta) - 1} movimientos en {len(self.segmentos)} tramos '
            f'({nombre(self.mz.start)} -> {nombre(self.mz.goal)})'
        )
        for i, s in enumerate(self.segmentos):
            self.get_logger().info(
                f'  tramo {i + 1}: {s["celdas"]} celdas hacia {s["dir"]} '
                f'({s["celdas"] * self.p.celda_cm:.0f} cm)'
            )

    # ------------------------------------------------------------------
    # Carga de mapa (formato 4-bits: bit0=N bit1=E bit2=S bit3=O)
    # ------------------------------------------------------------------
    def _cargar_mapa(self, ruta_param):
        ruta = os.path.expanduser(ruta_param)
        with open(ruta, encoding='utf-8') as f:
            lineas = [l for l in f if not l.lstrip().startswith('#')]

        cols = rows = None
        filas_bits = []
        for linea in lineas:
            s = linea.strip()
            if s.startswith('cols:'):
                cols = int(s.split(':', 1)[1])
            elif s.startswith('rows:'):
                rows = int(s.split(':', 1)[1])
            elif s.startswith('- ['):
                contenido = s[s.index('[') + 1: s.index(']')]
                filas_bits.append([int(v.strip()) for v in contenido.split(',')])

        if cols != self.p.num_columnas or rows != self.p.num_filas:
            self.get_logger().warn(
                f'mapa_entrada tiene {cols}x{rows} pero los parametros dicen '
                f'{self.p.num_columnas}x{self.p.num_filas} -- usando lo del mapa'
            )

        conocidos = set()
        deltas = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        for idx, fila in enumerate(filas_bits):
            y = rows - 1 - idx
            for x, v in enumerate(fila):
                for bit, (dx, dy) in enumerate(deltas):
                    if v & (1 << bit):
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < cols and 0 <= ny < rows:
                            conocidos.add(wall((x, y), (nx, ny)))
        return conocidos

    # ------------------------------------------------------------------
    # Compresion en tramos rectos (identica a speedrun.py::comprimir)
    # ------------------------------------------------------------------
    @staticmethod
    def _comprimir(ruta):
        segs = []
        i = 0
        while i < len(ruta) - 1:
            dx = ruta[i + 1][0] - ruta[i][0]
            dy = ruta[i + 1][1] - ruta[i][1]
            j = i
            while (j < len(ruta) - 1
                   and (ruta[j + 1][0] - ruta[j][0], ruta[j + 1][1] - ruta[j][1]) == (dx, dy)):
                j += 1
            segs.append({'dir': _DIR_POR_DELTA[(dx, dy)], 'celdas': j - i})
            i = j
        return segs

    # ------------------------------------------------------------------
    # Ciclo de control
    # ------------------------------------------------------------------
    def _on_timer(self):
        if not self.sensores_listos:
            return
        if self._terminado:
            self._publish_twist(Twist())
            return
        if self._handle_obstaculo_frente():
            return

        handler = getattr(self, f'_handle_{self._state.lower()}', None)
        if handler is None:
            raise RuntimeError(f'estado desconocido: {self._state}')
        handler()

    def _handle_iniciar(self):
        self._set_state('DECIDIR_SEGMENTO')

    def _handle_decidir_segmento(self):
        if self._seg_idx >= len(self.segmentos):
            self._terminar()
            return

        seg = self.segmentos[self._seg_idx]
        lado = lado_para_girar(self.pose.heading, seg['dir'])
        if lado == 'NINGUNO':
            self._iniciar_avance()
            self._set_state('AVANZAR')
        else:
            self._lado_giro_pendiente = lado
            self._iniciar_pausa()
            self._set_state('PAUSA_GIRO')

    def _handle_pausa_giro(self):
        if self._tick_pausa(self.p.tiempo_pausa_antes_girar_s):
            self._iniciar_giro()
            self._set_state('GIRAR')

    def _handle_girar(self):
        if self._tick_giro(self._lado_giro_pendiente):
            self.pose.girar(self._lado_giro_pendiente)
            self._iniciar_avance()
            self._set_state('AVANZAR')

    def _handle_avanzar(self):
        seg = self.segmentos[self._seg_idx]
        distancia_m = seg['celdas'] * self.p.celda_cm / 100.0
        if self._tick_avance(distancia_m):
            for _ in range(seg['celdas']):
                self.pose.avanzar()
            self.get_logger().info(f'tramo {self._seg_idx + 1}/{len(self.segmentos)} completo, en {self.pose.cell_name}')
            self._seg_idx += 1
            self._set_state('DECIDIR_SEGMENTO')

    def _terminar(self):
        self._publish_twist(Twist())
        self._terminado = True
        self._set_state('META')
        llego = self.pose.cell == self.mz.goal
        self.get_logger().info(
            f'SPEEDRUN TERMINADO en {self.pose.cell_name} '
            f'({"META alcanzada" if llego else "OJO: no coincide con la meta esperada"})'
        )

    def _set_state(self, nuevo_estado: str):
        self._state = nuevo_estado
        self.get_logger().debug(f'-> {nuevo_estado}')


def main(args=None):
    rclpy.init(args=args)
    node = SpeedrunNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
