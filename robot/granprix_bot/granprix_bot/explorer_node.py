#!/usr/bin/env python3
"""Intento 1 (exploracion) en el robot real -- puerto celda por celda
de ``simulador_pista/sim/explorer.py`` (flood fill + fase B de
verificacion), reemplazando el sensado "de mentira" (leer el mapa
completo) por sensado REAL con el LiDAR en cada celda.

Secuencia por celda (pedida explicitamente): llegar -> detenerse 1s
(``tiempo_pausa_antes_girar_s`` reusado como pausa de celda, ver YAML) ->
sensar con el LiDAR y registrar los muros nuevos -> decidir el
siguiente movimiento con flood fill -> girar si hace falta -> avanzar
30cm a la siguiente celda -> repetir.

Fase A: igual que el simulador, va directo a la meta con la mejor
opcion greedy de flood fill (distancia BFS a la meta sobre lo
conocido, optimista: una pared no sensada se asume abierta).

Fase B: al llegar a la meta, verifica que la ruta candidata (BFS sobre
lo conocido) pase 100% por celdas ya sensadas; si no, sigue explorando
las celdas pendientes de esa ruta. Cuando la fase B termina, la ruta
del intento 2 (speedrun) queda garantizada como optima -- exactamente
la misma logica que el simulador.

Al terminar, guarda el mapa descubierto en ``mapa_salida`` (YAML,
mismo formato 4-bits N/E/S/O que ``main.py --exportar`` del
simulador) para que ``speedrun_node`` lo cargue en el intento 2.
"""

import os

import rclpy
from geometry_msgs.msg import Twist

from .maze_model import Maze, celda, nombre, wall
from .motion import NodoRobotBase
from .robot_state import lado_para_girar

INF = 10 ** 9


class ExplorerNode(NodoRobotBase):

    def __init__(self):
        super().__init__('explorer')

        self.mz = Maze(self.p.num_columnas, self.p.num_filas,
                        start=celda(self.p.celda_inicio), goal=celda(self.p.celda_meta))
        self.conocidos = set()   # muros descubiertos (frozenset de celdas)
        self.sensadas = set()    # celdas cuyas 4 paredes ya son conocidas
        self.fase = 'A'
        self.num_celdas = 0
        self._direccion_elegida = None
        self._lado_giro_pendiente = None
        self._terminado = False

        self._state = 'INICIAR'
        self.get_logger().info(
            f'explorer listo: inicio={nombre(self.mz.start)} meta={nombre(self.mz.goal)} '
            f'pista {self.mz.cols}x{self.mz.rows} celdas de {self.p.celda_cm:.0f}cm'
        )

    # ------------------------------------------------------------------
    # Sensado (equivalente real de Explorer._sense_celda del simulador)
    # ------------------------------------------------------------------
    def _sensar_celda_actual(self):
        pos = self.pose.cell
        muros_detectados = self.leer_muros_celda_actual()
        nuevos = []
        for direccion, vecino in ((d, (pos[0] + dx, pos[1] + dy))
                                   for d, (dx, dy) in
                                   (('N', (0, 1)), ('E', (1, 0)), ('S', (0, -1)), ('O', (-1, 0)))):
            if direccion in muros_detectados and self.mz.dentro(vecino):
                w = wall(pos, vecino)
                if w not in self.conocidos:
                    self.conocidos.add(w)
                    nuevos.append(direccion)
        self.sensadas.add(pos)
        self.get_logger().info(
            f'sensada {nombre(pos)}: muros nuevos = {nuevos or "ninguno"} '
            f'({len(self.sensadas)}/{self.mz.cols * self.mz.rows} celdas)'
        )
        return nuevos

    # ------------------------------------------------------------------
    # Decision (identica a Explorer.paso del simulador)
    # ------------------------------------------------------------------
    def _paso(self, pos, targets):
        dist = self.mz.flood(targets, self.conocidos)
        opciones = []
        for n, d in self.mz.vecinos(pos):
            if wall(pos, n) not in self.conocidos:
                opciones.append((dist.get(n, INF), self.p.orden_desempate.index(d), d, n))
        opciones.sort()
        _, _, d_elegida, siguiente = opciones[0]
        return d_elegida, siguiente

    # ------------------------------------------------------------------
    # Ciclo de control
    # ------------------------------------------------------------------
    def _on_timer(self):
        if not self.sensores_listos:
            return
        if self._terminado:
            self._publish_twist_vacio()
            return
        if self._handle_obstaculo_frente():
            return

        handler = getattr(self, f'_handle_{self._state.lower()}', None)
        if handler is None:
            raise RuntimeError(f'estado desconocido: {self._state}')
        handler()

    def _handle_iniciar(self):
        self._sensar_celda_actual()
        self._iniciar_pausa()
        self._set_state('PAUSA_CELDA')

    def _handle_pausa_celda(self):
        if self._tick_pausa(self.p.tiempo_pausa_antes_girar_s):
            self._set_state('DECIDIR')

    def _handle_decidir(self):
        pos = self.pose.cell

        if self.fase == 'A' and pos == self.mz.goal:
            self.get_logger().info('FASE A completa: llego a la meta. Empieza FASE B (verificacion).')
            self.fase = 'B'

        if self.fase == 'A':
            targets = [self.mz.goal]
        else:
            candidata = self.mz.bfs(self.mz.start, self.mz.goal, walls=self.conocidos)
            pendientes = [c for c in candidata if c not in self.sensadas]
            if not pendientes:
                self._terminar()
                return
            targets = pendientes

        self.num_celdas += 1
        if self.num_celdas > self.p.max_celdas_recorridas:
            self.get_logger().error('limite de celdas recorridas alcanzado sin terminar -- deteniendo')
            self._terminar()
            return

        d_elegida, siguiente = self._paso(pos, targets)
        self._direccion_elegida = d_elegida

        lado = lado_para_girar(self.pose.heading, d_elegida)
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
        if self._tick_avance():
            self.pose.avanzar()
            self._sensar_celda_actual()
            self._iniciar_pausa()
            self._set_state('PAUSA_CELDA')

    def _terminar(self):
        self._publish_twist_vacio()
        self._terminado = True
        self._set_state('TERMINADO')
        self._guardar_mapa()
        self.get_logger().info(
            f'EXPLORACION TERMINADA: {self.num_celdas} movimientos, '
            f'{len(self.sensadas)}/{self.mz.cols * self.mz.rows} celdas sensadas, '
            f'mapa guardado en {self.p.mapa_salida}'
        )

    def _guardar_mapa(self):
        """Mismo formato 4-bits (bit0=N bit1=E bit2=S bit3=O) que
        ``main.py --exportar`` del simulador."""
        ruta = os.path.expanduser(self.p.mapa_salida)
        os.makedirs(os.path.dirname(ruta), exist_ok=True)
        lineas = [
            '# mapa descubierto (bit0=N bit1=E bit2=S bit3=O)',
            f'cols: {self.mz.cols}', f'rows: {self.mz.rows}',
            f'inicio: {nombre(self.mz.start)}', f'meta: {nombre(self.mz.goal)}', 'celdas:',
        ]
        for y in range(self.mz.rows - 1, -1, -1):
            fila = []
            for x in range(self.mz.cols):
                v = 0
                for bit, (dx, dy) in enumerate([(0, 1), (1, 0), (0, -1), (-1, 0)]):
                    n = (x + dx, y + dy)
                    if not self.mz.dentro(n) or wall((x, y), n) in self.conocidos:
                        v |= 1 << bit
                fila.append(str(v))
            lineas.append(f'  - [{", ".join(fila)}]   # fila {y + 1}')
        with open(ruta, 'w', encoding='utf-8') as f:
            f.write('\n'.join(lineas) + '\n')

    def _publish_twist_vacio(self):
        self._publish_twist(Twist())

    def _set_state(self, nuevo_estado: str):
        self._state = nuevo_estado
        self.get_logger().debug(f'-> {nuevo_estado}')


def main(args=None):
    rclpy.init(args=args)
    node = ExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
