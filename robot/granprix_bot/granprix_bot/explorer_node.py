#!/usr/bin/env python3
"""Intento 1 (exploracion) en el robot real -- puerto celda por celda
de ``simulador_pista/sim/explorer.py`` (flood fill + fase B de
verificacion), reemplazando el sensado "de mentira" (leer el mapa
completo) por sensado REAL con el LiDAR en cada celda.

Secuencia por celda (pedida explicitamente): llegar -> detenerse 1s
(``tiempo_pausa_antes_girar_s`` reusado como pausa de celda, ver YAML) ->
validar el sensado tomando ``sensado_muestras`` lecturas de LiDAR
seguidas (una sola lectura puede fallar por ruido; una zona solo se
confirma como pared si aparece en al menos ``sensado_consenso_minimo``
de esas muestras) -> registrar los muros nuevos -> decidir el
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
        self._log_evento(
            'INICIO', meta=nombre(self.mz.goal),
            pista=f'{self.mz.cols}x{self.mz.rows}', celda_cm=self.p.celda_cm,
        )

    # ------------------------------------------------------------------
    # Sensado (equivalente real de Explorer._sense_celda del simulador,
    # con el muro ya confirmado por consenso de varias muestras -- ver
    # motion.py::_iniciar_validacion_muros/_tick_validacion_muros)
    # ------------------------------------------------------------------
    def _procesar_sensado(self, muros_detectados: set):
        pos = self.pose.cell
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
        conteo_txt = ','.join(f'{z}={c}/{self.p.sensado_muestras}' for z, c in self._muros_conteo.items())
        self._log_evento(
            'SENSADO',
            muros_nuevos=','.join(nuevos) or 'ninguno',
            conteo_zonas=conteo_txt,
            sensadas=f'{len(self.sensadas)}/{self.mz.cols * self.mz.rows}',
        )
        return nuevos

    # ------------------------------------------------------------------
    # Decision (identica a Explorer.paso del simulador)
    # ------------------------------------------------------------------
    def _paso(self, pos, targets):
        """Devuelve (d_elegida, siguiente, opciones) o (None, None, [])
        si no hay ninguna direccion sin pared conocida -- en el
        simulador esto nunca pasa (el mapa viene garantizado resoluble
        por datos ground-truth), pero con sensado LiDAR real,
        ruido/desalineacion puede marcar las 4 direcciones de una celda
        como pared. El llamador (_handle_decidir) debe manejar el caso
        None. ``opciones`` se devuelve tambien para poder loguear
        exactamente que evaluo y por que eligio lo que eligio."""
        dist = self.mz.flood(targets, self.conocidos)
        opciones = []
        for n, d in self.mz.vecinos(pos):
            if wall(pos, n) not in self.conocidos:
                opciones.append((dist.get(n, INF), self.p.orden_desempate.index(d), d, n))
        if not opciones:
            return None, None, []
        opciones.sort()
        _, _, d_elegida, siguiente = opciones[0]
        return d_elegida, siguiente, opciones

    # ------------------------------------------------------------------
    # Ciclo de control
    # ------------------------------------------------------------------
    def _on_timer(self):
        if not self.sensores_listos:
            return
        if self._terminado:
            self._publish_twist_vacio()
            return
        if self.obstaculo_abandonado:
            self._fallar('obstaculo al frente persistente (ver warnings anteriores)')
            return
        if self._handle_obstaculo_frente():
            return

        handler = getattr(self, f'_handle_{self._state.lower()}', None)
        if handler is None:
            raise RuntimeError(f'estado desconocido: {self._state}')
        handler()

    def _handle_iniciar(self):
        self._iniciar_pausa()
        self._set_state('PAUSA_CELDA')

    def _handle_pausa_celda(self):
        if self._tick_pausa(self.p.tiempo_pausa_antes_girar_s):
            # Validar DESPUES de la pausa (no antes): la pausa existe
            # para dejar que el chasis se asiente (vibracion/inercia de
            # frenado) antes de confiar en la lectura del LiDAR -- si
            # se sensa apenas se detecta la llegada, esa lectura puede
            # tomarse con el robot todavia en movimiento residual.
            self._iniciar_validacion_muros()
            self._set_state('VALIDANDO_CELDA')

    def _handle_validando_celda(self):
        if self._tick_validacion_muros():
            self._procesar_sensado(self._muros_confirmados())
            self._set_state('DECIDIR')

    def _handle_decidir(self):
        pos = self.pose.cell

        if self.fase == 'A' and pos == self.mz.goal:
            self._log_evento('FASE_B_INICIO', motivo='llego a la meta, verificando ruta optima')
            self.fase = 'B'

        if self.fase == 'A':
            targets = [self.mz.goal]
        else:
            candidata = self.mz.bfs(self.mz.start, self.mz.goal, walls=self.conocidos)
            if candidata is None:
                # Una lectura de LiDAR (ruido o desalineacion) marco un
                # muro que, sumado a lo ya conocido, desconecta el
                # camino a la meta -- imposible en el simulador
                # (ground truth garantiza solucion), real con sensado
                # ruidoso. No hay forma segura de seguir explorando con
                # un mapa que se contradice a si mismo: se aborta.
                self._fallar('mapa inconsistente en fase B (bfs sin camino a la meta)')
                return
            pendientes = [c for c in candidata if c not in self.sensadas]
            if not pendientes:
                self._terminar()
                return
            targets = pendientes

        self.num_celdas += 1
        if self.num_celdas > self.p.max_celdas_recorridas:
            self._log_evento('LIMITE_CELDAS', num_celdas=self.num_celdas)
            self._terminar()
            return

        d_elegida, siguiente, opciones = self._paso(pos, targets)
        if d_elegida is None:
            # Las 4 direcciones de la celda actual quedaron marcadas
            # como pared (sensado ruidoso) -- no hay opcion segura.
            self._fallar(f'sin direccion abierta en {nombre(pos)} (las 4 zonas marcaron pared)')
            return
        self._direccion_elegida = d_elegida

        opciones_txt = ','.join(f'{d}->{nombre(n)}:{dv if dv < INF else "inf"}'
                                 for dv, _, d, n in opciones)
        self._log_evento(
            'DECISION', fase=self.fase, opciones=opciones_txt,
            elegida=d_elegida, hacia=nombre(siguiente),
        )

        lado = lado_para_girar(self.pose.heading, d_elegida)
        if lado == 'NINGUNO':
            self._iniciar_avance()
            self._set_state('AVANZAR')
        else:
            self._lado_giro_pendiente = lado
            self._emitir_pitido()
            self._iniciar_pausa()
            self._set_state('PAUSA_GIRO')

    def _handle_pausa_giro(self):
        if self._tick_pausa(self.p.tiempo_pausa_antes_girar_s):
            self._iniciar_giro()
            self._set_state('GIRAR')

    def _handle_girar(self):
        if self._tick_giro(self._lado_giro_pendiente):
            lado = self._lado_giro_pendiente
            self.pose.girar(lado)
            self._log_evento('GIRO_FIN', lado=lado)
            self._iniciar_avance()
            self._set_state('AVANZAR')

    def _handle_avanzar(self):
        if self._tick_avance():
            self.pose.avanzar()
            self._log_evento('AVANCE_FIN', celda=self.pose.cell_name)
            self._iniciar_pausa()
            self._set_state('PAUSA_CELDA')
            # El sensado ocurre en _handle_pausa_celda, DESPUES de la
            # pausa de 1s -- ver comentario ahi.

    def _terminar(self):
        self._publish_twist_vacio()
        self._terminado = True
        self._set_state('TERMINADO')
        self._guardar_mapa()
        self._log_evento(
            'EXPLORACION_TERMINADA', num_celdas=self.num_celdas,
            sensadas=f'{len(self.sensadas)}/{self.mz.cols * self.mz.rows}',
            mapa=self.p.mapa_salida,
        )

    def _fallar(self, motivo: str):
        """Aborto controlado: detiene el robot y deja de mandar
        comandos, en vez de crashear el nodo (IndexError/TypeError sin
        manejar) o quedar esperando para siempre en silencio. Guarda el
        mapa parcial de todas formas -- util para diagnosticar que paso
        y para no perder lo ya sensado si se necesita reintentar."""
        self._publish_twist_vacio()
        self._terminado = True
        self._set_state('TERMINADO')
        self._guardar_mapa()
        self._log_evento('EXPLORACION_ABORTADA', motivo=motivo)

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


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ExplorerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
