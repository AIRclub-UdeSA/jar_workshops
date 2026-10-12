"""Puntúa los reportes que llegan a /victimas contra los centros reales de las cajas rojas.

Reglas:
- Cuentan solo los primeros N reportes, con N = cantidad de cajas rojas. Los demás se ignoran.
- Un reporte mal formado (frame_id distinto de 'map', NaN o infinito) gasta un intento y da 0.
- Cada reporte se asigna a la caja roja más cercana, si está a 0,5 m o menos.
- Cada caja se queda con su reporte más cercano: 100 puntos a 0,25 m o menos, 50 hasta 0,5 m.

Además publica marcadores para RViz (/evaluador/marcadores): las cajas reales, los anillos de
0,25 y 0,5 m alrededor de cada caja roja, y cada reporte con su número y sus puntos.
"""
import math

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

RADIO_EXACTO = 0.25
RADIO_MAXIMO = 0.50
PUNTOS_EXACTO = 100
PUNTOS_APROXIMADO = 50
EPS = 1e-9  # los bordes (0,25 m justos) cuentan a favor

QOS_RETENIDO = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL)


def puntos_por_distancia(d):
    if d <= RADIO_EXACTO + EPS:
        return PUNTOS_EXACTO
    if d <= RADIO_MAXIMO + EPS:
        return PUNTOS_APROXIMADO
    return 0


def pares(lista):
    return [(lista[i], lista[i + 1]) for i in range(0, len(lista) - 1, 2)]


class Evaluador(Node):

    def __init__(self):
        super().__init__('evaluador')
        self.declare_parameter('victimas', [0.0])
        self.declare_parameter('distractores', [0.0])
        self.declare_parameter('lado_caja', 0.18)
        self.victimas = pares(self.get_parameter('victimas').value)
        self.distractores = pares(self.get_parameter('distractores').value)
        self.lado = self.get_parameter('lado_caja').value
        self.maximo = len(self.victimas)
        self.reportes = []          # (x, y, valido)
        self.ignorados = 0

        self.create_subscription(PointStamped, 'victimas', self.recibir_reporte, 10)
        self.pub_marcadores = self.create_publisher(MarkerArray, 'evaluador/marcadores', QOS_RETENIDO)
        self.pub_puntaje = self.create_publisher(String, 'evaluador/puntaje', QOS_RETENIDO)

        self.get_logger().info(
            f'{len(self.victimas)} cajas rojas: se aceptan {self.maximo} reportes en /victimas.')
        self.publicar()

    # ---------- reportes ----------

    def recibir_reporte(self, msg: PointStamped):
        if len(self.reportes) >= self.maximo:
            self.ignorados += 1
            self.get_logger().warn(
                f'Reporte ignorado: ya se usaron los {self.maximo} reportes.')
            return
        x, y = msg.point.x, msg.point.y
        valido = msg.header.frame_id == 'map' and math.isfinite(x) and math.isfinite(y)
        self.reportes.append((x, y, valido))
        n = len(self.reportes)
        if not valido:
            self.get_logger().warn(
                f'Reporte {n}/{self.maximo} mal formado (frame_id "{msg.header.frame_id}", '
                f'x={x}, y={y}): gasta el intento y da 0 puntos.')
        else:
            d, v = self.victima_mas_cercana(x, y)
            aviso = ''
            if any(math.hypot(x - dx, y - dy) <= RADIO_EXACTO for dx, dy in self.distractores):
                aviso = ' Está sobre una caja que no es roja.'
            self.get_logger().info(
                f'Reporte {n}/{self.maximo} en ({x:.3f}, {y:.3f}): a {d:.3f} m de la caja roja {v + 1}.{aviso}')
        self.publicar()

    def victima_mas_cercana(self, x, y):
        distancias = [math.hypot(x - vx, y - vy) for vx, vy in self.victimas]
        v = min(range(len(distancias)), key=lambda i: distancias[i])
        return distancias[v], v

    def puntuar(self):
        """{caja: (reporte, distancia)}: cada reporte va a la caja roja más cercana dentro de
        RADIO_MAXIMO, y cada caja se queda con su reporte más cercano."""
        mejor = {}
        for i, (x, y, valido) in enumerate(self.reportes):
            if not valido:
                continue
            d, v = self.victima_mas_cercana(x, y)
            if d > RADIO_MAXIMO + EPS:
                continue
            if v not in mejor or d < mejor[v][1] - EPS:
                mejor[v] = (i, d)
        return mejor

    # ---------- salida ----------

    def publicar(self):
        mejor = self.puntuar()
        total = sum(puntos_por_distancia(d) for _, d in mejor.values())
        por_reporte = {i: (v, d) for v, (i, d) in mejor.items()}

        lineas = [f'Puntaje: {total} de {PUNTOS_EXACTO * len(self.victimas)} '
                  f'({len(self.reportes)}/{self.maximo} reportes usados'
                  + (f', {self.ignorados} ignorados' if self.ignorados else '') + ')']
        for v in range(len(self.victimas)):
            if v in mejor:
                i, d = mejor[v]
                lineas.append(f'  caja roja {v + 1}: reporte {i + 1} a {d:.3f} m -> {puntos_por_distancia(d)} puntos')
            else:
                lineas.append(f'  caja roja {v + 1}: sin reporte que puntúe')
        texto = '\n'.join(lineas)
        self.pub_puntaje.publish(String(data=texto))
        if self.reportes:
            self.get_logger().info(texto)

        self.pub_marcadores.publish(self.marcadores(por_reporte))

    def marcador(self, ident, tipo, x, y, color, escala, ns, z=0.0):
        m = Marker()
        m.header.frame_id = 'map'
        m.ns, m.id, m.type, m.action = ns, ident, tipo, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = escala
        m.color.r, m.color.g, m.color.b, m.color.a = color
        return m

    def marcadores(self, por_reporte):
        arr = MarkerArray()
        borrar = Marker()
        borrar.action = Marker.DELETEALL
        arr.markers.append(borrar)
        k = 0
        for vx, vy in self.victimas:
            # zonas de 0,5 m y 0,25 m (discos planos, transparentes) y la caja
            arr.markers.append(self.marcador(k, Marker.CYLINDER, vx, vy, (1.0, 0.3, 0.3, 0.12),
                                             (2 * RADIO_MAXIMO, 2 * RADIO_MAXIMO, 0.002), 'zonas', 0.002)); k += 1
            arr.markers.append(self.marcador(k, Marker.CYLINDER, vx, vy, (1.0, 0.3, 0.3, 0.25),
                                             (2 * RADIO_EXACTO, 2 * RADIO_EXACTO, 0.004), 'zonas', 0.004)); k += 1
            arr.markers.append(self.marcador(k, Marker.CUBE, vx, vy, (0.9, 0.05, 0.05, 0.9),
                                             (self.lado, self.lado, 0.25), 'cajas', 0.125)); k += 1
        for dx, dy in self.distractores:
            arr.markers.append(self.marcador(k, Marker.CUBE, dx, dy, (0.1, 0.3, 0.95, 0.9),
                                             (self.lado, self.lado, 0.25), 'cajas', 0.125)); k += 1
        for i, (x, y, valido) in enumerate(self.reportes):
            if not valido:
                continue
            puntos = puntos_por_distancia(por_reporte[i][1]) if i in por_reporte else 0
            color = {100: (0.1, 1.0, 0.2, 1.0), 50: (1.0, 0.85, 0.1, 1.0)}.get(puntos, (0.7, 0.7, 0.7, 1.0))
            arr.markers.append(self.marcador(k, Marker.SPHERE, x, y, color, (0.07, 0.07, 0.07),
                                             'reportes', 0.3)); k += 1
            t = self.marcador(k, Marker.TEXT_VIEW_FACING, x, y + 0.12, (1.0, 1.0, 1.0, 1.0),
                              (0.0, 0.0, 0.08), 'reportes', 0.35); k += 1
            t.text = f'{i + 1}:{puntos}pts'
            arr.markers.append(t)
        return arr


def main(args=None):
    rclpy.init(args=args)
    nodo = Evaluador()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
