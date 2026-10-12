"""Publica un mapa en /map, una sola vez y retenido.

- Lee un mapa de map_server (yaml + pgm) y lo convierte en OccupancyGrid con la
  misma cuenta que map_server.
- Lo publica una sola vez en /map, con QoS reliable + transient local: un nodo
  que se suscribe después igual lo recibe.
- frame_id 'map'. No publica ninguna transformada: ubicarse en el mapa es
  trabajo de ustedes.
"""
from pathlib import Path

import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

QOS_MAPA = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def leer_pgm(ruta):
    """Lee un PGM binario (P5). Devuelve (ancho, alto, bytes de píxeles)."""
    datos = Path(ruta).read_bytes()
    partes, pos = [], 0
    while len(partes) < 4:  # magic, ancho, alto, maxval
        while datos[pos:pos + 1].isspace():
            pos += 1
        if datos[pos:pos + 1] == b'#':
            pos = datos.index(b'\n', pos)
            continue
        ini = pos
        while not datos[pos:pos + 1].isspace():
            pos += 1
        partes.append(datos[ini:pos])
    pos += 1  # un único whitespace tras maxval
    if partes[0] != b'P5':
        raise ValueError(f'{ruta}: solo se soporta PGM binario (P5)')
    ancho, alto = int(partes[1]), int(partes[2])
    return ancho, alto, datos[pos:pos + ancho * alto]


def cargar_mapa(ruta_yaml):
    """Convierte un mapa de map_server (yaml + pgm) en OccupancyGrid."""
    ruta_yaml = Path(ruta_yaml)
    meta = yaml.safe_load(ruta_yaml.read_text())
    ancho, alto, px = leer_pgm(ruta_yaml.parent / meta['image'])
    negate = int(meta.get('negate', 0))
    occ_t = float(meta.get('occupied_thresh', 0.65))
    libre_t = float(meta.get('free_thresh', 0.25))

    celdas = []
    for fila in range(alto - 1, -1, -1):  # la imagen tiene y hacia abajo, el mapa hacia arriba
        for v in px[fila * ancho:(fila + 1) * ancho]:
            p = (v / 255.0) if negate else ((255 - v) / 255.0)
            celdas.append(100 if p > occ_t else 0 if p < libre_t else -1)

    mapa = OccupancyGrid()
    mapa.header.frame_id = 'map'
    mapa.info.resolution = float(meta['resolution'])
    mapa.info.width, mapa.info.height = ancho, alto
    mapa.info.origin.position.x = float(meta['origin'][0])
    mapa.info.origin.position.y = float(meta['origin'][1])
    mapa.info.origin.orientation.w = 1.0
    mapa.data = celdas
    return mapa


class PublicarMapa(Node):
    def __init__(self):
        super().__init__('publicar_mapa')
        self.declare_parameter('mapa', '')
        ruta = self.get_parameter('mapa').value
        if not ruta:
            raise ValueError('falta el parámetro "mapa" (ruta al .yaml)')

        self.mapa = cargar_mapa(ruta)
        self.pub = self.create_publisher(OccupancyGrid, 'map', QOS_MAPA)
        self.mapa.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.mapa)
        info = self.mapa.info
        self.get_logger().info(
            f'Mapa publicado en /map: {info.width}x{info.height} celdas, '
            f'{info.resolution:.2f} m/celda, origen ({info.origin.position.x:.3f}, '
            f'{info.origin.position.y:.3f})')


def main(args=None):
    rclpy.init(args=args)
    nodo = PublicarMapa()
    try:
        rclpy.spin(nodo)  # sigue vivo para que el mapa quede retenido (transient local)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
