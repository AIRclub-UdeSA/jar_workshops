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
    """Lee un PGM binario (P5) de 8 bits. Devuelve (ancho, alto, bytes de píxeles)."""
    datos = Path(ruta).read_bytes()
    partes, pos = [], 0
    while len(partes) < 4:  # magic, ancho, alto, maxval
        while pos < len(datos) and datos[pos:pos + 1].isspace():
            pos += 1
        if datos[pos:pos + 1] == b'#':
            pos = datos.find(b'\n', pos)
            if pos < 0:
                raise ValueError(f'{ruta}: encabezado PGM incompleto')
            continue
        ini = pos
        while pos < len(datos) and not datos[pos:pos + 1].isspace():
            pos += 1
        if pos == ini:  # se terminó el archivo antes de completar el encabezado
            raise ValueError(f'{ruta}: encabezado PGM incompleto')
        partes.append(datos[ini:pos])
    pos += 1  # un único whitespace tras maxval
    if partes[0] != b'P5':
        raise ValueError(f'{ruta}: solo se soporta PGM binario (P5)')
    try:
        ancho, alto, maxval = (int(p) for p in partes[1:])
    except ValueError:
        raise ValueError(f'{ruta}: encabezado PGM inválido') from None
    if ancho <= 0 or alto <= 0:
        raise ValueError(f'{ruta}: tamaño inválido ({ancho}x{alto})')
    if maxval != 255:
        raise ValueError(f'{ruta}: solo se soporta PGM de 8 bits (maxval 255), tiene {maxval}')
    px = datos[pos:pos + ancho * alto]
    if len(px) != ancho * alto:
        raise ValueError(
            f'{ruta}: el encabezado dice {ancho}x{alto} = {ancho * alto} píxeles '
            f'pero el archivo trae {len(px)}')
    return ancho, alto, px


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
