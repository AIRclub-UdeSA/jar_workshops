import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                        QoSReliabilityPolicy, qos_profile_sensor_data)
from rclpy.time import Time

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion

from tf2_ros import Buffer, TransformListener

from cobertura_mapa.trazado import trazar_rayo


def yaw_de_quaternion(q: Quaternion) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GrillaCobertura(Node):
    """
    Construye, a partir del mapa estático de map_server, una segunda
    grilla propia de "cubierto / no cubierto", a una resolución más gruesa
    (factor_escala celdas del mapa original por lado, por cada celda de
    cobertura). Por cada rayo del /scan, traza la línea (Bresenham) entre
    la celda del robot y la celda donde terminó el rayo, y marca como
    cubiertas todas las celdas libres que esa línea atraviesa — no un
    radio fijo alrededor del robot, que ignoraría las paredes y marcaría
    como "visto" lo que en realidad está oculto detrás de un obstáculo.

    Publica la grilla resultante como su propio OccupancyGrid
    (`grilla_cobertura`): 0 = celda libre ya cubierta, 100 = celda libre
    todavía sin cubrir, -1 = celda ocupada o fuera del mapa (no se
    dibuja). Con "Color Scheme: costmap" en el display de Map de RViz,
    eso se ve como una zona que pasa de rojo a verde a medida que el
    robot la recorre.

    No marca nada hasta que un humano confirma la pose inicial con "2D
    Pose Estimate" en RViz (mismo tópico `initialpose` que usa
    `localizador` para re-sembrar el filtro de partículas) -- si se
    empieza a marcar cobertura con una pose todavía sin converger, esas
    marcas quedan mal para siempre: a diferencia de la pose del robot, la
    cobertura no se corrige sola después.
    """

    def __init__(self):
        super().__init__('grilla_cobertura')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('factor_escala', 4)
        self.declare_parameter('distancia_minima_valida', 0.25)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.factor_escala = self.get_parameter('factor_escala').value
        self.distancia_minima_valida = self.get_parameter('distancia_minima_valida').value

        # Se completan al recibir /map (recibir_mapa)
        self.libre_c = None       # bool[alto_c, ancho_c]: la celda es transitable
        self.cubierta = None      # bool[alto_c, ancho_c]: la celda ya fue vista
        self.alto_c = self.ancho_c = 0
        self.resolucion_c = 0.0
        self.origen_x = self.origen_y = 0.0
        self.orientacion_origen = Quaternion()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.aviso_tf_mostrado = False
        self.pose_inicial_confirmada = False

        qos_mapa = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, 'map', self.recibir_mapa, qos_mapa)
        self.create_subscription(LaserScan, 'scan', self.recibir_scan, qos_profile_sensor_data)
        self.create_subscription(
            PoseWithCovarianceStamped, 'initialpose', self.recibir_pose_inicial, 10)
        self.pub_grilla = self.create_publisher(OccupancyGrid, 'grilla_cobertura', qos_mapa)

    # ---------- construcción de la grilla ----------

    def recibir_mapa(self, msg: OccupancyGrid):
        alto, ancho = msg.info.height, msg.info.width
        datos = np.array(msg.data, dtype=np.int16).reshape((alto, ancho))
        libre_fino = (datos == 0)

        factor = self.factor_escala
        self.alto_c = alto // factor
        self.ancho_c = ancho // factor
        recorte = libre_fino[:self.alto_c * factor, :self.ancho_c * factor]
        bloques = recorte.reshape(self.alto_c, factor, self.ancho_c, factor)
        # Una celda de cobertura es "libre" si la mayoría de las celdas
        # finas que agrupa lo son — evita que una sola celda ocupada en el
        # borde de un pasillo angosto marque todo el bloque como pared.
        self.libre_c = bloques.mean(axis=(1, 3)) > 0.5
        self.cubierta = np.zeros((self.alto_c, self.ancho_c), dtype=bool)

        self.resolucion_c = msg.info.resolution * factor
        self.origen_x = msg.info.origin.position.x
        self.origen_y = msg.info.origin.position.y
        self.orientacion_origen = msg.info.origin.orientation

        self.get_logger().info(
            f'Grilla de cobertura: {self.alto_c}x{self.ancho_c} celdas de '
            f'{self.resolucion_c:.2f} m ({int(np.sum(self.libre_c))} celdas libres).')
        self.publicar_grilla()

    def celda_de(self, x: float, y: float):
        col = int((x - self.origen_x) / self.resolucion_c)
        fila = int((y - self.origen_y) / self.resolucion_c)
        return fila, col

    def dentro_de_grilla(self, fila: int, col: int) -> bool:
        return 0 <= fila < self.alto_c and 0 <= col < self.ancho_c

    # ---------- callbacks ----------

    def recibir_pose_inicial(self, msg):
        # Si ya se había confirmado antes (el humano corrigió la pose a
        # mitad de recorrido), la cobertura marcada hasta ahora quedó
        # calculada con una localización que ya no se considera buena --
        # se vuelve a empezar de cero en vez de arrastrar marcas dudosas.
        if self.cubierta is not None:
            self.cubierta[:] = False
            self.publicar_grilla()
        self.pose_inicial_confirmada = True
        self.get_logger().info('Pose inicial confirmada — arranca a marcar cobertura.')

    def obtener_pose_robot(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except Exception as e:
            if not self.aviso_tf_mostrado:
                self.get_logger().warn(f'Todavía no hay tf {self.base_frame}->{self.map_frame}: {e}')
                self.aviso_tf_mostrado = True
            return None
        t = tf.transform.translation
        return t.x, t.y

    def recibir_scan(self, msg: LaserScan):
        """
        TODO: el corazón de este nodo -- corre una vez por cada scan que
        llega. Por cada rayo del lidar, trazar la línea entre la celda del
        robot y la celda donde terminó el rayo, y marcar como cubiertas
        todas las celdas libres que esa línea atraviesa.

          1. Si `self.cubierta` es `None` (todavía no llegó `/map`) o
             `self.pose_inicial_confirmada` es `False` (todavía no hay un
             humano confirmando dónde está el robot), `return`.
          2. Pedir la pose del robot con `self.obtener_pose_robot()` (ya
             resuelta). Si da `None`, `return`.
          3. Convertir esa pose a celda con `self.celda_de(x, y)`. Si cae
             afuera de la grilla (`not self.dentro_de_grilla(...)`),
             `return` -- no hay nada que trazar desde ahí.
          4. Pedir la transformada del frame del scan a `map` con
             `self.tf_buffer.lookup_transform(...)` (mismo patrón que
             semana 04 y semana 07). Si falla (`except Exception`),
             `return`.
          5. Armar `rangos` (`np.array(msg.ranges)`) y `angulos`
             (`msg.angle_min + np.arange(n) * msg.angle_increment`, con
             `n = len(rangos)`).
          6. Calcular, para cada rayo, hasta dónde "vio" el lidar
             (`alcance`), y cuáles rayos hay que descartar por completo
             (`rebote_corto`):
             - Si el rango es finito y está entre
               `max(msg.range_min, self.distancia_minima_valida)` y
               `msg.range_max`: el alcance es ese rango tal cual.
             - Si no tiene retorno (`inf`) o es mayor a `msg.range_max`:
               tratarlo como si hubiera "visto" hasta `msg.range_max` --
               un rayo sin retorno también es información (no hay nada
               ahí), y para cobertura eso cuenta igual que un rayo que sí
               pegó en algo.
             - Si el rango es finito pero menor al mínimo válido: es un
               rebote contra la propia estructura del robot (antena,
               soporte, igual que en semana 07) -- ese rayo se descarta
               entero, no se traza.
             (Tip: `np.where(condición, valor_si_true, valor_si_false)`
             te arma `alcance` vectorizado sin loop.)
          7. Pasar cada rayo de polar a cartesiano en el frame del lidar
             (`xs_laser = alcance * cos(angulos)`, ídem con `sin`), y de
             ahí a `map` con la transformada del paso 4 -- mismo patrón
             que ya usaste en semana 04 y semana 07
             (`xs_map = tx + c*xs_laser - s*ys_laser`, con
             `c, s = cos(yaw), sin(yaw)` del `tf`).
          8. Por cada rayo que no se descartó en el paso 6 (recorrer con
             un `for i in range(n)` normal, no hace falta vectorizar
             esta parte): convertir el punto final `(xs_map[i], ys_map[i])`
             a celda con `self.celda_de()`, y llamar a `trazar_rayo()`
             (importado arriba, de `cobertura_mapa.trazado`) entre la
             celda del robot (paso 3) y esa celda. Por cada celda que
             devuelva la traza: si cae afuera de la grilla, cortar ahí
             (`break` -- no tiene sentido seguir, ya salió del área que
             cubre el mapa); si no, y la celda es libre (`self.libre_c`)
             y todavía no estaba cubierta, marcarla cubierta
             (`self.cubierta[fila, col] = True`).
          9. Si se marcó al menos una celda nueva en el paso 8, llamar a
             `self.publicar_grilla()` (ya resuelta) para que se vea en
             RViz en vivo -- si no cambió nada, no hace falta publicar de
             nuevo.
        """
        pass

    # ---------- publisher ----------

    def publicar_grilla(self):
        msg = OccupancyGrid()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.resolucion_c
        msg.info.width = self.ancho_c
        msg.info.height = self.alto_c
        msg.info.origin.position.x = self.origen_x
        msg.info.origin.position.y = self.origen_y
        msg.info.origin.orientation = self.orientacion_origen

        datos = np.full((self.alto_c, self.ancho_c), -1, dtype=np.int8)
        datos[self.libre_c & self.cubierta] = 0
        datos[self.libre_c & ~self.cubierta] = 100
        msg.data = datos.flatten().tolist()
        self.pub_grilla.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    nodo = GrillaCobertura()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
