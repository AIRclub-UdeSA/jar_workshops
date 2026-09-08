import math

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                        QoSReliabilityPolicy, qos_profile_sensor_data)
from rclpy.time import Time

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseArray, Quaternion
from visualization_msgs.msg import Marker, MarkerArray

from tf2_ros import Buffer, TransformListener


def yaw_de_quaternion(q: Quaternion) -> float:
    """Extrae el ángulo de yaw (rotación en el plano) de un quaternion.
    Álgebra de siempre, no es el objetivo de este workshop — la misma
    cuenta que ya usaste en semana 06."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def agrupar_en_rachas(booleanos: np.ndarray):
    """
    TODO: agrupar los índices contiguos de un array booleano en "rachas".

    Un obstáculo real casi nunca genera un solo punto "no mapeado" suelto:
    genera una racha de varios rayos seguidos del lidar pegando en la misma
    cara del objeto (igual que ya viste con `/scan_rojo` en semana 04). Acá
    identificamos esas rachas para después convertir cada una en un objeto.

    Parámetro: `booleanos`, un array de N bools (uno por rayo del scan,
    True donde ese rayo cayó en un punto "no mapeado").

    Devolvé una lista de arrays de índices, uno por racha — por ejemplo,
    para [F,F,T,T,T,F,T,T,F] tendría que devolver [[2,3,4], [6,7]].

    Pasos sugeridos:
      1. `np.flatnonzero(booleanos)` te da, en orden, los índices donde hay
         `True`. Si viene vacío (no hay ningún `True`), devolvé `[]`
         directamente.
      2. Dos índices consecutivos de esa lista pertenecen a la misma racha
         si difieren en exactamente 1 (son rayos contiguos del scan); una
         racha nueva empieza donde el índice "salta" más que eso.
         `np.diff(indices)` te da esas diferencias consecutivas;
         `np.flatnonzero(diferencias > 1)` te da las posiciones (dentro de
         `indices`) donde hay un salto — es decir, dónde cortar.
      3. `np.split(indices, cortes)` parte el array de índices en esos
         puntos. Ojo: `np.split` corta *antes* de cada posición que le
         pasás, así que al array de cortes del paso 2 hay que sumarle 1
         antes de pasárselo.
      4. Caso especial — el scan es circular (el rayo en el índice 0 y el
         último son angularmente contiguos, no hay "borde" real): si
         quedó más de una racha, Y la primera empieza en el índice 0, Y la
         última termina en el último índice del array, en realidad son la
         misma racha partida en dos por el límite del array — hay que
         unirlas (concatenar la última al principio de la primera, y sacar
         la última de la lista).
      5. Devolver la lista de rachas resultante.
    """
    pass


class DetectorObstaculos(Node):
    """
    Compara cada punto del /scan contra /likelihood_map (el campo de
    verosimilitud de semana 06) para encontrar puntos que el mapa no
    esperaba ahí, los agrupa en objetos (rachas contiguas del scan) y
    publica la coordenada en `map` de cada uno.
    """

    def __init__(self):
        super().__init__('detector_obstaculos')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('umbral_probabilidad', 25.0)
        self.declare_parameter('racha_minima', 3)
        self.declare_parameter('distancia_minima_valida', 0.25)

        self.map_frame = self.get_parameter('map_frame').value
        self.umbral_probabilidad = self.get_parameter('umbral_probabilidad').value
        self.racha_minima = self.get_parameter('racha_minima').value
        self.distancia_minima_valida = self.get_parameter('distancia_minima_valida').value

        self.campo = None
        self.info_mapa = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.aviso_tf_mostrado = False

        # /likelihood_map se publica una sola vez, retenido (mismo QoS que
        # /map en semana 06): transient local, para que igual llegue aunque
        # este nodo arranque después que campo_verosimilitud.
        qos_mapa = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, 'likelihood_map', self.recibir_campo, qos_mapa)
        # El lidar publica en Best Effort (qos_profile_sensor_data), como en
        # las semanas 03, 04 y 06.
        self.create_subscription(
            LaserScan, 'scan', self.recibir_scan, qos_profile_sensor_data)

        self.pub_scan_no_mapeado = self.create_publisher(LaserScan, 'scan_no_mapeado', 10)
        self.pub_objetos = self.create_publisher(PoseArray, 'objetos_no_mapeados', 10)
        self.pub_marcadores = self.create_publisher(MarkerArray, 'objetos_no_mapeados_markers', 10)

    # ---------- callbacks ----------

    def recibir_campo(self, msg: OccupancyGrid):
        alto, ancho = msg.info.height, msg.info.width
        self.campo = np.array(msg.data, dtype=np.float64).reshape((alto, ancho))
        self.info_mapa = msg.info
        self.get_logger().info('Campo de verosimilitud recibido.')

    def obtener_transform_a_map(self, frame_id_scan):
        """Busca en tf2 la transformada del frame del lidar a `map`. None si
        todavía no está disponible (por ejemplo, si este nodo arranca antes
        de que semana 06 haya publicado map->odom)."""
        try:
            return self.tf_buffer.lookup_transform(self.map_frame, frame_id_scan, Time())
        except Exception as e:
            if not self.aviso_tf_mostrado:
                self.get_logger().warn(
                    f'Todavía no hay tf {frame_id_scan}->{self.map_frame}: {e}')
                self.aviso_tf_mostrado = True
            return None

    def recibir_scan(self, msg: LaserScan):
        """
        TODO: el corazón del workshop — corre una vez por cada scan que
        llega. Comparar cada punto contra /likelihood_map y, con los que no
        calzan, armar los objetos "no mapeados".

          1. Si `self.campo` es `None` (todavía no llegó /likelihood_map),
             `return`.
          2. Pedir la transformada con
             `self.obtener_transform_a_map(msg.header.frame_id)`. Si da
             `None`, `return`.
          3. Armar `rangos` (`np.array(msg.ranges)`) y `angulos`
             (`msg.angle_min + np.arange(n) * msg.angle_increment`, con
             `n = len(rangos)`).
          4. Armar `validos`: `True` donde el rango es finito
             (`np.isfinite`) y está entre un mínimo y `msg.range_max`. Para
             el mínimo usá `max(msg.range_min, self.distancia_minima_valida)`
             en vez de `msg.range_min` solo — el lidar del robot rebota
             contra su propia estructura (antena, soporte) y devuelve
             rangos cortos pero "válidos" para el sensor;
             `distancia_minima_valida` filtra ese rebote.
          5. Pasar cada `(rango, ángulo)` de polar a cartesiano, en el
             frame del lidar (mismo patrón que semana 04):
               `xs_laser = rangos * cos(angulos)`
               `ys_laser = rangos * sin(angulos)`
          6. Aplicar la transformada del paso 2 a esos puntos para
             llevarlos a `map`. De `transform.transform` sacá la
             traslación (`tx, ty = .translation.x, .translation.y`) y el
             yaw (`yaw_de_quaternion(transform.rotation)`); con
             `c, s = cos(yaw), sin(yaw)`:
               `xs_map = tx + c*xs_laser - s*ys_laser`
               `ys_map = ty + s*xs_laser + c*ys_laser`
          7. Convertir cada punto `(x, y)` en `map` a índices de fila y
             columna de `self.campo`, usando `self.info_mapa.resolution` y
             `self.info_mapa.origin.position.(x/y)` (mismo cálculo que ya
             viste en `pesar_particulas` de `localizador.py`, semana 06):
               `col  = ((xs_map - origen_x) / resolucion)`, redondeado a
                      entero (`.astype(np.int32)`)
               `fila = ((ys_map - origen_y) / resolucion)`, ídem
          8. Armar `dentro_grilla`: `True` donde `0 <= col < ancho` Y
             `0 <= fila < alto` (`ancho, alto = self.campo.shape`, en ese
             orden porque una grilla se indexa `[fila, columna]`).
          9. Armar `probabilidad`: un array de N ceros
             (`np.zeros(len(rangos))`), y sobreescribirlo con
             `self.campo[fila, col]` **solo** en las posiciones donde
             `dentro_grilla` es `True` (indexá con la máscara booleana en
             ambos lados:
             `probabilidad[dentro_grilla] = self.campo[fila[dentro_grilla], col[dentro_grilla]]`).
          10. Armar `no_mapeado`: `validos` Y `dentro_grilla` Y
              `(probabilidad < self.umbral_probabilidad)`. El
              `Y dentro_grilla` no es opcional: un punto que cae afuera del
              área que cubre el mapa queda con probabilidad 0 en el paso
              anterior (nunca se sobreescribe), y sin este filtro se
              reportaría **siempre** como "obstáculo no mapeado" — el mapa
              simplemente no tiene nada que decir sobre lo que está fuera
              de sus límites, así que ahí no se puede concluir nada.
          11. Llamar a `self.publicar_scan_filtrado(msg, rangos, no_mapeado)`
              y `self.publicar_objetos(no_mapeado, xs_map, ys_map)`.
        """
        pass

    # ---------- publishers ----------

    def publicar_scan_filtrado(self, msg: LaserScan, rangos, no_mapeado):
        """Mismo truco que /scan_rojo en semana 04: una copia del scan de
        entrada con `inf` (sin retorno) en todos los rangos salvo los "no
        mapeados". Sirve para debug visual en RViz — el resultado que
        importa de este workshop son los objetos de publicar_objetos()."""
        salida = LaserScan()
        salida.header = msg.header
        salida.angle_min = msg.angle_min
        salida.angle_max = msg.angle_max
        salida.angle_increment = msg.angle_increment
        salida.time_increment = msg.time_increment
        salida.scan_time = msg.scan_time
        salida.range_min = msg.range_min
        salida.range_max = msg.range_max
        salida.ranges = np.where(no_mapeado, rangos, math.inf).tolist()
        self.pub_scan_no_mapeado.publish(salida)

    def publicar_objetos(self, no_mapeado, xs_map, ys_map):
        """Agrupa no_mapeado en rachas (con agrupar_en_rachas, de arriba),
        descarta las muy cortas (ruido), y por cada racha que sobrevive
        promedia sus puntos en `map` para obtener la coordenada del
        objeto — publica esa lista como PoseArray (el dato) y como
        MarkerArray (para verla en RViz sin agregar un display de
        PoseArray)."""
        ahora = self.get_clock().now().to_msg()

        objetos = PoseArray()
        objetos.header.frame_id = self.map_frame
        objetos.header.stamp = ahora

        marcadores = MarkerArray()
        borrar_todos = Marker()
        borrar_todos.action = Marker.DELETEALL
        marcadores.markers.append(borrar_todos)

        for i, indices in enumerate(agrupar_en_rachas(no_mapeado)):
            if len(indices) < self.racha_minima:
                continue

            x = float(np.mean(xs_map[indices]))
            y = float(np.mean(ys_map[indices]))

            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.orientation.w = 1.0
            objetos.poses.append(pose)

            marcador = Marker()
            marcador.header.frame_id = self.map_frame
            marcador.header.stamp = ahora
            marcador.ns = 'objetos_no_mapeados'
            marcador.id = i
            marcador.type = Marker.SPHERE
            marcador.action = Marker.ADD
            marcador.pose.position.x = x
            marcador.pose.position.y = y
            marcador.pose.orientation.w = 1.0
            marcador.scale.x = marcador.scale.y = marcador.scale.z = 0.2
            marcador.color.r = 1.0
            marcador.color.g = 0.3
            marcador.color.a = 1.0
            marcadores.markers.append(marcador)

        self.pub_objetos.publish(objetos)
        self.pub_marcadores.publish(marcadores)


def main(args=None):
    rclpy.init(args=args)
    nodo = DetectorObstaculos()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
