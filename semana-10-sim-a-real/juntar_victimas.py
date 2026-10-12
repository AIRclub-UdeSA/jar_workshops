import math

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, PoseArray
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


class JuntarVictimas(Node):
    """
    Junta las detecciones de cajas rojas en una posición por caja y la
    publica en /victimas.

    Escucha /objetos_rojos: la salida de tu detector de la semana 07 corriendo
    sobre /scan_rojo, un PoseArray en el frame `map` con un punto por cada
    racha roja del lidar. Cada punto es el promedio de los puntos de **una
    cara** de la caja, no su centro.

    Con eso:
      1. corre cada detección media caja hacia adentro, para acercarla al
         centro;
      2. agrupa las detecciones que están cerca (una caja = un grupo);
      3. cuando un grupo junta `detecciones_minimas`, publica la mediana del
         grupo en /victimas, una sola vez por grupo y como mucho `victimas`
         veces en total.
    """

    def __init__(self):
        super().__init__('juntar_victimas')
        self.declare_parameter('victimas', 2)
        self.declare_parameter('detecciones_minimas', 30)
        self.declare_parameter('radio_grupo', 0.30)
        self.declare_parameter('mitad_caja', 0.09)
        self.declare_parameter('base_frame', 'base_footprint')
        self.victimas = self.get_parameter('victimas').value
        self.minimas = self.get_parameter('detecciones_minimas').value
        self.radio = self.get_parameter('radio_grupo').value
        self.mitad = self.get_parameter('mitad_caja').value
        self.base_frame = self.get_parameter('base_frame').value

        # Cada grupo es un dict: {'puntos': [(x, y), ...], 'publicado': bool}
        self.grupos = []
        self.publicados = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(PoseArray, 'objetos_rojos', self.recibir_objetos, 10)
        self.pub = self.create_publisher(PointStamped, 'victimas', 10)

    def posicion_robot(self):
        """(x, y) del robot en `map`, o None si todavía no hay transformada
        (por ejemplo, antes de que el localizador publique `map -> odom`)."""
        try:
            tf = self.tf_buffer.lookup_transform('map', self.base_frame, Time())
        except Exception:
            return None
        return tf.transform.translation.x, tf.transform.translation.y

    def recibir_objetos(self, msg: PoseArray):
        """
        TODO: pasar cada detección de la cara al centro de la caja y
        agregarla a un grupo.

        El lidar ve la caja desde donde está el robot, así que el punto que
        llega cae en la cara que mira al robot. El centro está **media caja
        más lejos**, en la misma dirección en que el robot ve la caja.

          1. Pedí la posición del robot con `self.posicion_robot()`. Si es
             `None`, `return`.
          2. Para cada `pose` de `msg.poses`, con `x, y = pose.position.x,
             pose.position.y`:
             - calculá el vector del robot a la detección (`dx`, `dy`) y su
               largo `d` con `math.hypot`. Si `d` es casi cero, salteala;
             - corré el punto `self.mitad` metros en esa dirección:
               `x + self.mitad * dx / d`, y lo mismo para `y`;
             - pasáselo a `self.agregar(x, y)`.
        """
        pass

    def agregar(self, x, y):
        """
        TODO: agregar el punto (x, y) al grupo que corresponda, o crear uno
        nuevo.

          1. Recorré `self.grupos`. Para cada grupo, calculá su mediana con
             `np.median(np.array(g['puntos']), axis=0)`.
          2. Si el punto está a menos de `self.radio` de esa mediana, agregalo
             a `g['puntos']`, llamá a `self.quizas_publicar(g)` y `return`.
          3. Si no entró en ningún grupo, creá uno nuevo
             (`{'puntos': [(x, y)], 'publicado': False}`), agregalo a
             `self.grupos` y llamá a `self.quizas_publicar` con él.

        ¿Por qué mediana y no promedio? Pensá qué pasa con unos pocos puntos
        que caen lejos de la caja.
        """
        pass

    def quizas_publicar(self, g):
        """
        TODO: publicar la posición del grupo en /victimas si ya está listo.

          1. No publiques si el grupo ya se publicó, si ya se publicaron
             `self.victimas` reportes, o si el grupo tiene menos de
             `self.minimas` puntos. Cada mensaje en /victimas gasta uno de los
             reportes, así que no se puede publicar de más.
          2. Calculá la mediana del grupo, armá un `PointStamped` con
             `header.frame_id = 'map'` y el stamp de ahora
             (`self.get_clock().now().to_msg()`), y publicalo.
          3. Marcá el grupo como publicado, sumá 1 a `self.publicados` y
             logueá dónde lo publicaste.
        """
        pass


def main(args=None):
    rclpy.init(args=args)
    nodo = JuntarVictimas()
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
