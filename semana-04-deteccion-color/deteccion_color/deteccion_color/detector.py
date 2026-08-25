import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge

FRECUENCIA_HZ = 10.0


class DetectorColor(Node):

    def __init__(self):
        super().__init__('detector_color')

        # El rojo, en el círculo de tonos (H) de HSV, queda partido a ambos
        # extremos (cerca de 0 y cerca de 180 en la escala 0-179 de OpenCV).
        # Por eso hacen falta dos rangos de H, no uno solo.
        self.declare_parameter('hue_rojo_bajo_1', 0.0)
        self.declare_parameter('hue_rojo_alto_1', 10.0)
        self.declare_parameter('hue_rojo_bajo_2', 170.0)
        self.declare_parameter('hue_rojo_alto_2', 180.0)
        # Saturación y valor (brillo) mínimos, 0-255. Sin este piso, un
        # gris o un rosa muy pálido también entrarían en el rango de H del
        # rojo y se contarían como "rojo".
        self.declare_parameter('saturacion_min', 120.0)
        self.declare_parameter('valor_min', 80.0)
        # Área mínima (en píxeles) del contorno rojo más grande para
        # considerar que lo que ve la cámara es un cuadrado y no ruido.
        self.declare_parameter('area_minima_px', 800.0)

        self.hue_rojo_bajo_1 = self.get_parameter('hue_rojo_bajo_1').value
        self.hue_rojo_alto_1 = self.get_parameter('hue_rojo_alto_1').value
        self.hue_rojo_bajo_2 = self.get_parameter('hue_rojo_bajo_2').value
        self.hue_rojo_alto_2 = self.get_parameter('hue_rojo_alto_2').value
        self.saturacion_min = self.get_parameter('saturacion_min').value
        self.valor_min = self.get_parameter('valor_min').value
        self.area_minima_px = self.get_parameter('area_minima_px').value

        self.puente = CvBridge()

        self.publisher_ = self.create_publisher(Bool, 'rojo_detectado', 10)
        self.subscription = self.create_subscription(
            Image, 'cam_1/color/image_raw', self.recibir_imagen, 10
        )
        self.timer = self.create_timer(1.0 / FRECUENCIA_HZ, self.procesar_imagen)

        self.ultima_imagen = None
        self.ultimo_valor_publicado = None

    def recibir_imagen(self, msg: Image):
        """Convierte el Image de ROS a un array BGR de OpenCV y lo guarda.
        No decide nada acá — la decisión pasa en procesar_imagen()."""
        self.ultima_imagen = self.puente.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def area_mayor_contorno(self, mascara: np.ndarray) -> float:
        """Dada una máscara binaria (0/255), devuelve el área (en píxeles)
        del contorno más grande que encuentra. 0.0 si no hay ninguno."""
        contornos, _ = cv2.findContours(
            mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contornos:
            return 0.0
        return max(cv2.contourArea(c) for c in contornos)

    def mascara_rojo(self, imagen_hsv: np.ndarray) -> np.ndarray:
        """
        TODO: devolver una máscara binaria (mismo alto/ancho que
        imagen_hsv, dtype uint8) con 255 donde el pixel es "rojo" y 0 donde
        no lo es.

        Pasos sugeridos (con cv2.inRange; imagen_hsv tiene 3 canales H,S,V):
          1. Armar el límite inferior y superior del primer rango de rojo
             como arrays [H, S, V]: usá hue_rojo_bajo_1 / hue_rojo_alto_1
             para H, y saturacion_min / valor_min como piso de S y V (255
             como techo de ambos).
          2. Lo mismo para el segundo rango, con hue_rojo_bajo_2 / alto_2.
          3. cv2.inRange(imagen_hsv, bajo, alto) para cada rango — devuelve
             una máscara de 0/255 del mismo tamaño que la imagen.
          4. Combinar las dos máscaras con cv2.bitwise_or (un pixel es
             rojo si cae en cualquiera de los dos rangos).
        """
        pass

    def hay_cuadrado_rojo(self) -> bool:
        """
        TODO: usando self.ultima_imagen, decidir si hay un cuadrado rojo
        visible en este momento.

        Pasos sugeridos:
          1. Si self.ultima_imagen es None (todavía no llegó ninguna
             imagen), devolver False.
          2. Convertir la imagen de BGR a HSV con
             cv2.cvtColor(self.ultima_imagen, cv2.COLOR_BGR2HSV).
          3. Llamar a self.mascara_rojo(...) con esa imagen HSV.
          4. Medir self.area_mayor_contorno(...) de esa máscara.
          5. Devolver True si esa área es mayor a self.area_minima_px.
        """
        pass

    def procesar_imagen(self):
        """
        TODO: este es el timer callback, corre a FRECUENCIA_HZ. Es el
        corazón del workshop:

          1. Llamar a self.hay_cuadrado_rojo() para decidir el valor actual.
          2. Si (y solo si) detectado es True, armar un Bool
             (std_msgs.msg.Bool) con .data = True y publicarlo en
             self.publisher_. Si es False, no publicar nada — este tópico
             nunca manda un False, solo avisa cuando hay un cuadrado rojo
             a la vista.
          3. Loguear (self.get_logger().info(...)) solo cuando el valor
             cambia respecto de la última vez (en cualquiera de los dos
             sentidos) — no en cada tick del timer, o el log se vuelve
             ilegible. Guardá el último valor en self.ultimo_valor_publicado
             para poder compararlo la próxima vez.
        """
        pass


def main(args=None):
    rclpy.init(args=args)
    nodo = DetectorColor()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
