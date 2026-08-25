import math

import cv2
import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from cv_bridge import CvBridge


def mascara_rojo(imagen_hsv, hue_bajo_1, hue_alto_1, hue_bajo_2, hue_alto_2,
                  saturacion_min, valor_min):
    """Máscara binaria (255 = rojo). Es la misma función que ya escribiste
    en detector.py — la repetimos acá tal cual para que detector_scan sea
    un nodo independiente (no depende de importar detector.py)."""
    bajo1 = np.array([hue_bajo_1, saturacion_min, valor_min])
    alto1 = np.array([hue_alto_1, 255, 255])
    bajo2 = np.array([hue_bajo_2, saturacion_min, valor_min])
    alto2 = np.array([hue_alto_2, 255, 255])
    mascara1 = cv2.inRange(imagen_hsv, bajo1, alto1)
    mascara2 = cv2.inRange(imagen_hsv, bajo2, alto2)
    return cv2.bitwise_or(mascara1, mascara2)


def cuaternion_a_rotacion(x, y, z, w):
    """Matriz de rotación 3x3 equivalente a un quaternion (x, y, z, w).
    Álgebra de siempre, no es el objetivo de este workshop."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def puntos_laser_a_pixeles(rangos, angulos, rotacion, traslacion, fx, fy, cx, cy):
    """
    TODO: el corazón geométrico del workshop. Convertir cada punto del
    lidar (dado como range/angulo) en el pixel (u, v) donde caería si la
    cámara lo estuviera mirando.

    Parámetros: rangos y angulos son arrays de igual largo (uno por rayo
    del lidar); rotacion es una matriz 3x3 y traslacion un vector de 3
    (la tf real que ubica la cámara respecto del lidar); fx, fy, cx, cy son
    los intrínsecos de la cámara (de camera_info).

    Pasos sugeridos:
      1. Pasar cada (rango, ángulo) de polar a cartesiano, en el frame del
         lidar (x = rango*cos(ángulo), y = rango*sin(ángulo), z = 0). Con
         arrays de numpy esto sale vectorizado (sin loop): armá un array
         Nx3 apilando (x, y, z) con np.stack(..., axis=1).
      2. Pasar esos puntos al frame de la cámara aplicando la rotación y
         la traslación: punto_camara = punto_laser @ rotacion.T + traslacion
         (aplicado a los N puntos a la vez, con la misma cuenta).
      3. Extraer x_cam, y_cam, z_cam de ese array Nx3.
      4. Armar "adelante": un array de bool, True donde z_cam > 0.01 (si
         z_cam <= 0, el punto está detrás de la cámara y no tiene sentido
         proyectarlo).
      5. Proyección pinhole: u = fx * x_cam / z_cam + cx,
         v = fy * y_cam / z_cam + cy. Usá np.errstate(divide='ignore',
         invalid='ignore') alrededor de esta cuenta — vas a dividir por
         z_cam aunque tenga ceros o negativos en las posiciones que
         "adelante" después descarta, y no querés que numpy tire warnings
         por eso.
      6. Devolver (u, v, adelante).
    """
    pass


class DetectorScanColor(Node):
    """
    Proyecta cada punto del /scan sobre la imagen de la cámara (usando la
    tf entre el lidar y la cámara, y los intrínsecos de camera_info) y
    publica en /scan_rojo una copia del LaserScan donde solo sobreviven
    los rangos cuyo punto cae, en la imagen, sobre un pixel rojo. El resto
    se pone en infinito (sin retorno), como cualquier LaserScan filtrado.
    """

    def __init__(self):
        super().__init__('detector_scan_color')

        self.declare_parameter('hue_rojo_bajo_1', 0.0)
        self.declare_parameter('hue_rojo_alto_1', 10.0)
        self.declare_parameter('hue_rojo_bajo_2', 170.0)
        self.declare_parameter('hue_rojo_alto_2', 180.0)
        self.declare_parameter('saturacion_min', 120.0)
        self.declare_parameter('valor_min', 80.0)

        self.hue_rojo_bajo_1 = self.get_parameter('hue_rojo_bajo_1').value
        self.hue_rojo_alto_1 = self.get_parameter('hue_rojo_alto_1').value
        self.hue_rojo_bajo_2 = self.get_parameter('hue_rojo_bajo_2').value
        self.hue_rojo_alto_2 = self.get_parameter('hue_rojo_alto_2').value
        self.saturacion_min = self.get_parameter('saturacion_min').value
        self.valor_min = self.get_parameter('valor_min').value

        self.puente = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.ultima_imagen = None
        self.info_camara = None
        self.aviso_tf_mostrado = False

        self.create_subscription(
            Image, 'cam_1/color/image_raw', self.recibir_imagen, 10
        )
        self.create_subscription(
            CameraInfo, 'cam_1/color/camera_info', self.recibir_info_camara, 10
        )
        self.create_subscription(LaserScan, 'scan', self.recibir_scan, 10)

        # TODO: arrancá por acá esta parte del workshop. Creá self.publisher_,
        # el publisher de LaserScan en el tópico 'scan_rojo' (mismo patrón que
        # create_subscription de arriba, pero con create_publisher).

    def recibir_imagen(self, msg: Image):
        self.ultima_imagen = self.puente.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def recibir_info_camara(self, msg: CameraInfo):
        # K es la matriz intrínseca 3x3 en fila: [fx 0 cx; 0 fy cy; 0 0 1]
        self.info_camara = {
            'fx': msg.k[0], 'fy': msg.k[4],
            'cx': msg.k[2], 'cy': msg.k[5],
            'frame_id': msg.header.frame_id,
            'width': msg.width, 'height': msg.height,
        }

    def obtener_transform_laser_a_camara(self, frame_id_scan):
        """Busca en tf2 la transformada del frame del lidar al frame
        óptico de la cámara. None si todavía no está disponible (por
        ejemplo, en el primer instante después de lanzar el nodo)."""
        try:
            return self.tf_buffer.lookup_transform(
                self.info_camara['frame_id'], frame_id_scan, Time()
            )
        except tf2_ros.TransformException as e:
            if not self.aviso_tf_mostrado:
                self.get_logger().warn(f'Todavía no hay tf laser->cámara: {e}')
                self.aviso_tf_mostrado = True
            return None

    def recibir_scan(self, msg: LaserScan):
        """
        TODO: callback del /scan — corre una vez por cada scan que llega.
        Es el segundo corazón del workshop, el que junta todo:

          1. Si todavía no llegó ninguna imagen o ningún camera_info
             (self.ultima_imagen / self.info_camara son None), no hacer
             nada (return).
          2. Pedir la transformada con
             self.obtener_transform_laser_a_camara(msg.header.frame_id).
             Si da None (todavía no está en tf2), return.
          3. De transform.transform sacar la traslación (.translation, con
             .x/.y/.z) y la rotación (.rotation, con .x/.y/.z/.w), y armar
             la matriz de rotación con cuaternion_a_rotacion(...).
          4. Armar los arrays rangos (np.array(msg.ranges)) y angulos
             (msg.angle_min + np.arange(n) * msg.angle_increment, con
             n = len(rangos)).
          5. Armar "validos": un array de bool, True donde el rango es
             finito (np.isfinite) y está entre msg.range_min y
             msg.range_max.
          6. Llamar a puntos_laser_a_pixeles(...) con esos rangos/angulos,
             la rotación, la traslación, y los intrínsecos de
             self.info_camara (fx, fy, cx, cy) — te devuelve (u, v,
             adelante).
          7. Armar "dentro_de_imagen": adelante Y u/v caen dentro del
             ancho/alto de self.info_camara (0 <= u < ancho, 0 <= v < alto).
          8. Convertir self.ultima_imagen a HSV y llamar a mascara_rojo(...)
             con los parámetros de self (hue_rojo_bajo_1, etc.) para
             obtener la máscara de rojo de la imagen actual.
          9. Armar "es_rojo": un array de bool del mismo largo que rangos,
             en False por defecto, y en True donde dentro_de_imagen es
             True Y la máscara vale >0 en el pixel (v, u) correspondiente
             (ojo con el orden: las imágenes se indexan [fila, columna] =
             [v, u], no [u, v]). Tip: primero sacá los índices donde
             dentro_de_imagen es True con np.where(...), y usalos para
             indexar u, v y la máscara de una sola vez (evita tener que
             filtrar por fuera de rango antes de castear a int).
          10. Armar rangos_filtrados con np.where(validos & es_rojo,
              rangos, math.inf) — los puntos que no cumplen las dos
              condiciones quedan en infinito (sin retorno).
          11. Armar un LaserScan nuevo, copiando header, angle_min,
              angle_max, angle_increment, time_increment, scan_time,
              range_min y range_max del mensaje original, y poniendo
              rangos_filtrados.tolist() como ranges. Publicarlo en
              self.publisher_.
        """
        pass


def main(args=None):
    rclpy.init(args=args)
    nodo = DetectorScanColor()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
