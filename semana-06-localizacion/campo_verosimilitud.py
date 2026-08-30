import numpy as np
from scipy import ndimage

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

UMBRAL_OCUPADO = 65


class CampoVerosimilitud(Node):

    def __init__(self):
        super().__init__('campo_verosimilitud')

        # sigma_sensor controla qué tan angosto es el "halo" de probabilidad
        # alrededor de cada obstáculo: valores chicos son más exigentes (una
        # partícula tiene que estar muy cerca de la pared real para pesar
        # bien), valores grandes son más permisivos.
        self.declare_parameter('sigma_sensor', 0.2)
        self.sigma_sensor = self.get_parameter('sigma_sensor').value

        # transient local: si el filtro de partículas arranca después que
        # este nodo, igual recibe el último mapa publicado.
        qos_mapa = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.publisher_ = self.create_publisher(OccupancyGrid, 'likelihood_map', qos_mapa)
        self.subscription = self.create_subscription(
            OccupancyGrid, 'map', self.recibir_mapa, qos_mapa
        )

    def recibir_mapa(self, msg: OccupancyGrid):
        """Convierte el OccupancyGrid de /map en un array 2D, calcula el campo
        de probabilidad, y lo republica con el mismo header/info que el mapa
        de entrada. No decide nada acá — el cálculo vive en
        campo_de_probabilidad()."""
        alto = msg.info.height
        ancho = msg.info.width
        grid = np.array(msg.data, dtype=np.int16).reshape((alto, ancho))

        campo = self.campo_de_probabilidad(grid, msg.info.resolution)

        salida = OccupancyGrid()
        salida.header = msg.header
        salida.info = msg.info
        salida.data = campo.astype(np.int8).flatten().tolist()

        self.publisher_.publish(salida)
        self.get_logger().info(
            f'Campo de verosimilitud publicado (sigma_sensor={self.sigma_sensor}).'
        )

    def campo_de_probabilidad(self, grid: np.ndarray, resolucion: float) -> np.ndarray:
        """
        TODO: dada la grilla de ocupación (mismo formato que /map: 0 = libre,
        100 = ocupado, -1 = desconocido; grid[fila, columna], fila 0 es la
        esquina inferior izquierda), devolver un array del mismo shape con la
        probabilidad (0-100) de que un rayo del lidar termine en cada celda.

        Pasos sugeridos:
          1. Armar una máscara booleana de qué celdas son obstáculo:
             grid >= UMBRAL_OCUPADO (ya importado arriba).
          2. Con esa máscara, calcular para cada celda la distancia (en
             píxeles) a la celda ocupada más cercana. Pista:
             scipy.ndimage.distance_transform_edt(~mascara) — le tenés que
             pasar la máscara negada, porque esa función mide distancia a la
             celda más cercana que es False en el array que le pasás, y acá
             queremos distancia a un obstáculo (True en la máscara).
          3. Convertir esa distancia de píxeles a metros (multiplicar por
             resolucion).
          4. Convertir la distancia en metros a probabilidad con una
             gaussiana centrada en distancia 0: probabilidad =
             exp(-distancia² / (2 * sigma_sensor²)). Da 1.0 en los
             obstáculos mismos (distancia 0) y decae a medida que te alejás.
          5. Escalar a 0-100 (multiplicar por 100) y devolver.
        """
        pass


def main(args=None):
    rclpy.init(args=args)
    nodo = CampoVerosimilitud()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
