import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    def __init__(self):
        super().__init__('talker')

        # TODO 1: creá el publisher. Va a publicar mensajes String en el
        # tópico 'mensaje', con una cola (queue size) de 10.
        # Descomentá la siguiente línea:

        # self.publisher_ = self.create_publisher(String, 'mensaje', 10)

        # TODO 2: creá un timer que llame a self.publicar cada 1.0 segundos.
        # Descomentá la siguiente línea:

        # self.timer = self.create_timer(1.0, self.publicar)

        self.contador = 0

    def publicar(self):
        msg = String()
        msg.data = f'Hola desde el talker, mensaje {self.contador}'

        # TODO 3: publicá el mensaje usando el publisher que creaste en
        # el TODO 1. Descomentá la siguiente línea:

        # self.publisher_.publish(msg)

        self.get_logger().info(f'Publiqué: "{msg.data}"') #Es como un print, pero para ROS 2. Va a aparecer en la consola.
        self.contador += 1


def main(args=None):
    rclpy.init(args=args)
    nodo = Talker()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
