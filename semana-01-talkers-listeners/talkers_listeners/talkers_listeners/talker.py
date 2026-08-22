import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'mensaje', 10)
        self.timer = self.create_timer(1.0, self.publicar)
        self.contador = 0

    def publicar(self):
        msg = String()
        msg.data = f'Hola desde el talker, mensaje {self.contador}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publiqué: "{msg.data}"')
        self.contador += 1


def main(args=None):
    rclpy.init(args=args)
    nodo = Talker()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
