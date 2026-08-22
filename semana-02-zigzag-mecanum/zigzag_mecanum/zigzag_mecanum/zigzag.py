import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

FRECUENCIA_HZ = 10.0
VELOCIDAD_ADELANTE = 0.3  # m/s, eje x
VELOCIDAD_LATERAL = 0.3  # m/s, eje y
PERIODO_ZIGZAG = 2.0  # segundos entre cada cambio de lado


class Zigzag(Node):
    def __init__(self):
        super().__init__('zigzag')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(1.0 / FRECUENCIA_HZ, self.publicar)
        self.tiempo = 0.0
        self.lado = 1.0

    def publicar(self):
        self.tiempo += 1.0 / FRECUENCIA_HZ
        if self.tiempo >= PERIODO_ZIGZAG:
            self.tiempo = 0.0
            self.lado *= -1.0

        msg = Twist()
        msg.linear.x = VELOCIDAD_ADELANTE
        msg.linear.y = VELOCIDAD_LATERAL * self.lado
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    nodo = Zigzag()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
