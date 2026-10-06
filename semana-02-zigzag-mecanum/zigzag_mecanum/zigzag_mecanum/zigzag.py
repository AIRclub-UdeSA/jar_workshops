#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


class ZigzagNode(Node):

    def __init__(self):
        super().__init__('zigzag')

        # Publicador de comandos de velocidad en /cmd_vel
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer a 10 Hz: publica cada 0.1 segundos
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.step_count = 0
        self.direction = 1.0

    def timer_callback(self):
        msg = Twist()

        # Mantiene una velocidad de avance constante
        msg.linear.x = 0.2

        # Alterna la dirección lateral cada 20 pasos
        if self.step_count % 20 == 0:
            self.direction *= -1.0

        msg.linear.y = 0.2 * self.direction

        # Mantiene el chasis apuntando al frente
        msg.angular.z = 0.0

        self.publisher_.publish(msg)
        self.step_count += 1


def main(args=None):
    # Sin esto, rclpy se apaga solo al recibir Ctrl+C y el mensaje de
    # frenado de abajo ya no se puede publicar.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ZigzagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Velocidad cero antes de cerrar, para que el robot frene
        node.publisher_.publish(Twist())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
