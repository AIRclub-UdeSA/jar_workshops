import math

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

FRECUENCIA_HZ = 10.0

ESTADO_AVANZAR = 0
ESTADO_GIRAR = 1


class Evasor(Node):

    def __init__(self):
        super().__init__('evasor')

        # Cuántos grados a cada lado del frente mira el sensor para decidir si hay
        # un obstáculo (cono de visión total = angulo_vision_deg).
        self.declare_parameter('angulo_vision_deg', 60.0)
        # A qué distancia (metros) se considera que el choque es inminente.
        self.declare_parameter('distancia_choque_m', 0.6)
        # Ángulo del /scan que corresponde al frente del robot. Depende del
        # montaje del lidar; en el rosmaster_x3 simulado el joint de laser_link
        # tiene 180 grados de yaw fijo (ver lidar.urdf.xacro), así que el 0 del
        # /scan apunta para atrás y hay que compensarlo con 180.0.
        self.declare_parameter('angulo_frente_deg', 180.0)
        self.declare_parameter('velocidad_adelante', 0.3)
        # Signo y magnitud de la velocidad angular con la que se gira. El signo
        # define el sentido: siempre se gira para el mismo lado (positivo =
        # antihorario / izquierda, negativo = horario / derecha).
        self.declare_parameter('velocidad_angular', 1.0)
        # Magnitud fija del giro cada vez que se detecta un obstáculo.
        self.declare_parameter('angulo_giro_deg', 110.0)

        self.angulo_vision_deg = self.get_parameter('angulo_vision_deg').value
        self.distancia_choque_m = self.get_parameter('distancia_choque_m').value
        self.angulo_frente_deg = self.get_parameter('angulo_frente_deg').value
        self.velocidad_adelante = self.get_parameter('velocidad_adelante').value
        self.velocidad_angular = self.get_parameter('velocidad_angular').value
        self.angulo_giro_deg = self.get_parameter('angulo_giro_deg').value

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan, 'scan', self.recibir_scan, 10
        )
        # Odometría real (integrada por wheel_state_odometry.py a partir de
        # /joint_states, no lo que se comanda). Se usa para medir cuánto giró
        # efectivamente el robot, en vez de asumir un tiempo fijo.
        self.subscription_odom = self.create_subscription(
            Odometry, 'odom', self.recibir_odom, 10
        )
        self.timer = self.create_timer(1.0 / FRECUENCIA_HZ, self.maquina_de_estados)

        self.ultimo_scan = None
        self.yaw_actual = None
        self.estado = ESTADO_AVANZAR
        self.yaw_inicial_giro = 0.0

    def recibir_scan(self, msg: LaserScan):
        self.ultimo_scan = msg

    def recibir_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.yaw_actual = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def normalizar_angulo(self, angulo: float) -> float:
        """Lleva un ángulo en radianes al rango [-pi, pi]."""
        return math.atan2(math.sin(angulo), math.cos(angulo))

    def hay_obstaculo(self) -> bool:
        """
        TODO: devolver True si hay algo más cerca que distancia_choque_m
        dentro del cono frontal (angulo_frente_deg +/- angulo_vision_deg/2).

        Pasos sugeridos (con numpy, self.ultimo_scan es un LaserScan):
          1. Si no llegó ningún scan, es decir es None, devolver False
          2. Armar un array con el ángulo de cada medición
          3. Restarle a cada ángulo el centro del cono y normalizar el
             resultado al rango [-pi, pi] (podés usar self.normalizar_angulo,
             o hacerlo vectorizado)
          4. Armar una máscara para las mediciones que caen dentro del cono,
             y otra para las que están más cerca que la distancia de choque.
          5. Combinar ambas máscaras y devolver si hay al menos una medición
             que las cumple a la vez.
        """
        pass

    def iniciar_giro(self):
        """Guarda el yaw actual como referencia para saber, más adelante,
        cuánto giró realmente el robot (medido con la odometría)."""
        self.yaw_inicial_giro = self.yaw_actual

    def angulo_girado(self) -> float:
        """Cuánto giró el robot (en radianes, siempre positivo) desde que
        empezó el giro actual, según la odometría."""
        return abs(self.normalizar_angulo(self.yaw_actual - self.yaw_inicial_giro))

    def avanzar(self) -> Twist:
        """TODO: devolver un Twist que mueva el robot derecho hacia
        adelante, a velocidad_adelante (m/s)."""
        msg = Twist()
        # TODO: completar el campo de avance
        return msg

    def girar(self) -> Twist:
        """TODO: igual que avanzar(), pero para girar en el lugar,
        a velocidad_angular (rad/s)."""
        msg = Twist()
        # TODO: completar el campo de giro
        return msg

    def maquina_de_estados(self):
        """
        TODO: esta función es el timer callback, corre a FRECUENCIA_HZ. Es
        la máquina de estados completa: primero decide si hay que cambiar
        de estado (transición), después actúa según el estado ya
        actualizado, y publica.

        1. Si estoy en AVANZAR, ver si corresponde pasar a GIRAR. Al
           entrar a GIRAR, guardar la referencia para poder medir después
           cuánto giré.
        2. Si estoy en GIRAR, ver si ya giré lo suficiente para volver a
           AVANZAR.
        3. Según el estado ya actualizado, armar el Twist llamando a la
           función que corresponda.
        4. Publicar el Twist.

        Tip: loguear cada transición ayuda a debuggear en qué estado está
        el robot en cada momento.
        """

        # Transición de estados

        if self.estado == ESTADO_AVANZAR:
            pass
        if self.estado == ESTADO_GIRAR:
            pass

        
        # Acción según el estado
        
        if self.estado == ESTADO_AVANZAR:
            msg = self.avanzar()
        elif self.estado == ESTADO_GIRAR:
            msg = self.girar()

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    nodo = Evasor()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
