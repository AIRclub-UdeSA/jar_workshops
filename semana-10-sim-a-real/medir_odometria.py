import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan

PERIODO_REPORTE_S = 2.0


def yaw_de_quaternion(q):
    """Ángulo de yaw (rotación en el plano) de un quaternion. La misma
    cuenta de las semanas 06 y 07."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def normalizar_angulo(angulo):
    """Lleva un ángulo al rango (-pi, pi]."""
    return math.atan2(math.sin(angulo), math.cos(angulo))


def segundos(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def indice_rayo_hacia_atras(scan: LaserScan) -> int:
    """
    TODO: devolver el índice del rayo del lidar que apunta hacia **atrás del
    robot**.

    El rayo i de un LaserScan tiene ángulo
    `scan.angle_min + i * scan.angle_increment`, medido en el frame del
    lidar (`laser_link`), no en el del robot.

    Pasos sugeridos:
      1. Averiguá cómo está montado el lidar respecto del robot:
         `ros2 run tf2_ros tf2_echo base_footprint laser_link` (con el bag
         corriendo). Mirá la rotación: ¿hacia dónde apunta el ángulo 0 del
         lidar, visto desde el robot?
      2. Con eso, decidí qué ángulo del lidar apunta hacia atrás del robot.
      3. Despejá i de la fórmula de arriba y redondealo a entero.
      4. Probalo: al principio del bag, con el robot quieto, `lidar atrás` tiene
         que dar un valor finito y chico (unos 0,3 m: la pared que el robot
         tiene detrás), no `nan`.
    """
    return 0


class MedirOdometria(Node):
    """
    Compara la odometría de las ruedas contra dos sensores que no dependen
    de las ruedas: el IMU para el giro y el lidar para el avance.

    Cada PERIODO_REPORTE_S segundos (del bag) imprime una línea con:
      - el giro acumulado según /odom y según el IMU, y su cociente;
      - cuánto avanzó el robot según /odom desde donde arrancó;
      - la distancia que mide el lidar hacia atrás del robot.
    """

    def __init__(self):
        super().__init__('medir_odometria')
        self.giro_odom = 0.0          # rad, con signo
        self.giro_imu = 0.0           # rad, con signo
        self.yaw_anterior = None
        self.t_imu_anterior = None
        self.inicio = None            # (x, y) del primer /odom
        self.avance_odom = 0.0        # m, distancia en línea recta desde el inicio
        self.rango_atras = float('nan')
        self.t0 = None
        self.t_ultimo_reporte = None

        self.create_subscription(Odometry, 'odom', self.recibir_odom, 10)
        self.create_subscription(Imu, 'imu/data', self.recibir_imu, 10)
        # El lidar publica en Best Effort, como siempre.
        self.create_subscription(LaserScan, 'scan', self.recibir_scan, qos_profile_sensor_data)

    def recibir_odom(self, msg: Odometry):
        """
        TODO (solo la parte del giro; el avance y los reportes ya están):
        sumar en `self.giro_odom` cuánto giró el robot desde el /odom
        anterior, **con signo**.

          1. Sacá el yaw del mensaje con `yaw_de_quaternion`.
          2. Si `self.yaw_anterior` no es `None`, sumá a `self.giro_odom` la
             diferencia entre el yaw actual y el anterior, pasada por
             `normalizar_angulo`. Sin normalizar, cuando el yaw pasa de 179°
             a -179° sumarías -358° en vez de +2°.
          3. Guardá el yaw actual en `self.yaw_anterior`.

        Ojo: la suma va con signo. Un giro a la izquierda suma y uno a la
        derecha resta. (Pregunta para después: ¿qué pasaría con el cociente
        si sumaras valores absolutos?)
        """
        p = msg.pose.pose.position
        if self.inicio is None:
            self.inicio = (p.x, p.y)
        self.avance_odom = math.hypot(p.x - self.inicio[0], p.y - self.inicio[1])

        # TODO: acá va el giro (pasos 1 a 3 del docstring).

        t = segundos(msg.header.stamp)
        if self.t0 is None:
            self.t0 = t
            self.t_ultimo_reporte = t
        elif t - self.t_ultimo_reporte >= PERIODO_REPORTE_S:
            self.t_ultimo_reporte = t
            self.reportar(t - self.t0)

    def recibir_imu(self, msg: Imu):
        """
        TODO: sumar en `self.giro_imu` cuánto giró el robot según el
        giróscopo.

        El IMU no da un ángulo: da la **velocidad** de giro,
        `msg.angular_velocity.z`, en rad/s. Para tener el ángulo hay que
        integrar: velocidad por el tiempo que pasó desde el mensaje anterior.

          1. Calculá `t = segundos(msg.header.stamp)`.
          2. Si `self.t_imu_anterior` no es `None`, sumá a `self.giro_imu`
             `msg.angular_velocity.z * (t - self.t_imu_anterior)`.
          3. Guardá `t` en `self.t_imu_anterior`.
        """
        pass

    def recibir_scan(self, msg: LaserScan):
        r = msg.ranges[indice_rayo_hacia_atras(msg)]
        self.rango_atras = r if math.isfinite(r) else float('nan')

    def reportar(self, t_bag):
        odom, imu = math.degrees(self.giro_odom), math.degrees(self.giro_imu)
        cociente = imu / odom if abs(odom) > 1.0 else float('nan')
        self.get_logger().info(
            f'bag {t_bag:5.1f} s | giro odom {odom:7.1f}°  IMU {imu:7.1f}°  IMU/odom {cociente:6.3f} '
            f'| avance odom {self.avance_odom:5.3f} m  lidar atrás {self.rango_atras:5.3f} m')


def main(args=None):
    rclpy.init(args=args)
    nodo = MedirOdometria()
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
