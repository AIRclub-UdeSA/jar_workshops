# Semana 02 — Zigzag mecanum

## Objetivo

En esta práctica vas a publicar comandos de velocidad en `/cmd_vel` para que
Donatello avance mientras se desplaza de un lado al otro, aprovechando el
movimiento holonómico de sus ruedas mecanum. Al terminar vas a tener un nodo
que publica una trayectoria en zigzag a `10 Hz`: la vas a poder ver en el
simulador, inspeccionar los comandos, y reconocer cómo ROS 2 representa la
orientación del robot.

## Teoría

### Velocidad y movimiento mecanum

ROS 2 usa mensajes [`geometry_msgs/msg/Twist`](https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html)
para indicar la velocidad deseada de una base móvil. Cada mensaje contiene
dos vectores:

```text
Twist
 ├── linear  [x, y, z]  metros por segundo
 └── angular [x, y, z]  radianes por segundo
```

Para un robot que se mueve sobre un plano con `z = 0`, nos interesan tres
componentes:

| Componente | Movimiento | Valor positivo | Valor negativo |
| --- | --- | --- | --- |
| `linear.x` | Longitudinal | Avanza | Retrocede |
| `linear.y` | Lateral | Se desplaza a la izquierda | Se desplaza a la derecha |
| `angular.z` | Giro sobre el eje vertical | Gira en sentido antihorario | Gira en sentido horario |

Una base diferencial puede avanzar y girar, pero no moverse directamente de
costado. En una base mecanum, los rodillos a 45 grados permiten coordinar
las cuatro ruedas para generar movimiento lateral. Por eso `linear.y` puede
ser distinto de `0`.

### Orientación y cuaterniones

Los mensajes de [odometría](https://docs.ros2.org/latest/api/nav_msgs/msg/Odometry.html)
(`/odom`) y las [transformaciones](https://docs.ros.org/en/lyrical/Tutorials/Intermediate/Tf2/Tf2-Main.html)
(`/tf`) representan la orientación con un
[cuaternión](https://docs.ros.org/en/lyrical/Tutorials/Intermediate/Tf2/Quaternion-Fundamentals.html)
`[x, y, z, w]`, no directamente con ángulos de *roll*, *pitch* y *yaw*.

| Representación | Valores | Ventaja | Límite |
| --- | :---: | --- | --- |
| Ángulos de Euler | 3 | Resultan fáciles de interpretar | Pueden sufrir *gimbal lock* y discontinuidades |
| Matriz de rotación | 9 | No tiene singularidades | Usa más valores de los necesarios |
| Cuaternión | 4 | Es compacto y permite interpolaciones suaves | No se interpreta a simple vista |

Para obtener el ángulo de giro sobre el plano, podés convertir el
cuaternión a *yaw*:

```python
import math


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Convierte un cuaternión [x, y, z, w] a yaw en radianes."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)
```

Para esta práctica el valor debería mantenerse cerca del ángulo inicial,
porque el nodo deja `angular.z` en `0`.

## Qué hay que completar

Creá [`zigzag.py`](zigzag_mecanum/zigzag_mecanum/zigzag.py) dentro del
paquete `zigzag_mecanum`:

```python
#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


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
    rclpy.init(args=args)
    node = ZigzagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        stop_msg = Twist()
        node.publisher_.publish(stop_msg)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

El nodo mantiene `linear.x` en `0.2 m/s` y cambia el signo de `linear.y`
cada dos segundos. Como `angular.z` queda en `0`, el chasis conserva la
misma orientación mientras se desplaza.

**Desafío extra:** agregá una velocidad angular suave con
`msg.angular.z = 0.3 * self.direction`. Volvé a ejecutar el nodo y comparé
el resultado: la traslación lateral ahora se combina con el giro del
chasis y produce curvas en lugar de un zigzag paralelo.

## Cómo correrlo

Con `yahboom_rosmaster` clonado y buildeado en `~/rosmaster_ws` (ver su
[README](https://github.com/AIRclub-UdeSA/yahboom_rosmaster)) y el
[Workshop 01](../semana-01-talkers-listeners/) completo:

```bash
# Terminal 1 — build
cd ~/rosmaster_ws
colcon build --packages-select zigzag_mecanum
source install/setup.bash
```

Con el simulador abierto, ejecutá el nodo en otra terminal:

```bash
# Terminal 2 — el nodo
source install/setup.bash
ros2 run zigzag_mecanum zigzag
```

Para detenerlo, volvé a esa terminal y presioná `Ctrl+C`. El nodo publica
un mensaje vacío antes de cerrar para pedir velocidad cero.

Chequeo útil, en otra terminal:

```bash
ros2 topic echo /cmd_vel
```

En el simulador, Donatello debería avanzar mientras alterna el
desplazamiento lateral, con el frente del chasis apuntando siempre en la
misma dirección. Vas a ver `linear.x` constante en `0.2` y `linear.y`
alternando entre `0.2` y `-0.2` cada dos segundos. Si el robot avanza pero
no se desplaza lateralmente, confirmá que el nodo publique en `/cmd_vel` y
que `linear.y` cambie de signo.

En RViz podés agregar una visualización de `Odometry` y seleccionar
`/odom` para revisar la pose y orientación estimadas del robot — RViz no
dibuja por sí solo un rastro histórico del recorrido.
