# Semana 01 — Talkers y listeners

## Objetivo

Este es el primer workshop, pensado para alguien que nunca tocó ROS 2. La
idea es armar el ejemplo más chico posible que igual use las piezas que se
van a repetir en todos los workshops que siguen: un nodo que publica algo
a frecuencia fija (`talker.py`) y otro que lo escucha y lo loguea
(`listener.py`).

No hay lidar, ni cámara, ni robot moviéndose — a propósito: la meta acá no
es resolver un problema de robótica, es entender cómo se comunican dos
programas de ROS 2 entre sí, sin que un sensor real complique el ejemplo.
El talker y el listener no se llaman entre sí: los dos se conectan al
mismo topic.

## Teoría

### ¿Qué es un nodo?

Un [**nodo**](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)
es un programa que corre dentro de ROS 2 y puede hablar con otros nodos.
En un robot real puede haber decenas corriendo a la vez — uno que lee el
lidar, otro la cámara, otro que mueve las ruedas — cada uno un proceso
separado, y ROS 2 es la capa que les permite intercambiar información sin
que cada uno tenga que saber cómo están escritos los demás.

En este workshop van a escribir dos nodos: `Talker` y `Listener`, cada uno
heredando de la clase `Node` de [`rclpy`](https://docs.ros2.org/latest/api/rclpy/)
(la librería de ROS 2 para Python). Eso es lo mínimo que necesita
cualquier nodo:

```python
class Talker(Node):
    def __init__(self):
        super().__init__('talker')   # el nombre con el que este nodo se registra en ROS 2
```

### Tópicos, publishers y subscribers

Los nodos no se conocen entre sí directamente: se comunican a través de
[**topics**](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html),
canales con nombre (acá, `mensaje`) a los que unos publican y otros se
suscriben, sin necesitarse mutuamente para funcionar.

Cada topic tiene un tipo de mensaje fijo — usamos
[`std_msgs/String`](https://docs.ros2.org/latest/api/std_msgs/msg/String.html),
el más simple que hay. Publisher y subscriber tienen que declarar el mismo
tipo para el mismo topic, o ROS 2 no los deja conectarse.

- **Publisher** (`talker.py`): se crea una vez, en `__init__`, con
  `create_publisher(TipoDeMensaje, 'nombre_del_topico', tamaño_de_cola)`.
  Después, en cualquier momento, se le puede pedir que publique un mensaje
  con `.publish(msg)`.
- **Subscriber** (`listener.py`): se crea con
  `create_subscription(TipoDeMensaje, 'nombre_del_topico', callback, tamaño_de_cola)`.
  El `callback` es una función propia del nodo que ROS 2 llama
  automáticamente cada vez que llega un mensaje nuevo — nunca lo llaman
  ustedes a mano.

El tamaño de cola (acá, `10`) es cuántos mensajes sin procesar guarda
ROS 2 como máximo antes de descartar los más viejos; con un mensaje por
segundo casi nunca se llena, pero en sensores más rápidos importa más.

### El timer callback

El talker no publica "cuando quiere": publica a un ritmo fijo con un
**timer**.

```python
self.timer = self.create_timer(1.0, self.publicar)
```

Esto le dice a ROS 2 "llamá a `self.publicar` cada 1.0 segundos mientras
el nodo esté vivo". Usar un timer en vez de, por ejemplo, un `while True`
con `time.sleep(1.0)` es lo que se repite en todos los workshops
siguientes: le deja a ROS 2 el control de cuándo se ejecuta cada cosa, y
permite que un mismo nodo tenga varios timers y callbacks corriendo
intercalados sin pisarse.

### Una regla importante para más adelante

Desde la semana 03 vamos a usar una convención: el callback de cada
sensor guarda el último dato recibido y un timer separado toma las
decisiones a una frecuencia conocida. No es la única arquitectura posible
en ROS 2, pero nos permite combinar sensores que publican a ritmos
diferentes sin atar el comportamiento del robot a uno de ellos. Guarden
esta idea para cuando lleguen a la semana 03 — ahí van a ver
`recibir_scan()` (que solo guarda el `LaserScan`) separado de
`maquina_de_estados()` (el timer callback que decide y actúa).

### El paquete ROS 2: `setup.py` y `package.xml`

Todo paquete [`ament_python`](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
necesita dos archivos de configuración para que ROS 2 sepa cómo instalarlo
y correrlo. En este workshop ya vienen completos — alcanza con mirarlos
una vez para entender qué hace cada uno, porque de acá en adelante los van
a tener que tocar ustedes:

- **[`package.xml`](talkers_listeners/package.xml)** declara las
  dependencias del paquete con `<depend>` — acá `rclpy` y `std_msgs`. Por
  cada paquete de ROS que se importa en el código, tiene que haber un
  `<depend>` acá; si falta uno, `colcon build` puede fallar.
- **[`setup.py`](talkers_listeners/setup.py)** es el instalador de Python
  del paquete. Lo que más importa es `entry_points`: ahí se registra qué
  ejecutables expone el paquete y a qué función de qué archivo apuntan —
  eso es lo que hace que `ros2 run talkers_listeners talker` funcione. Si
  un ejecutable no está en `entry_points`, `ros2 run` no lo va a
  encontrar, aunque el código esté perfecto.

## Qué hay que completar

Los dos archivos, [`talker.py`](talkers_listeners/talkers_listeners/talker.py)
y [`listener.py`](talkers_listeners/talkers_listeners/listener.py), tienen
comentarios `TODO` simples: líneas ya escritas, comentadas, que solo hay
que descomentar en el lugar indicado. No hay que escribir código nuevo —
la idea es que el primer contacto sea leer y entender qué hace cada línea.

**Háganlo en este orden, probando cada uno antes de pasar al siguiente:**

1. **`talker.py` primero.** Tiene 3 comentarios `TODO`: crear el
   publisher, crear el timer, y publicar el mensaje dentro de
   `publicar()`. Compilá y corré *solo* el talker, y confirmá con
   `ros2 topic echo /mensaje` que está publicando antes de tocar el
   listener.
2. **`listener.py`, después.** Tiene 1 `TODO`: suscribirse al topic
   `mensaje`. Con el talker ya andando en una terminal, corré el listener
   en otra y confirmá en sus logs que va recibiendo cada mensaje.

En `talker.py` vas a habilitar estas tres líneas:

```python
self.publisher_ = self.create_publisher(String, 'mensaje', 10)
self.timer = self.create_timer(1.0, self.publicar)
self.publisher_.publish(msg)
```

En `listener.py`, vas a habilitar la suscripción:

```python
self.subscription = self.create_subscription(
    String, 'mensaje', self.recibir, 10
)
```

**Desafío extra:** modificá `listener.py` para que en vez de suscribirse a
`mensaje` escuche `/cmd_vel` (tipo `geometry_msgs/Twist`) y loguee
`linear.x`, `linear.y` y `angular.z` cada vez que llega un mensaje. Agregá
también `<depend>geometry_msgs</depend>` a `package.xml`, porque el nodo
ahora importa ese paquete. Volvé a compilar, cargá el overlay y corré tu
listener modificado mientras manejás con la teleoperación por teclado —
confirmá que ves los mismos valores que en `ros2 topic echo /cmd_vel`.

## Cómo correrlo

`rclpy` y `std_msgs` ya vienen con `ros-humble-desktop`, así que no hace
falta instalar nada extra esta semana. Si todavía no clonaste el repo de
workshops dentro de tu workspace (junto a `yahboom_rosmaster`):

```bash
cd ~/rosmaster_ws/src
git clone https://github.com/AIRclub-UdeSA/jar_workshops.git
```

Este workshop no necesita el simulador ni el robot, corre standalone:

```bash
# Terminal 1 — build
cd ~/rosmaster_ws
colcon build --symlink-install --packages-select talkers_listeners
source install/setup.bash
```

```bash
# Terminal 2 — el talker (probalo solo, primero)
source install/setup.bash
ros2 run talkers_listeners talker
```

```bash
# Terminal 3 — chequeo mientras el talker corre solo
source install/setup.bash
ros2 topic echo /mensaje    # tienen que aparecer mensajes nuevos cada 1 segundo
```

Una vez confirmado que el talker publica, detené `echo` con `Ctrl+C`. Con
`listener.py` ya completado, reutilizá esa terminal:

```bash
# Terminal 3 — el listener
source install/setup.bash
ros2 run talkers_listeners listener
```

Con las dos terminales corriendo a la vez, cada mensaje del talker debería
aparecer en el listener apenas se publica: un par nuevo por segundo,
`Publiqué: "Hola desde el talker, mensaje N"` de un lado y
`Recibí: "..."` del otro, con `N` incrementando.

Chequeos útiles en una cuarta terminal, mientras los dos nodos siguen
activos:

```bash
source install/setup.bash
ros2 topic list
ros2 topic info /mensaje
ros2 topic hz /mensaje
```

`ros2 topic list` debería incluir `/mensaje`; `ros2 topic info` debería
encontrar un publisher y una suscripción; y `ros2 topic hz` debería medir
una frecuencia cercana a `1 Hz`.

Cerrá el talker y dejá el listener corriendo: el topic sigue declarado
por el suscriptor, pero deja de recibir datos hasta que aparece otro
publisher — se puede confirmar con `ros2 topic info /mensaje`.

### El mismo patrón con Donatello

Con el simulador levantado (ver la
[guía del simulador](https://airclub-udesa.github.io/jar_site/setup/simulador/#6-moverlo-desde-el-teclado)),
abrí otra terminal para la teleoperación por teclado:

```bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

En otra terminal, inspeccioná el topic real del robot mientras manejás:

```bash
source install/setup.bash
ros2 topic echo /cmd_vel
```

Es el mismo patrón que tu talker y listener, pero con un topic real:
`teleop_twist_keyboard` publica `geometry_msgs/Twist` en `/cmd_vel`, y el
controlador de Donatello lo escucha para mover las ruedas. Fijate cómo
cambian `linear.x`, `linear.y` y `angular.z` según las teclas que uses —
el publisher no sabe ni le importa que del otro lado hay ruedas de
verdad, es el mismo desacople que viste entre tu talker y tu listener.
