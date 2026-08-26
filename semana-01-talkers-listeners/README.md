# Semana 01 — Talkers y listeners

## Objetivo

Este es el primer workshop, pensado para alguien que nunca tocó ROS 2. La
idea es armar el ejemplo más chico posible que igual use las piezas que se
van a repetir en todos los workshops que siguen: un **nodo** que publica
algo a frecuencia fija (`talker.py`) y otro que lo escucha y lo loguea
(`listener.py`).

No hay lidar, ni cámara, ni robot moviéndose — a propósito. La meta acá no
es resolver un problema de robótica, es entender cómo se comunican dos
programas de ROS 2 entre sí, sin que un sensor real complique el ejemplo.

## Teoría

### ¿Qué es un nodo?

Un [**nodo**](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html) es, ni más ni menos, un programa que corre dentro de ROS 2 y
que puede hablar con otros nodos. En un robot real puede haber decenas de
nodos corriendo a la vez: uno que lee el lidar, otro que lee la cámara,
otro que decide para dónde moverse, otro que mueve las ruedas — cada uno
es un proceso separado, y ROS 2 es la capa que les permite intercambiar
información sin que cada uno tenga que saber cómo están escritos los
demás.

En este workshop van a escribir dos nodos: `Talker` y `Listener`, cada uno
heredando de la clase `Node` de [`rclpy`](https://docs.ros2.org/latest/api/rclpy/) (la librería de ROS 2 para
Python). Eso es lo mínimo que necesita cualquier nodo:

```python
class Talker(Node):
    def __init__(self):
        super().__init__('talker')   # el nombre con el que este nodo se registra en ROS 2
```

### Tópicos, publishers y subscribers

Los nodos no se hablan directamente entre sí (el talker no conoce al
listener, ni falta que le haga). Se comunican a través de [**tópicos**](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html): un
canal con nombre (acá, `'mensaje'`) al que unos nodos **publican** datos y
otros se **suscriben** para recibirlos. Puede haber cero, uno o varios
publishers y subscribers en el mismo tópico, y no se necesitan entre sí
para funcionar — el talker publica igual aunque no haya ningún listener
escuchando.

Cada tópico tiene un **tipo de mensaje** fijo — acá usamos
[`std_msgs/String`](https://docs.ros2.org/latest/api/std_msgs/msg/String.html), el tipo de mensaje más simple que hay (un solo campo,
`data`, con texto). Todo tópico tiene un tipo de mensaje: como van a ver en
workshops que vienen, un lidar publica `LaserScan`, una cámara publica
`Image`, etc. Publisher y subscriber tienen que declarar el mismo tipo
para el mismo tópico, si no ROS 2 no los deja conectarse.

- **Publisher** (`talker.py`): se crea una vez, en `__init__`, con
  `create_publisher(TipoDeMensaje, 'nombre_del_topico', tamaño_de_cola)`.
  Después, en cualquier momento, se le puede pedir que publique un mensaje
  con `.publish(msg)`.
- **Subscriber** (`listener.py`): se crea con
  `create_subscription(TipoDeMensaje, 'nombre_del_topico', callback, tamaño_de_cola)`.
  El `callback` es una función propia del nodo que ROS 2 llama
  automáticamente cada vez que llega un mensaje nuevo — nunca lo llaman
  ustedes a mano.

El `tamaño_de_cola` (acá, `10`) es cuántos mensajes sin procesar guarda
ROS 2 como máximo antes de empezar a descartar los más viejos. Con un
mensaje por segundo como acá, prácticamente nunca se llena; en sensores
más rápidos importa más.

### El timer callback

El talker no publica "cuando quiere", publica a un ritmo fijo. Eso se
logra con un **timer**:

```python
self.timer = self.create_timer(1.0, self.publicar)
```

Esto le dice a ROS 2 "llamá a `self.publicar` cada 1.0 segundos, para
siempre, mientras el nodo esté vivo". `publicar` es el **timer callback**:
ahí es donde vive la lógica real del nodo (armar el mensaje, publicarlo,
loguearlo).

Usar un timer en vez de, por ejemplo, un `while True` con un `sleep(1.0)`
adentro es lo que se va a repetir en todos los workshops siguientes:
un timer le deja a ROS 2 el control de cuándo se ejecuta cada cosa, y
permite que el mismo nodo tenga varios timers y varios callbacks
corriendo "en paralelo" (más precisamente, intercalados) sin pisarse.

### Una regla importante para más adelante: los callbacks de sensores solo guardan datos

En este workshop el callback del listener (`recibir`) no hace nada más
que loguear — no hay ninguna decisión que tomar todavía, así que no hace
falta separar nada. Pero vale la pena adelantar una regla que sí va a
importar desde la semana 03 en adelante, cuando aparezcan sensores reales
(lidar, cámara):

**El callback de un sensor solo debería guardar el último dato recibido
en una variable — nunca decidir ni actuar directamente ahí adentro.** La
lógica que decide qué hacer con esos datos va en un timer callback aparte,
que corre a una frecuencia fija y conocida. La razón es que los sensores
publican a frecuencias distintas e impredecibles entre sí (un lidar y una
cámara no van al mismo ritmo); si el robot actuara directamente desde
cada callback de sensor, su comportamiento terminaría atado a esas
frecuencias en vez de a una sola, controlada por ustedes.

Guarden esta idea para cuando lleguen a la semana 03 — ahí van a ver
`recibir_scan()` (que solo guarda el `LaserScan`) separado de
`maquina_de_estados()` (el timer callback que decide y actúa).

### El paquete ROS 2: `setup.py` y `package.xml`

Todo nodo de Python vive adentro de un **paquete** ROS 2 (acá,
`talkers_listeners`), y todo [paquete `ament_python`](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html) necesita dos archivos
de configuración para que ROS 2 sepa cómo instalarlo y correrlo. En este
workshop ya vienen completos — alcanza con mirarlos una vez para entender
qué hace cada uno, porque de acá en adelante los van a tener que tocar
ustedes:

- **[`package.xml`](talkers_listeners/package.xml)** declara, entre otras
  cosas, las dependencias del paquete con `<depend>` — acá `rclpy` (la
  librería de ROS 2 para Python) y `std_msgs` (el paquete de mensajes que
  incluye `String`). Por cada paquete de ROS que se importa en el código,
  tiene que haber un `<depend>` acá; si falta uno, `colcon build` puede
  fallar.
- **[`setup.py`](talkers_listeners/setup.py)** es el instalador de Python
  del paquete. Lo que más importa es `entry_points`: ahí se registra qué
  ejecutables expone el paquete y a qué función de qué archivo apuntan.
  Fíjense que `talker` y `listener` están registrados ahí apuntando a
  `talker:main` y `listener:main` — eso es lo que hace que
  `ros2 run talkers_listeners talker` funcione. Si un ejecutable no está
  en `entry_points`, `ros2 run` no lo va a encontrar, aunque el código
  esté perfecto.

## Qué hay que completar

Los dos archivos, [`talker.py`](talkers_listeners/talkers_listeners/talker.py)
y [`listener.py`](talkers_listeners/talkers_listeners/listener.py), tienen
`TODO`s muy simples: son líneas ya escritas, comentadas, que solo hay que
descomentar (sacarles el `#` de adelante) en el lugar indicado. No hay que
escribir código nuevo — la idea es que el primer contacto sea leer y
entender qué hace cada línea, no pelearse con la sintaxis.

**Importante: háganlo en este orden, probando cada uno antes de pasar al
siguiente.**

1. **`talker.py` primero.** Tiene 3 TODOs: crear el publisher, crear el
   timer, y publicar el mensaje dentro de `publicar()`. Una vez
   completado, buildeen y corran *solo* el talker (ver "Cómo correrlo"
   abajo) y confirmen con `ros2 topic echo /mensaje` que efectivamente
   está publicando, antes de tocar el listener.
2. **`listener.py`, después.** Tiene 1 TODO: suscribirse al tópico
   `'mensaje'`. Con el talker ya andando en una terminal, corran el
   listener en otra y confirmen en sus logs que va recibiendo cada
   mensaje.

## Cómo correrlo

Con el workspace ROS ya armado (ver la
[guía de setup](https://airclub-udesa.github.io/jar_site/setup/simulador/)
si todavía no lo tienen) — este workshop no necesita el simulador ni el
robot, corre standalone:

```bash
# Terminal 1 — build
cd ~/rosmaster_ws
colcon build --packages-select talkers_listeners
source install/setup.bash
```

```bash
# Terminal 2 — el talker (probar esto solo, primero)
source install/setup.bash
ros2 run talkers_listeners talker
```

Chequeo útil en una tercera terminal, mientras el talker corre solo:

```bash
ros2 topic echo /mensaje    # tienen que aparecer mensajes nuevos cada 1 segundo
```

Una vez confirmado que el talker publica, y con `listener.py` ya
completado:

```bash
# Terminal 3 — el listener
source install/setup.bash
ros2 run talkers_listeners listener
```

Con las dos terminales corriendo a la vez, cada mensaje que loguea el
talker en la Terminal 2 debería aparecer logueado como recibido en la
Terminal 3, un segundo después.
