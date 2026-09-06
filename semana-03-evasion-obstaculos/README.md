# Semana 03 — Evasión de obstáculos

## Objetivo

Esta semana armás una máquina de estados de 2 estados para que el robot
avance en línea recta, detecte con el lidar que está por chocar, gire un
ángulo fijo, y siga avanzando — repitiendo esto para esquivar los
obstáculos que se cruce en el camino. Al terminar vas a tener a Donatello
moviéndose solo por un mundo con obstáculos, alternando entre avanzar y
girar cada vez que el lidar detecta algo demasiado cerca dentro de un
cono frontal configurable.

Necesitás la [semana 01](../semana-01-talkers-listeners/) (timers) y la
[semana 02](../semana-02-zigzag-mecanum/) (`Twist` y `/cmd_vel`)
completadas — esta semana combina las dos ideas.

## Teoría: máquina de estados

### ¿Por qué una máquina de estados?

El comportamiento que buscamos (avanzar, y cuando corresponda girar)
tiene dos modos claramente distintos, y en cada momento el robot solo
puede estar haciendo uno de los dos. La tentación, sin pensarlo como una
máquina de estados, es ir agregando variables booleanas sueltas
(`girando`, `bloqueado`...) y `if`s repartidos por todo el nodo a medida
que aparecen casos nuevos — funciona al principio, pero rápidamente se
vuelve difícil de leer, difícil de debuggear ("¿por qué el robot está
haciendo esto ahora?") y frágil ante casos que no se pensaron de
antemano.

Pensar el problema como una máquina de estados obliga a responder dos
preguntas separadas: **¿en qué estado estoy?** y **¿qué hace que pase de
uno a otro?** Esto importa especialmente en robótica porque el programa
corre en tiempo real sobre sensores ruidosos y un entorno que no
controlamos del todo — tener un modelo claro y predecible de "qué hace el
robot en cada situación" es lo que permite razonar sobre el
comportamiento y debuggearlo cuando algo no sale como se esperaba.

**Estado**: representa la situación actual del sistema — acá,
`ESTADO_AVANZAR` o `ESTADO_GIRAR`. Los estados son excluyentes: el
sistema está en uno solo a la vez, nunca "un poco en cada uno".

**Transición**: el cambio de un estado a otro, disparado por un evento,
sensor o temporizador — acá, `hay_obstaculo()` lleva de Avanzar a Girar,
y girar lo suficiente vuelve a Avanzar.

![Animación de la máquina de estados: a la izquierda los estados Avanzar y Girar se activan por turnos, a la derecha un robot esquemático avanza, detecta un obstáculo, gira en el lugar y retoma el avance](maquina-estados-animada.svg)

### Cómo se traduce esto a código ROS 2

Para que esto funcione bien arriba de un robot conviene seguir algunas
reglas de diseño:

- **El loop principal va en un timer callback**, no en los callbacks de
  los sensores — si moviéramos el robot directamente desde
  `recibir_scan()`, la frecuencia de movimiento quedaría atada a la
  frecuencia (impredecible) del sensor.
- **Los callbacks de sensores solo actualizan variables**, nunca deciden
  ni mueven el robot — `recibir_scan()` guarda el último
  [`LaserScan`](https://docs.ros2.org/latest/api/sensor_msgs/msg/LaserScan.html),
  `recibir_odom()` guarda el yaw actual (a partir de un
  [`nav_msgs/Odometry`](https://docs.ros2.org/latest/api/nav_msgs/msg/Odometry.html)).
  Toda la lógica vive en un único lugar.
- **La función de transición está separada de la acción** — decidir si
  el estado cambia es una cosa, actuar según el estado ya actualizado es
  otra.
- **Sé verboso**: loguear cada transición ayuda a entender en qué estado
  está el robot cuando el comportamiento no es el esperado.
- **Visualizá, no solo loguees**: además de tomar una decisión,
  republicá la porción de datos que usaste para tomarla en su propio
  topic — acá, qué rayos del lidar caen dentro del cono de detección.
  Verlo en RViz (semana 05) deja confirmar de un vistazo si los
  parámetros están calibrados como pensás, en vez de inferirlo
  indirectamente de que el robot gire donde no esperabas. Es la misma
  idea detrás de `/scan_rojo` en semana 04, y conviene tenerla presente
  como hábito general: cuando algo se decide adentro de un nodo y no se
  ve, es un buen candidato para republicarlo.

`evasor.py` sigue estas reglas: `recibir_scan()` / `recibir_odom()` son
los callbacks que solo tocan variables, y `maquina_de_estados()` es el
timer callback que corre a frecuencia fija — primero decide la
transición, después actúa según el estado ya actualizado.

### QoS: por qué el `/scan` no se suscribe igual que un topic común

En la [semana 01](../semana-01-talkers-listeners/) las suscripciones se
crearon así:

```python
self.create_subscription(String, 'mensaje', self.recibir, 10)
```

Ese `10` es el **tamaño de cola**: cuántos mensajes sin procesar guarda
ROS 2 antes de descartar los más viejos. Pero pasar solo un número
también elige, sin que se note, el resto del perfil de
[**QoS**](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
(*Quality of Service*): las políticas que publisher y subscriber negocian
para poder conectarse. La que importa acá es *reliability*, y el default
de rclpy es **reliable** — reintentar hasta que el mensaje llegue.

Para un `String` a 1 Hz está perfecto. Los sensores no funcionan así: el
lidar publica a 5 Hz y no va a parar nunca, así que si un scan se pierde,
reintentarlo no tiene ningún sentido — ya viene el próximo, y es más
nuevo. Por eso los sensores publican en **best effort** ("mandalo, y si
se pierde, se perdió"), tanto en el simulador como en el ROSMASTER X3
físico.

Y acá está el detalle que conviene ver una vez con alguien al lado: **un
subscriber reliable no se conecta a un publisher best effort.** DDS
considera los perfiles incompatibles y directamente no arma la conexión.
No es un error, no es una excepción, no se cae nada — el callback
simplemente no se llama nunca. El robot se queda quieto y la pantalla no
dice absolutamente nada.

Por eso `evasor.py` suscribe el `/scan` con el perfil de sensores, que
rclpy ya trae armado:

```python
from rclpy.qos import qos_profile_sensor_data
...
self.create_subscription(LaserScan, 'scan', self.recibir_scan, qos_profile_sensor_data)
```

Mirá el `__init__` del nodo: las dos suscripciones están una al lado de
la otra y son distintas a propósito. `/scan` va con
`qos_profile_sensor_data` porque lo publica un sensor; `/odom` va con
`10` porque lo publica `wheel_state_odometry`, un nodo común, en
reliable. La regla no es "los sensores siempre en best effort", es que
**el subscriber tiene que matchear con el publisher del otro lado**, sea
cual sea. Para averiguar con qué QoS publica alguien:

```bash
ros2 topic info /scan --verbose   # mirá el "Reliability" de los publishers
```

Si alguna vez te toca depurar esto, ROS 2 deja una única pista, en el log
del nodo que se quedó sin datos:

```text
[evasor] New publisher discovered on topic 'scan', offering incompatible QoS.
         No messages will be received from it. Last incompatible policy: RELIABILITY
```

Buscar ese `incompatible QoS` es lo primero que conviene hacer cuando un
nodo no recibe nada pero `ros2 topic hz` muestra el tópico publicando
perfecto. En la [semana 05](../semana-05-launch-rviz/) vuelve a aparecer
lo mismo, pero del lado de los displays de RViz.

### El paquete ROS 2: `setup.py` y `package.xml`

Además de la máquina de estados, un paquete `ament_python` necesita dos
archivos de configuración para que ROS 2 sepa cómo buildearlo y
correrlo:

- **[`setup.py`](evasion_obstaculos/setup.py)** es el instalador de
  Python del paquete. La parte que nos importa es `entry_points`: ahí se
  registra qué ejecutables expone el paquete. Si un ejecutable no está
  registrado ahí, `ros2 run <paquete> <ejecutable>` no lo va a encontrar,
  aunque el código esté perfecto.
- **[`package.xml`](evasion_obstaculos/package.xml)** declara las
  dependencias del paquete con `<depend>` — un `<depend>` por cada
  paquete de ROS que el código importa. Sin estos dos, `colcon build`
  puede fallar o el ejecutable no va a existir aunque el código esté
  perfecto.

## Qué hay que completar

[`evasor.py`](evasion_obstaculos/evasion_obstaculos/evasor.py) ya trae
armado todo lo que no es la máquina de estados en sí: los parámetros
ROS, los publishers/subscribers, y algunas funciones de apoyo resueltas
(`normalizar_angulo()`, `iniciar_giro()`, `angulo_girado()` — la
trigonometría de medir cuánto giró el robot con la odometría). Quedan 4
funciones con `TODO`, cada una con una guía en su docstring:

1. **`hay_obstaculo()`** — la percepción: mirar el `LaserScan` y decidir
   si hay algo demasiado cerca dentro del cono frontal. Además del bool,
   publica en `scan_cono` la máscara que usó para decidir (ya resuelto),
   para poder verla en RViz.
2. **`avanzar()`** — un `Twist` que mueve el robot derecho hacia
   adelante.
3. **`girar()`** — un `Twist` que hace girar al robot en el lugar.
4. **`maquina_de_estados()`** — el corazón del workshop: la transición
   (cuándo pasar de `AVANZAR` a `GIRAR` y viceversa) y el despacho a
   `avanzar()` / `girar()` según el estado.

Completalas en ese orden: `hay_obstaculo()` y las dos acciones son
piezas chicas y fáciles de probar por separado, antes de escribir la
máquina de estados que las usa a las tres.

También hay dos `TODO` en los archivos de configuración (ver
[El paquete ROS 2](#el-paquete-ros-2-setuppy-y-packagexml) arriba):
registrar el ejecutable `evasor` en `entry_points` de `setup.py`, y
declarar en `package.xml` las dependencias que usa `evasor.py`.

## Parámetros

Todos son configurables vía `--ros-args -p <nombre>:=<valor>`:

| Parámetro | Default | Qué es |
| --- | --- | --- |
| `angulo_vision_deg` | 60.0 | Ancho total (en grados) del cono frontal donde se busca un obstáculo. |
| `distancia_choque_m` | 0.6 | Distancia (metros) a la que se considera inminente el choque. |
| `angulo_frente_deg` | 180.0 | A qué ángulo del `/scan` corresponde el frente del robot. Depende del montaje del lidar — ver nota abajo. |
| `velocidad_adelante` | 0.3 | Velocidad lineal (m/s) al avanzar. |
| `velocidad_angular` | 1.0 | Velocidad angular (rad/s) al girar. |
| `angulo_giro_deg` | 110.0 | Magnitud fija del giro cada vez que se detecta un obstáculo. |

**Nota sobre `angulo_frente_deg`:** el ángulo 0° de un `LaserScan` es
relativo al frame del sensor (`laser_link`), no al frente del robot. En
Donatello, el lidar está montado con 180° de yaw fijo (ver
`lidar.urdf.xacro` en `yahboom_rosmaster_description`), así que el 0° del
scan apunta para atrás — por eso el default es `180.0` y no `0.0`. Es un
buen ejemplo de por qué conviene revisar siempre el frame de un sensor
antes de asumir que sus ángulos coinciden con los del chasis.

## Cómo correrlo

Con `yahboom_rosmaster` clonado y buildeado en `~/rosmaster_ws` (ver su
[README](https://github.com/AIRclub-UdeSA/yahboom_rosmaster)):

```bash
# Terminal 1 — build
cd ~/rosmaster_ws
colcon build --packages-select evasion_obstaculos
source install/setup.bash
```

`cafe.world` no es el único mundo con obstáculos para esquivar — hay
otros (los `maze_*`, por ejemplo). Para ver cuáles hay instalados:

```bash
cd ~/rosmaster_ws
source install/setup.bash
ls "$(ros2 pkg prefix yahboom_rosmaster_gazebo)/share/yahboom_rosmaster_gazebo/worlds/"
```

Podés elegir cualquiera; acá usamos `cafe.world` como ejemplo porque
tiene muebles a distintas distancias, bueno para probar el cono de
detección:

```bash
# Terminal 2 — simulador (mundo cafe, con obstáculos)
source install/setup.bash
ros2 launch yahboom_rosmaster_bringup rosmaster_x3_sim.launch.py \
  world:="$(ros2 pkg prefix yahboom_rosmaster_gazebo)/share/yahboom_rosmaster_gazebo/worlds/cafe.world" \
  motion_profile:=ideal
```

```bash
# Terminal 3 — nuestro nodo
source install/setup.bash
ros2 run evasion_obstaculos evasor --ros-args \
  -p angulo_vision_deg:=90.0 -p distancia_choque_m:=0.6 -p angulo_giro_deg:=110.0
```

Esperá a ver en la Terminal 2 el mensaje de odometría publicándose antes
de correr el nodo. Usamos `motion_profile:=ideal` (sin resbalamiento de
ruedas) mientras se prueba la lógica; una vez que funciona, probar con
el default (`motion_profile:=stress`, más realista) es un buen próximo
paso.

Chequeos útiles en una cuarta terminal:

```bash
ros2 topic hz /scan        # ~5 Hz
ros2 topic hz /odom        # ~30 Hz
ros2 topic hz /scan_cono   # solo se publica desde adentro de hay_obstaculo()
```

Donatello debería avanzar en línea recta hasta acercarse a un obstáculo,
girar el ángulo configurado, y retomar el avance — repitiendo el patrón
por todo el mundo. Los logs de transición te muestran en qué estado está
en cada momento. Si el robot gira antes de tiempo o choca igual, aislá
con logs si el problema está más probablemente en `hay_obstaculo()`
(percepción) o en `maquina_de_estados()` (transición).

En el workshop de la [semana 05](../semana-05-launch-rviz/) vamos a
poder ver el cono de detección dibujado (además de chequearlo por
tópico) en RViz.
