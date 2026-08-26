# Semana 05 — Armá tu launch y tu RViz

## Objetivo

Esta semana no tiene un tema de robótica propio: no vamos a escribir ningún
nodo nuevo ni a resolver ningún problema del robot. Vamos a aprender las dos
herramientas que hasta ahora veníamos esquivando a mano, y que de acá en
adelante se usan en todos los workshops:

1. **Launch files** — un archivo de Python que levanta varios nodos a la vez,
   con sus parámetros ya puestos, en vez de abrir tres terminales y tipear tres
   comandos largos cada vez.
2. **Configuraciones de RViz** — un archivo que guarda qué se está mirando
   (el `LaserScan`, la imagen de la cámara, `/scan_rojo`, la pose del robot),
   para no tener que rearmar la vista a mano cada vez que se abre RViz.

La idea es hacer esto **una sola vez, acá**, y después reusarlo. Los workshops
que siguen no van a volver a explicar launch ni RViz: van a decir "agregá tu
nodo al launch" y "agregá este display a tu RViz", y eso va a alcanzar.

Como no hay tema nuevo, lo que vamos a lanzar son los workshops que ya
hicieron: la semana 03 (como ejemplo resuelto) y la semana 04 (el ejercicio).
Hace falta tenerlas completas y andando para que esto tenga sentido.

Todo vive en un paquete nuevo, `launch_rviz`. Es un paquete distinto a los
anteriores en un sentido: no tiene código Python propio, ningún nodo — solo
launch files y configuraciones de RViz. Este tipo de paquete es tan común en
ROS 2 que tiene nombre propio: se lo llama paquete de **bringup** (de "levantar
el sistema"), y en muchos proyectos se lo ve nombrado `algo_bringup` — como
`yahboom_rosmaster_bringup`, que seguramente vieron de pasada al mirar el
simulador. Acá lo llamamos `launch_rviz` para que el nombre siga matcheando el
de la carpeta, como en las semanas anteriores, pero tengan presente el patrón
para cuando lo encuentren en otro proyecto.

---

## Parte A — Launch files

### El problema

Ya lo venían sintiendo desde la semana 03: para probar el evasor hacían falta
tres terminales — el simulador, el nodo, y RViz para poder ver algo — y
encima con `--ros-args -p` largos para pasar cada parámetro. En la semana 04
se puso peor, porque así se corría:

```bash
# Terminal 1
ros2 launch yahboom_rosmaster_gazebo rosmaster_gazebo_fortress.launch.py \
  world:=".../laberinto_simple_obs.world" motion_profile:=ideal
# Terminal 2
ros2 run deteccion_color detector --ros-args -p saturacion_min:=140.0
# Terminal 3
ros2 run deteccion_color detector_scan --ros-args -p saturacion_min:=140.0
# Terminal 4
rviz2
# ...y en RViz, agregar a mano los displays de siempre
```

Cuatro terminales, y encima el `saturacion_min` va repetido en dos de ellas: si
lo calibran y se olvidan de cambiarlo en las dos, los nodos quedan viendo
colores distintos y el bug es de los que cuesta encontrar. Multiplíquenlo por
cada vez que quieran probar algo y se entiende por qué existe `ros2 launch`.

### Qué es un launch file

Un [launch file](https://docs.ros.org/en/humble/Tutorials/Launch/Launch-Main.html)
es un archivo de Python que **describe** qué procesos hay que lanzar. No es
un script que se ejecuta de arriba a abajo: es una descripción que el
sistema de launch lee y después ejecuta por su cuenta.

Tres reglas que lo definen:

- El archivo tiene que terminar en `.launch.py`.
- Adentro tiene que haber una función que se llame **exactamente**
  `generate_launch_description()`, sin argumentos.
- Esa función tiene que devolver un objeto **`LaunchDescription`**: la lista de
  todo lo que hay que lanzar.

Se corre con `ros2 launch <paquete> <archivo>.launch.py`, y ya lo vinieron
usando sin darse cuenta: eso es exactamente lo que hacían con
`ros2 launch yahboom_rosmaster_gazebo rosmaster_gazebo_fortress.launch.py`.

### Anatomía: el ejemplo resuelto

[`evasion.launch.py`](launch_rviz/launch/evasion.launch.py) está completo y
comentado: es el que hay que leer antes de escribir nada. Levanta el
simulador, el evasor de la semana 03 y RViz con un solo comando. Su
esqueleto, sin los detalles, es:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    evasor = Node(
        package='evasion_obstaculos',  # igual que en ros2 run <paquete> ...
        executable='evasor',           # la clave que registraron en entry_points
        name='evasor',                 # con qué nombre se registra en ROS 2
        output='screen',               # sin esto, los logs no aparecen en la terminal
        parameters=parametros_evasor,  # los mismos que antes iban en --ros-args -p
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', config_rviz],  # el -d de un `rviz2 -d archivo.rviz` a mano
        parameters=[{'use_sim_time': True}],
    )
    return LaunchDescription([simulador, evasor, rviz])
```

Cada `Node(...)` es el equivalente declarado de un `ros2 run`. Notar que
`executable` es la clave que registraron en `entry_points` de `setup.py` — es
el mismo nombre, y por eso aquel TODO de las semanas anteriores importaba.
Y que RViz **también es un nodo**: se lanza con el mismo `Node(...)`, con
`package` y `executable` en `'rviz2'`.

Dos cosas que suelen confundir al principio:

- **`LaunchDescription` no es una secuencia de pasos.** No espera a que el
  simulador "termine" para arrancar el evasor y RViz: lanza todo en paralelo.
  Que es justo lo que queremos, porque son procesos independientes que se
  encuentran solos a través de los tópicos.
- **`output='screen'` no es decorativo.** Sin eso los logs van a un archivo y
  parece que el nodo no hace nada.

El archivo real tiene más piezas que este esqueleto — de dónde salen
`simulador`, `parametros_evasor` y `config_rviz` — y están explicadas abajo.

### Parámetros ROS vs argumentos de launch

Son dos cosas distintas que se parecen, y conviene tener clara la diferencia:

| | Argumento de launch | Parámetro ROS |
| --- | --- | --- |
| Quién lo lee | el launch file | el nodo |
| Cómo se declara | `DeclareLaunchArgument('x', default_value='1.0')` | `self.declare_parameter('x', 1.0)` en el nodo |
| Cómo se pasa por consola | `ros2 launch ... x:=2.0` | `ros2 run ... --ros-args -p x:=2.0` |
| Cómo se lee | [`LaunchConfiguration('x')`](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Substitutions.html) | `self.get_parameter('x').value` |

En general se usan juntos: se declara un argumento de launch y se lo enchufa
como parámetro de uno o más nodos. Eso es lo que resuelve el problema del
`saturacion_min` repetido — se declara una vez y se lo pasa a los dos nodos:

```python
parametros_color = [{
    'saturacion_min': ParameterValue(
        LaunchConfiguration('saturacion_min'), value_type=float),
}]
```

**Ojo con `ParameterValue(..., value_type=float)`.** `LaunchConfiguration`
devuelve siempre *texto*, porque viene de la línea de comandos. Si se lo pasan
directo a un nodo que declaró ese parámetro como `float`, el nodo recibe la
cadena `"120.0"` donde esperaba un número y arranca con un error de tipo.
`ParameterValue` con `value_type` es lo que hace la conversión. Los valores
fijos, en cambio, se escriben como números de Python y no necesitan nada:
`'angulo_vision_deg': 90.0`.

### `use_sim_time`: el parámetro que arregla la mitad de los bugs raros

Cuando corre el simulador, hay **dos relojes**: el de la computadora y el de la
simulación, que Gazebo publica en el tópico `/clock` y que puede ir más rápido
o más lento que el real. Un nodo, por defecto, usa el reloj de la computadora.

Si el simulador estampa los mensajes con un reloj y el nodo los interpreta con
otro, los timestamps no coinciden y aparecen síntomas que no parecen tener
nada que ver entre sí: RViz muestra el `LaserScan` parpadeando o directamente
no lo muestra, y `detector_scan` se queja de que no encuentra la transformada
entre el lidar y la cámara (`lookup would require extrapolation into the past`).

La solución es el parámetro `use_sim_time`, que se le pasa a **todos** los
nodos, RViz incluido:

```python
parameters=[{'use_sim_time': True}]
```

Regla simple: **si está corriendo el simulador, va siempre en `True`.**

### Incluir otro launch adentro del tuyo

No hace falta reescribir lo que hace el launch del simulador: se lo puede
meter entero adentro del nuestro con `IncludeLaunchDescription`.

```python
simulador = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(launch_simulador),
    launch_arguments={
        'world': LaunchConfiguration('world'),
        'motion_profile': 'ideal',
        'rviz': 'false',
    }.items(),
)
```

Tres detalles a tener en cuenta:

- `launch_arguments` espera pares clave-valor, de ahí el **`.items()`** al
  final. Sin eso, error.
- Los valores van **siempre como texto**, incluso los booleanos: `'false'`
  entre comillas, no `False` de Python.
- **`rviz:='false'` es importante acá.** El launch del simulador abre su propio
  RViz con su propia configuración. Si no lo apagan, se van a abrir dos RViz:
  el suyo y el nuestro. Este argumento existe justamente para poder traer el
  RViz propio.

### Dónde viven los archivos: `share/`

`get_package_share_directory('launch_rviz')` no devuelve la carpeta donde
ustedes escriben el código, sino la carpeta **instalada**
(`install/launch_rviz/share/launch_rviz/`). Y `colcon build` no copia el
paquete entero ahí: copia solo lo que está declarado en `data_files` de
`setup.py`.

Esto genera el error más desconcertante de este workshop: el archivo está ahí,
delante suyo, lo pueden abrir y editar, pero `ros2 launch` dice que no existe.
Es porque nunca se instaló. Por eso el TODO 1 va primero, y por eso **cada vez
que agreguen o modifiquen un `.launch.py` o un `.rviz` hay que volver a correr
`colcon build`.**

---

## Parte B — RViz

### Lo primero: RViz no es el simulador

Es la confusión más común, y vale la pena sacársela de encima antes de
cualquier otra cosa:

- **Gazebo** simula. Tiene la física, los objetos, el robot; genera los datos.
- **[RViz](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html)**
  solo *dibuja lo que ya está publicado en los tópicos*. No simula
  nada, no crea datos, y no puede mostrar algo que ningún nodo esté publicando.

Consecuencia práctica: si algo no aparece en RViz, hay dos posibilidades bien
distintas, y conviene distinguirlas antes de tocar nada. O el dato no se está
publicando (y el problema está en su nodo — se chequea con `ros2 topic hz`), o
se está publicando pero RViz no lo está dibujando (y el problema es de
configuración de RViz). Cerrar RViz y volver a abrirlo no arregla ninguna de
las dos.

Si RViz todavía no les abre, eso es parte del setup y no de este workshop — lo
cubre la página "Gazebo y RViz" de la
[guía de instalación](https://airclub-udesa.github.io/jar_site/setup/).

### Fixed Frame

Arriba de todo, en **Global Options**, está el **Fixed Frame**: el marco de
referencia contra el que se dibuja todo lo demás. Es lo primero que hay que
revisar cuando "no se ve nada".

Los datos de los sensores vienen cada uno en su propio frame — el `/scan` viene
en `laser_link`, la cámara en su frame óptico — y RViz usa **tf2** para llevar
todo al Fixed Frame. Si no existe una cadena de transformadas entre el frame
del dato y el Fixed Frame, RViz no puede dibujarlo y marca el display en rojo
con un `No transform from [laser_link] to [odom]`.

Los frames que importan en el ROSMASTER simulado son:

| Frame | Qué es |
| --- | --- |
| `odom` | el punto donde arrancó el robot. Fijo respecto del mundo, pero deriva con el tiempo. |
| `base_footprint` / `base_link` | el robot. Se mueve respecto de `odom`. |
| `laser_link` | el lidar, montado sobre el robot. |

En estos workshops el Fixed Frame va en **`odom`**: con `base_link` el robot
queda quieto en el centro y el mundo se mueve alrededor, que es útil para mirar
el scan pero desorienta bastante. Todavía no hay un frame `map` — eso llega
cuando aparezca el workshop de localización.

### Displays

El panel de la izquierda es una lista de **displays**: cada uno dibuja un
tópico. Se agregan con el botón **Add**, y la solapa **By topic** es la más
cómoda porque lista directamente lo que se está publicando ahora mismo, ya
emparejado con el tipo de display que corresponde.

Los que se usan en estos workshops:

| Display | Tópico | Para qué |
| --- | --- | --- |
| `LaserScan` | `/scan` | lo que ve el lidar (semana 03) |
| `LaserScan` | `/scan_cono` | solo los rayos dentro del cono que usa el evasor para decidir choque (semana 03) |
| `LaserScan` | `/scan_rojo` | solo los rayos que dieron contra algo rojo (semana 04) |
| `Image` | `/cam_1/color/image_raw` | lo que ve la cámara (semana 04) |
| `RobotModel` | `/robot_description` | el robot dibujado |
| `TF` | — | los ejes de cada frame; sirve para entender qué está pegado a qué |
| `Grid` | — | el piso, como referencia |

Un truco que se repite en varios de estos workshops: poner **dos** displays
`LaserScan` a la vez, uno con el `/scan` completo (rojo y chico, como ya viene
en [`evasion.rviz`](launch_rviz/rviz/evasion.rviz)) y otro con una versión
filtrada del mismo scan, en otro color y más grande (`Size (m)` más alto,
`Style: Spheres`) — `/scan_cono` en semana 03, `/scan_rojo` en semana 04.
Superpuestos se ve de un vistazo qué subconjunto de rayos está usando el nodo
para decidir algo, sea "hay obstáculo" o "esto es rojo".

Esto no es casualidad ni un agregado cosmético: es la técnica que ya vieron
explicada en semana 03 (sección "Visualizar, no solo loguear" de ese README),
aplicada de nuevo acá en `evasion.rviz`. La idea general vale para cualquier
nodo que decida algo a partir de un subconjunto de datos: republicar ese
subconjunto como su propio tópico y agregarlo a la config de RViz convierte
una decisión que vive escondida adentro del nodo en algo que se puede mirar
mientras el robot se mueve — y así detectar una mala calibración *antes* de
que se note como un comportamiento raro, no después.

### El otro clásico: QoS

Si un tópico *seguro* está publicando (lo confirmaron con `ros2 topic hz`) pero
el display no muestra nada y no da error de transformada, casi siempre es
**[QoS](https://docs.ros.org/en/lyrical/Concepts/Intermediate/About-Quality-of-Service-Settings.html)**.

Los sensores publican con *Reliability* en **Best Effort** ("mandá el dato, y
si se pierde uno, no importa, ya viene el próximo"), mientras que el default de
un display de RViz suele ser **Reliable**. Un subscriber Reliable no se conecta
a un publisher Best Effort: no es un error, simplemente nunca llegan datos.

La solución es abrir el display, desplegar **Topic** y poner *Reliability
Policy* en **Best Effort**. En la config que viene con este workshop
([`evasion.rviz`](launch_rviz/rviz/evasion.rviz)) ya está así para `/scan`
— pueden buscar la línea `Reliability Policy: Best Effort` y ver a qué
display pertenece.

Ojo que esto no es una regla fija de "siempre Best Effort": depende de con
qué QoS publica el nodo del otro lado. `/scan` viene del bridge de Gazebo,
que usa QoS de sensor (Best Effort). `/scan_cono`, en cambio, lo publica
[`evasor.py`](../semana-03-evasion-obstaculos/evasion_obstaculos/evasion_obstaculos/evasor.py)
con `create_publisher(...)` sin tocar el perfil de QoS, así que
queda con el default de rclpy — **Reliable**. Por eso el display de
`/scan_cono` en `evasion.rviz` tiene `Reliability Policy: Reliable`, distinto
del de `/scan`. La regla real es: la Reliability Policy del display tiene que
coincidir con la del publisher, sea cual sea — Best Effort no es más
"correcto", solo es lo que corresponde para datos crudos de sensores.

### Guardar la configuración

Los `.rviz` **no se escriben a mano**: se arman en la GUI y se guardan.

Ahora bien, `File > Save Config` guarda en la config de usuario
(`~/.rviz2/default.rviz`), que es privada de su máquina y no se versiona. Para
que la configuración sea parte del workshop y viaje con el repo, hay que usar
**`File > Save Config As`** y guardarla dentro del **código fuente** del
paquete:

```
semana-05-launch-rviz/launch_rviz/rviz/<nombre>.rviz
```

Y después **`colcon build`**, por lo mismo que la Parte A: si no se instala en
`share/`, el launch no la encuentra.

Aunque no se escriban a mano, sí conviene abrir uno una vez con un editor de
texto: es YAML, y se reconoce sin problema lo que se tocó en la GUI (el
`Fixed Frame: odom`, el `Value: /scan` de cada tópico, los colores en RGB).
Saber leerlos hace que un `.rviz` deje de ser una caja negra, y permite
arreglar a mano un tópico mal escrito sin volver a abrir la GUI.

---

## Qué hay que completar

**El orden importa más que en otros workshops.** Los TODO 1 y 2 van primero: sin
ellos `ros2 launch` no encuentra ningún archivo de este paquete y no se puede
probar nada. En las semanas anteriores los archivos de configuración eran el
último paso; acá son el primero.

### Configuración del paquete (primero)

1. **[`setup.py`](launch_rviz/setup.py) — `data_files`** — instalar las carpetas
   `launch/` y `rviz/` en `share/`. Son dos líneas para descomentar. Notar que
   `entry_points` queda vacío a propósito: este paquete no tiene nodos propios.
2. **[`package.xml`](launch_rviz/package.xml) — `<exec_depend>`** — declarar de
   qué depende este paquete. Van comentadas para descomentar, pero lean el
   comentario: acá va `<exec_depend>` y no `<depend>`, porque este paquete no
   compila contra nada, solo lanza cosas de otros paquetes en tiempo de
   ejecución.

Con esos dos hechos, buildeen y confirmen que el paquete se instaló bien
antes de seguir:

```bash
cd ~/rosmaster_ws
colcon build --packages-select launch_rviz
source install/setup.bash
ros2 launch launch_rviz evasion.launch.py --show-args
```

`--show-args` arma el `LaunchDescription` y lista sus argumentos sin llegar a
lanzar nada — no hace falta tener el simulador arriba para esto. Tienen que
ver `world`, `distancia_choque_m` y `angulo_giro_deg` en la salida. Si dice
que no encuentra el archivo, es el TODO 1.

### Semana 03 — [`evasion.launch.py`](launch_rviz/launch/evasion.launch.py)

Este archivo **ya viene completo**, sin TODOs. Ábranlo y lean los comentarios
de punta a punta antes de seguir: ahí está resuelto y explicado exactamente
el patrón que tienen que reproducir en `deteccion_color.launch.py` — un
`Node(...)` por nodo del workshop, más RViz, todo sumado al
`LaunchDescription` del final.

### Semana 04 — [`deteccion_color.launch.py`](launch_rviz/launch/deteccion_color.launch.py)

Acá sí hay TODOs, y a propósito viene con menos ayuda: la forma es la misma
que en `evasion.launch.py`, así que se escribe mirando ese archivo. Solo
vienen resueltos el simulador y los parámetros compartidos.

3. **El nodo `detector`**, con `parameters=parametros_color`. Mismo patrón que
   el `Node` del evasor en `evasion.launch.py`.
4. **El nodo `detector_scan`**, del mismo paquete y con los mismos parámetros.
5. **RViz**, apuntando a `rviz/deteccion_color.rviz`. Es el mismo `Node` de
   `rviz2` que ya está resuelto en `evasion.launch.py` — solo cambia qué
   `config_rviz` le pasan.
6. **Sumar los tres al `LaunchDescription`.**

Y además de los tres `Node(...)`, esta semana necesita su propia config de
RViz, que todavía no existe:

7. **Armar `rviz/deteccion_color.rviz`** — este archivo **no existe**: lo
   arman ustedes desde la GUI, igual que hicieron con
   [`evasion.rviz`](launch_rviz/rviz/evasion.rviz) — que ya viene armado y
   sirve de referencia de cómo debería quedar. Con el launch corriendo
   (aunque RViz abra vacío y se queje de que no encuentra el archivo),
   agreguen:

   - `Image` en `/cam_1/color/image_raw`
   - `LaserScan` en `/scan`, gris y chico
   - `LaserScan` en `/scan_rojo`, rojo y grande
   - `RobotModel` y `Grid`, y el Fixed Frame en `odom`

   Después `File > Save Config As` →
   `semana-05-launch-rviz/launch_rviz/rviz/deteccion_color.rviz`, y
   `colcon build` de nuevo. A partir de ahí el launch la abre sola.

---

## Cómo correrlo

Con las semanas 03 y 04 completas, y `launch_rviz` buildeado en
`~/rosmaster_ws`:

```bash
# Build (una sola vez, sirve para las dos semanas)
cd ~/rosmaster_ws
colcon build --packages-select launch_rviz
source install/setup.bash
```

Un `Ctrl-C` en la terminal del launch baja todos los procesos de una. Y como
Gazebo tarda unos segundos en levantar, es normal ver al principio algún error
del nodo quejándose de que todavía no llegan datos: si a los ~15 segundos sigue
igual, ahí sí hay algo mal.

### Semana 03 — evasión de obstáculos

```bash
# Una sola terminal: simulador + evasor + RViz
ros2 launch launch_rviz evasion.launch.py
```

Los argumentos de launch se pasan con `nombre:=valor`, igual que se venía
haciendo con `world:=`:

```bash
ros2 launch launch_rviz evasion.launch.py distancia_choque_m:=0.4 angulo_giro_deg:=90.0
```

Para ver qué argumentos acepta sin abrir el archivo:

```bash
ros2 launch launch_rviz evasion.launch.py --show-args
```

Chequeos útiles, en otra terminal:

```bash
ros2 node list                          # tienen que estar todos los nodos del launch
ros2 topic hz /scan                     # ~5 Hz — si no publica, el problema no es de RViz
ros2 topic hz /scan_cono                # solo se publica desde adentro de hay_obstaculo()
ros2 param get /evasor use_sim_time     # tiene que decir True
```

Ese último es el mejor chequeo cuando algo parpadea en RViz o tf2 se queja: si
dice `False`, falta el `use_sim_time` en ese nodo.

### Semana 04 — detección de color

```bash
# Una sola terminal: simulador + detector + detector_scan + RViz
ros2 launch launch_rviz deteccion_color.launch.py
```

```bash
ros2 launch launch_rviz deteccion_color.launch.py saturacion_min:=140.0
```

```bash
ros2 launch launch_rviz deteccion_color.launch.py --show-args
```

Chequeos útiles, en otra terminal:

```bash
ros2 node list                        # tienen que estar todos los nodos del launch
ros2 topic hz /cam_1/color/image_raw  # confirma que la cámara publica
ros2 topic hz /scan_rojo              # solo aparece mientras hay rojo a la vista
```

Reemplacen `<nombre>` por el `name` que le hayan puesto a cada `Node(...)` en
el TODO de `deteccion_color.launch.py` para chequear su `use_sim_time`:

```bash
ros2 param get /<nombre> use_sim_time
```

---

## De acá en adelante

Esto es infraestructura, no un tema cerrado: los workshops que siguen dan por
sentado que existe. Cuando alguno diga *"agregá tu nodo al launch"*, se refiere
a sumar un `Node(...)` a un archivo de `launch_rviz/launch/` como los de acá; y
cuando diga *"agregá este display a tu RViz"*, a sumarlo desde la GUI y volver a
guardar la config con `Save Config As`.

Dos cosas que van a aparecer más adelante y que ya tienen dónde enchufarse: el
`Map` (cuando haya un mapa que mostrar) y el `Pose` del robot en el frame `map`
(cuando llegue el workshop de localización) son, desde el punto de vista de este
workshop, dos displays más y un Fixed Frame distinto.
