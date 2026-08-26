# Semana 04 — Detección de color

## Objetivo

El workshop tiene dos partes, cada una con su propio nodo:

1. **[`detector.py`](deteccion_color/deteccion_color/detector.py)** — mirar
   la cámara del robot, reconocer cuando hay un cuadrado rojo en la imagen
   (y distinguirlo de uno azul) usando OpenCV, y avisar en un tópico
   (`rojo_detectado`, `std_msgs/Bool`) cada vez que lo ve.
2. **[`detector_scan.py`](deteccion_color/deteccion_color/detector_scan.py)**
   — un segundo nodo que reutiliza la detección de
   color de la Parte 1, pero en vez de avisar "veo rojo" sin más, proyecta
   cada punto del lidar (`/scan`) sobre la imagen de la cámara para saber
   **cuáles** de esos puntos caen sobre algo rojo. Publica en `/scan_rojo`
   un `LaserScan` filtrado, con distancia y ángulo reales al cuadrado —
   el dato que hace falta para, por ejemplo, navegar hacia él.

Hacé primero la Parte 1 completa (código andando) antes de arrancar la
Parte 2 — la Parte 2 reutiliza código de la Parte 1 y da por sentado que ya
la resolviste.


---

## Parte 1 — `detector.py`

### Teoría: por qué HSV y no RGB

#### El problema de detectar "rojo" en RGB

En una imagen RGB, el rojo no es "un color", es una región difusa del
espacio de 3 dimensiones (R, G, B) que además se mueve mucho con la
iluminación: un rojo bajo luz de sombra tiene valores de R, G y B muy
distintos a ese mismo rojo bajo luz directa. Filtrar por rangos de R, G y B
por separado tiende a fallar apenas cambia la luz.

[**HSV**](https://docs.opencv.org/4.x/de/d25/imgproc_color_conversions.html) (Hue/Matiz, Saturation/Saturación, Value/Brillo) separa el "qué
color es" (H) de "qué tan intenso" (S) y "qué tan claro/oscuro" (V). Esto
importa porque la luz cambia sobre todo S y V, y deja H relativamente
estable — así que filtrar por un rango de H es mucho más robusto a la
iluminación que filtrar por RGB.

#### El rojo parte el círculo de tonos

H se mide en grados alrededor de un círculo (0°-360°, que OpenCV comprime a
0-179 para que entre en un byte). El rojo está en 0°, así que un rojo "puro"
puede aparecer tanto en tonos cercanos a 0 como en tonos cercanos a 180
(que es lo mismo que -0° dando la vuelta completa). Por eso, para capturar
todos los rojos hacen falta **dos rangos de H** (uno pegado a 0, otro pegado
a 180), y no alcanza con un solo [`cv2.inRange`](https://docs.opencv.org/4.x/d2/de8/group__core__array.html#ga48af0ab51e36436c5d04340e036ce981).
Esto no pasa con el azul,
que cae cómodo en un solo rango en el medio del círculo — es una de las
razones por las que "rojo" es un buen primer color para practicar
segmentación por color.

#### Cómo se traduce esto a código ROS 2

Igual que en la [semana 03](../semana-03-evasion-obstaculos/), conviene
separar quién guarda datos de quién decide:

- **El callback de la cámara (`recibir_imagen`) solo convierte y guarda** la
  última imagen (de `sensor_msgs/Image` a un array de OpenCV con
  `cv_bridge`). No procesa nada ni decide nada ahí.
- **Un timer (`procesar_imagen`), corriendo a `FRECUENCIA_HZ`,** es el que
  agarra la última imagen guardada y decide si hay un cuadrado rojo. Así la
  decisión se toma a una frecuencia fija y conocida, sin importar a qué
  frecuencia llegan los frames de la cámara.
- **Solo se publica cuando hay un cuadrado rojo** — el tópico es un aviso,
  no un estado continuo. Si no se detecta nada, el timer simplemente no
  publica (nunca manda `False`).
- **Loguear en las transiciones** (cuando pasa de detectar a no detectar, y
  viceversa) — eso sí conviene hacerlo siempre, aunque el `False` no se
  publique, para poder ver en los logs cuándo el robot dejó de ver el
  cuadrado.

### Qué hay que completar

El archivo ya tiene resuelto todo lo que no es la detección de color en sí:
los parámetros ROS, el publisher/subscriber, la conversión de `Image` a
array de OpenCV con [`cv_bridge`](https://github.com/ros-perception/vision_opencv)
(`recibir_imagen`), y una función de apoyo ya resuelta
(`area_mayor_contorno` — busca contornos con
[`cv2.findContours`](https://docs.opencv.org/4.x/d4/d73/tutorial_py_contours_begin.html)
y devuelve el área del más grande; no es el objetivo de este workshop). Quedan 3
funciones con `TODO` para completar, cada una con una guía en su docstring:

1. **`mascara_rojo()`** — la percepción: dada una imagen en HSV, devolver
   una máscara binaria de qué píxeles son rojos (combinando los dos rangos
   de H).
2. **`hay_cuadrado_rojo()`** — usa `mascara_rojo()` y `area_mayor_contorno()`
   para decidir si lo que ve la cámara ahora mismo cuenta como un cuadrado
   rojo (área suficientemente grande).
3. **`procesar_imagen()`** — el corazón del workshop: el timer callback que
   arma el `Bool`, lo publica, y loguea solo cuando el valor cambia.

Recomendamos completarlas en ese orden: `mascara_rojo()` es la pieza chica
y fácil de probar por separado (por ejemplo mirando la máscara con
`cv2.imshow` o contando píxeles a mano) antes de escribir la lógica que la
usa.

### Archivos de configuración

También hay `TODO` en los archivos de configuración del paquete (ver
[semana 03](../semana-03-evasion-obstaculos/README.md#el-paquete-ros-2-setuppy-y-packagexml)
para más detalle sobre qué hace cada uno):

- **[`setup.py`](deteccion_color/setup.py) — `entry_points`** — registrar
  el ejecutable `detector` para que `ros2 run deteccion_color detector`
  funcione.
- **[`package.xml`](deteccion_color/package.xml) — `<depend>`** —
  declarar los paquetes de los que depende `detector.py` (mirando sus
  imports).

Estos dos archivos son compartidos con la Parte 2 — cuando lleguen a esa
parte van a tener que volver a tocarlos para sumar el ejecutable y la
dependencia de `detector_scan.py`.

### Parámetros

Todos son configurables vía `--ros-args -p <nombre>:=<valor>`:

| Parámetro | Default | Qué es |
| --- | --- | --- |
| `hue_rojo_bajo_1` | 0.0 | Límite inferior del primer rango de tono (H, 0-179) para rojo — el rojo "bajo", pegado a 0°. |
| `hue_rojo_alto_1` | 10.0 | Límite superior del primer rango. |
| `hue_rojo_bajo_2` | 170.0 | Límite inferior del segundo rango de tono para rojo — el rojo "alto", pegado a 180°. |
| `hue_rojo_alto_2` | 180.0 | Límite superior del segundo rango. |
| `saturacion_min` | 120.0 | Saturación mínima (0-255). Sin este piso, un gris o un rosa muy pálido también caerían en el rango de H del rojo. |
| `valor_min` | 80.0 | Brillo mínimo (0-255), para descartar zonas muy oscuras. |
| `area_minima_px` | 800.0 | Área mínima (en píxeles) del contorno rojo más grande para contarlo como un cuadrado y no como ruido. |

**Nota:** si el rojo real (impreso, o de un objeto físico) queda muy pálido
o muy oscuro bajo la luz del simulador, probablemente haya que ajustar
`saturacion_min` / `valor_min` — es normal tener que calibrar estos rangos
a ojo, mirando la imagen real, en vez de confiar ciegamente en los defaults.

### Cómo correrlo

Con `yahboom_rosmaster` clonado y buildeado en `~/rosmaster_ws` (ver su
[README](https://github.com/AIRclub-UdeSA/yahboom_rosmaster)):

```bash
# Terminal 1 — build
cd ~/rosmaster_ws
colcon build --packages-select deteccion_color
source install/setup.bash
```

Hay varios mundos con cuadrados de colores para detectar (sufijo
`_victimas`). Para ver cuáles hay instalados:

```bash
cd ~/rosmaster_ws
source install/setup.bash
ls "$(ros2 pkg prefix yahboom_rosmaster_gazebo)/share/yahboom_rosmaster_gazebo/worlds/"
```

Podés elegir cualquiera de los `_victimas`; acá usamos
`laberinto_simple_victimas.world` como ejemplo:

```bash
# Terminal 2 — simulador
source install/setup.bash
ros2 launch yahboom_rosmaster_gazebo rosmaster_gazebo_fortress.launch.py \
  world:="$(ros2 pkg prefix yahboom_rosmaster_gazebo)/share/yahboom_rosmaster_gazebo/worlds/laberinto_simple_victimas.world" \
  motion_profile:=ideal
```

```bash
# Terminal 3 — nuestro nodo
source install/setup.bash
ros2 run deteccion_color detector
```

El robot arranca en `laberinto_simple_victimas.world` sin ningún cuadrado a la
vista, así que hace falta manejarlo hasta ponerlo frente a uno para ver algo
en `/rojo_detectado`. Usá una cuarta terminal con [teleoperación por
teclado](https://index.ros.org/p/teleop_twist_keyboard/) (instalada en el
[paso 6 de la guía de
setup](https://airclub-udesa.github.io/jar_site/setup/simulador/#6-teleoperación)
— si no la tenés, `sudo apt install ros-humble-teleop-twist-keyboard`):

```bash
# Terminal 4 — teleop (para acercar el robot a un cuadrado)
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Al ser mecanum, además de `i`/`,` (adelante/atrás) y `j`/`l` (girar) podés
usar las teclas en mayúscula (`I`/`J`/`L`/`U`/`O`/`M`/`<`/`>`, con Shift)
para strafear de costado sin girar.

Chequeos útiles en una quinta terminal:

```bash
ros2 topic hz /cam_1/color/image_raw   # confirmar que la cámara publica
ros2 topic echo /rojo_detectado        # solo aparece mientras ve un cuadrado rojo
```

---

## Parte 2 — `detector_scan.py`

### Teoría: de "veo rojo" a "el rojo está ahí"

#### Por qué no alcanza con la cámara sola

`detector.py` contesta una pregunta binaria: ¿hay rojo en la imagen ahora
mismo? Eso sirve para avisar, pero no dice **dónde** está el cuadrado
respecto del robot — ni a qué distancia ni en qué dirección exacta. Para
navegar hacia el cuadrado, o para marcar su posición en un mapa, hace falta
esa segunda parte.

El lidar sí mide distancia y ángulo con precisión (eso es literalmente lo
que es un `LaserScan`), pero no tiene idea de colores. La idea de esta
parte es combinar los dos sensores: para cada punto que devuelve el lidar,
nos preguntamos "si la cámara estuviera mirando exactamente donde apunta
este rayo, ¿qué pixel de la imagen le tocaría?", y miramos si ese pixel es
rojo.

#### Lo que hace falta para proyectar un punto

Para pasar un punto 3D del mundo a un pixel de imagen hacen falta dos
cosas:

- **Dónde está la cámara respecto del lidar** — la transformada rígida
  (rotación + traslación) entre `laser_link` y el frame óptico de la
  cámara. No hay que medirla a mano: como las dos están descriptas en el
  URDF del robot con joints fijos, `robot_state_publisher` ya la publica en
  [`tf2`](https://docs.ros.org/en/lyrical/Tutorials/Intermediate/Tf2/Tf2-Main.html), y
  basta con pedirla (`tf_buffer.lookup_transform(...)`).
- **Cómo proyecta la cámara** — el modelo *pinhole*: un punto en el frame
  óptico de la cámara (x derecha, y abajo, z adelante) se proyecta al pixel
  `u = fx * x/z + cx`, `v = fy * y/z + cy`. `fx`, `fy`, `cx`, `cy` son los
  intrínsecos de la cámara, publicados en `sensor_msgs/CameraInfo`.

Con eso, cada punto del `/scan` (que viene en coordenadas polares, rango +
ángulo, en el frame del lidar) se puede pasar a cartesiano, rotar/trasladar
al frame de la cámara, y proyectar a un pixel. Si ese pixel cae dentro de
la máscara roja de la última imagen, ese punto del scan es "rojo".

#### Un límite importante: la altura de las marcas

El lidar escanea en un plano horizontal a una altura fija. Si el cuadrado
rojo es una marca chata pegada al piso, la cámara lo va a ver perfecto pero
el lidar nunca la va a tocar — no hay ningún punto de scan que proyecte
sobre ella, sea cual sea la calibración.

### Qué hay que completar

Antes de arrancar, copiá tu `mascara_rojo()` ya resuelta de `detector.py` a
`detector_scan.py` — es la misma función, reciclada tal cual (acá es una
función suelta, no un método, porque este nodo no depende de `detector.py`
para poder correr solo). El resto de la "plomería" ya está resuelta: los
parámetros, las suscripciones a `/cam_1/color/image_raw`,
`/cam_1/color/camera_info` y `/scan`, los callbacks que solo guardan datos
(`recibir_imagen`, `recibir_info_camara`), la matriz de rotación a partir
de un quaternion (`cuaternion_a_rotacion` — álgebra de siempre, no es el
objetivo) y la búsqueda de la transformada en tf2
(`obtener_transform_laser_a_camara`).

Quedan 3 `TODO`, y por acá conviene arrancar:

1. **El publisher de `/scan_rojo`** — a diferencia de la Parte 1, acá no
   está creado: hay que declararlo a mano en `__init__` (mismo tipo de
   línea que ya escribieron para el `entry_point`, pero con
   `create_publisher` en vez de `create_subscription`). Sin esto, nada de
   lo que sigue tiene dónde publicar.
2. **`puntos_laser_a_pixeles()`** — la geometría: dado un array de
   rangos/ángulos del lidar más la transformada y los intrínsecos,
   devolver a qué pixel (u, v) de la imagen correspondería cada punto (y
   si quedó adelante o detrás de la cámara). Es sacar cada punto de polar
   a cartesiano, aplicarle la rotación y traslación de la tf, y proyectarlo
   con el modelo pinhole — todo vectorizado con numpy, sin loops.
3. **`recibir_scan()`** — el corazón de esta parte: el callback del
   `/scan` que junta todo — pide la transformada, arma los puntos del
   scan, los proyecta con `puntos_laser_a_pixeles()`, arma la máscara
   roja de la última imagen, decide qué puntos son rojos, y publica el
   `LaserScan` filtrado (con `math.inf` en los puntos que no son rojos).

### Archivos de configuración

Los mismos dos archivos de la Parte 1, con `TODO` nuevos para
`detector_scan.py`:

- **[`setup.py`](deteccion_color/setup.py) — `entry_points`** — sumar el
  ejecutable `detector_scan` para que `ros2 run deteccion_color
  detector_scan` funcione (además del `detector` de la Parte 1).
- **[`package.xml`](deteccion_color/package.xml) — `<depend>`** — sumar la
  dependencia nueva que usa `detector_scan.py` y que `detector.py` no
  necesitaba: `tf2_ros`.

### Parámetros

Los mismos seis parámetros de HSV de la Parte 1 (`hue_rojo_bajo_1`,
`hue_rojo_alto_1`, `hue_rojo_bajo_2`, `hue_rojo_alto_2`, `saturacion_min`,
`valor_min` — ver la tabla en la Parte 1), con el mismo default y el mismo
significado. `detector_scan.py` **no** tiene `area_minima_px`: esa parte no
trabaja con el área de un contorno, sino con si cada punto del lidar cae o
no sobre un pixel rojo.

### Cómo correrlo

Con la Parte 1 ya buildeada (mismo paquete):

```bash
# Terminal 1 — build (si no lo hiciste ya en la Parte 1)
cd ~/rosmaster_ws
colcon build --packages-select deteccion_color
source install/setup.bash
```

```bash
# Terminal 2 — simulador (igual que en la Parte 1)
source install/setup.bash
ros2 launch yahboom_rosmaster_gazebo rosmaster_gazebo_fortress.launch.py \
  world:="$(ros2 pkg prefix yahboom_rosmaster_gazebo)/share/yahboom_rosmaster_gazebo/worlds/laberinto_simple_victimas.world" \
  motion_profile:=ideal
```

```bash
# Terminal 3 — nuestro nodo (no hace falta tener detector corriendo a la
# vez, son procesos independientes)
source install/setup.bash
ros2 run deteccion_color detector_scan
```

Igual que en la Parte 1, el robot arranca sin ningún cuadrado a la vista, así
que hace falta manejarlo hasta ponerlo frente a uno para ver algo en
`/scan_rojo`. Usá una cuarta terminal con [teleoperación por
teclado](https://index.ros.org/p/teleop_twist_keyboard/) (instalada en el
[paso 6 de la guía de
setup](https://airclub-udesa.github.io/jar_site/setup/simulador/#6-teleoperación)
— si no la tenés, `sudo apt install ros-humble-teleop-twist-keyboard`):

```bash
# Terminal 4 — teleop (para acercar el robot a un cuadrado)
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Al ser mecanum, además de `i`/`,` (adelante/atrás) y `j`/`l` (girar) podés
usar las teclas en mayúscula (`I`/`J`/`L`/`U`/`O`/`M`/`<`/`>`, con Shift)
para strafear de costado sin girar.

Chequeos útiles en una quinta terminal:

```bash
ros2 topic hz /scan                    # confirmar que el lidar publica
ros2 topic echo /scan_rojo             # rangos finitos solo en las direcciones "rojas"
```

En [RViz](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.html), agregar un segundo display `LaserScan` apuntando a `/scan_rojo`
(con otro color) es una buena forma de ver, superpuesto al scan completo,
exactamente qué rayos está clasificando como rojos.
