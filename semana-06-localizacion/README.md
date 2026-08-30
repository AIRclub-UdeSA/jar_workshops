# Semana 06 — Dónde estoy: localización en el mapa

## Objetivo

Todo lo hecho hasta ahora (esquivar, detectar rojo, proyectar el lidar
sobre la cámara) vive en el frame del robot o del lidar. Para reportar una
posición en la competencia hace falta algo más: saber dónde está el robot
**dentro del mapa**. Este workshop le da al robot una pose confiable en el
frame `map`, corrigiendo el *drift* de la odometría contra el mapa conocido
usando el lidar — con un **filtro de partículas (Monte Carlo
Localization)**.

A diferencia de las semanas 01-05, esta vez **no recibís un paquete ROS ya
armado**: creás el tuyo propio desde cero por primera vez, igual que ya
venís armando tu propio `launch` y tu propia config de RViz desde la
semana 05. Nosotros te damos el código de los dos nodos (con TODOs) y las
instrucciones para levantar el mapa; el paquete, el launch y el RViz son
tuyos.

Necesitás las semanas [03](../semana-03-evasion-obstaculos/),
[04](../semana-04-deteccion-color/) y
[05](../semana-05-launch-rviz/) completas: acá se reusa la separación
sensor/decisión y el patrón de transformadas con `tf2` de la 04, y el
launch/RViz propio armado en la 05.

---

## Teoría: `map`, `odom`, `base_link`, y por qué la odometría deriva sola

Un robot real tiene (al menos) tres frames relevantes, encadenados:

- **`base_link`** (acá, `base_footprint`): el propio robot. Todo lo demás
  se ubica relativo a él.
- **`odom`**: un frame que arranca coincidiendo con `base_link` y se mueve
  integrando lo que miden los encoders de las ruedas. Es continuo (nunca da
  saltos), pero **deriva**: pequeños errores de deslizamiento, redondeo, o
  terreno irregular se acumulan sin límite. Cuanto más tiempo pasa, más se
  aleja de la posición real.
- **`map`**: el frame del mapa conocido, fijo, que no se mueve. Es donde
  vive la posición "de verdad".

`base_footprint → odom` ya te lo da gratis `wheel_state_odometry` (parte
del simulador, corriendo desde que lanzás Gazebo). Lo que falta, y es el
objetivo de esta semana, es `map → odom`: una transformada que corrige el
error acumulado de la odometría comparando lo que ve el lidar (`/scan`)
contra el mapa conocido. Con las dos transformadas encadenadas
(`map → odom → base_footprint`), cualquier nodo (incluido RViz) puede saber
dónde está el robot en `map` sin tener que saber nada de cómo se calculó la
corrección.

### Filtro de partículas, en un párrafo

En vez de llevar una única hipótesis de "dónde estoy", un filtro de
partículas lleva **muchas** (cientos), cada una una hipótesis distinta de
pose `(x, y, theta)`. En cada ciclo:

1. **Predicción**: cada partícula se mueve según lo que dice la odometría,
   más ruido (porque la odometría no es perfecta).
2. **Corrección**: cada partícula "imagina" qué vería el lidar si esa fuera
   la pose real, lo compara contra el `/scan` real, y las que calzan mejor
   quedan con más peso.
3. **Resampleo**: se redibuja el conjunto de partículas dándole más
   chances de sobrevivir a las que pesan más — así, con el tiempo, la nube
   se concentra alrededor de la pose real.

La pose estimada es, simplemente, el promedio (ponderado) de todas las
partículas.

---

## Antes de empezar: creá tu paquete

Con `yahboom_rosmaster` y `jar_workshops` clonados en `~/rosmaster_ws/src`
(ver la [guía de setup](https://airclub-udesa.github.io/jar_site/setup/simulador/)):

`ros2 pkg create` es el comando que arma, de cero, la carpeta de un
paquete ROS 2 — hasta ahora esa carpeta ya venía clonada del repo
(semanas 01-05); acá la generás vos. Los flags que le pasamos:

- **`--build-type ament_python`**: le dice que es un paquete de Python
  puro (la misma clase de paquete que usaron todas las semanas
  anteriores) — la alternativa, `ament_cmake`, es para paquetes en C++.
- **`--dependencies ...`**: cada nombre de la lista es un paquete ROS que
  tu código va a importar; se completan solos como `<depend>` en
  `package.xml` para que no los tengas que escribir a mano (igual, en el
  paso 3 de más abajo vas a tener que sumar uno más).
- **El último argumento (`localizacion`)** es el **nombre del paquete**:
  así se va a llamar la carpeta que se crea, y así lo vas a invocar
  después (`ros2 run localizacion <ejecutable>`). No es un valor mágico —
  lo elegimos nosotros porque describe el tema de la semana. La
  convención en ROS 2 es `snake_case` (minúsculas, guiones bajos, sin
  espacios ni mayúsculas) y un nombre corto que diga qué hace el paquete,
  mismo criterio que ya viste en `deteccion_color` o `evasion_obstaculos`.

```bash
# Terminal 1
cd ~/rosmaster_ws/src/jar_workshops/semana-06-localizacion
ros2 pkg create --build-type ament_python --dependencies \
  rclpy nav_msgs sensor_msgs geometry_msgs tf2_ros \
  localizacion
```

Esto te genera la misma estructura que ya viste armada de antemano en
semana 04 (`package.xml`, `setup.py`, `setup.cfg`, `resource/`,
`localizacion/localizacion/__init__.py`) — si tenés dudas de qué debería
tener cada archivo, abrí el `package.xml`/`setup.py` de
[semana 04](../semana-04-deteccion-color/deteccion_color/) en otra pestaña
como referencia de formato.

Con el paquete creado:

1. Copiá [`campo_verosimilitud.py`](campo_verosimilitud.py) y
   [`localizador.py`](localizador.py) (los dos archivos de esta carpeta) a
   `localizacion/localizacion/`.
2. En `setup.py`, agregá los dos ejecutables a `entry_points`:
   `'campo_verosimilitud = localizacion.campo_verosimilitud:main'` y
   `'localizador = localizacion.localizador:main'`.
3. En `package.xml`, agregá un `<depend>` por cada import que no sea
   `rclpy`/`nav_msgs`/`sensor_msgs`/`geometry_msgs`/`tf2_ros` (que ya
   declaraste en el `ros2 pkg create`). Hay uno: `campo_verosimilitud.py`
   usa una librería de cálculo científico que no es de ROS — mismo caso que
   `cv2`/`numpy` en semana 04, buscá el nombre del paquete apt
   (`python3-<algo>`) en vez de que te lo demos resuelto.

```bash
cd ~/rosmaster_ws
colcon build --packages-select localizacion --symlink-install
source install/setup.bash
```

El `--symlink-install` es importante acá: sin él, cada vez que edites uno
de los TODO de `campo_verosimilitud.py`/`localizador.py` (que vas a hacer
muchas veces, no una sola) tendrías que correr `colcon build` de nuevo
para que el cambio se vea al ejecutar `ros2 run`. Con `--symlink-install`,
el paquete instalado queda como un symlink a tu código fuente: editás,
guardás, y ya está — no hace falta rebuildear entre cada prueba. (Si más
adelante cambiás `setup.py` para sumar un `entry_point` nuevo, a eso sí
hay que volver a buildearlo.)

---

## Parte 1 — `campo_verosimilitud.py`

### Teoría: el campo de verosimilitud

Comparar el lidar contra el mapa "en vivo" (simular qué vería cada
partícula, rayo por rayo, contra la grilla) es correcto pero carísimo en
Python puro con cientos de partículas corriendo en tiempo real. La
solución estándar es precalcular **una
sola vez**, apenas llega el mapa, un "campo de verosimilitud": para cada
celda del mapa, qué tan probable es que un rayo del lidar termine ahí. Las
celdas ocupadas (y sus alrededores inmediatos) tienen probabilidad alta;
lejos de cualquier obstáculo, la probabilidad cae. Después, comparar un
punto del scan contra el mapa es simplemente **leer un valor de un array**
— nada de buscar ni recalcular en cada ciclo.

### Qué hay que completar

`campo_verosimilitud.py` ya trae resuelta toda la plomería: la
suscripción a `/map` y el publisher de `/likelihood_map` (mismo QoS
*transient local* que un mapa — para que quien se suscriba después siga
recibiendo el último publicado). Queda **una función con `TODO`**:

- **`campo_de_probabilidad()`** — a partir de la grilla de ocupación,
  calcular la distancia de cada celda al obstáculo más cercano
  ([`scipy.ndimage.distance_transform_edt`](https://docs.scipy.org/doc/scipy/reference/generated/scipy.ndimage.distance_transform_edt.html))
  y convertirla a probabilidad con una gaussiana.

### Parámetro

| Parámetro | Default | Qué es |
| --- | --- | --- |
| `sigma_sensor` | 0.2 | Ancho (en metros) del "halo" de probabilidad alrededor de cada obstáculo. Chico = exigente, grande = permisivo. |

### Probarlo solo (antes de tocar el filtro)

No hace falta el simulador corriendo para esta parte — alcanza con
`map_server` sirviendo el mapa real. `yahboom_rosmaster` trae varios
mapas de ocupación ya generados en `maps/`; para ver cuáles hay
instalados (sin necesidad de tener el repo clonado en una ruta fija —
esto corre igual desde cualquier lado):

```bash
ls "$(ros2 pkg prefix yahboom_rosmaster_gazebo)/share/yahboom_rosmaster_gazebo/maps/"
```

Acá usamos `laberinto_simple.yaml` como ejemplo porque, de los mundos que
existen hoy, es el único con un mapa ya generado y verificado que
corresponde a él. A medida que el repo sume mapas nuevos (ver la issue de
mapeo del roadmap), vas a poder correr este mismo workshop contra
cualquier otro par mundo+mapa — pero el par tiene que corresponderse: el
`.yaml` que le pases a `map_server` tiene que ser el que se generó
mapeando justamente el `.world` que estás corriendo, no cualquiera de la
lista de arriba. Si usás un mapa distinto de `laberinto_simple`, además
**no asumas** que `pose_inicial_x/y/theta` sigue siendo `(0,0,0)` —
confirmalo para ese mapa (o pasá los valores correctos por parámetro) antes
de dar por buena la corrección del filtro.

```bash
# Terminal 1
source ~/rosmaster_ws/install/setup.bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:="$(ros2 pkg prefix yahboom_rosmaster_gazebo)/share/yahboom_rosmaster_gazebo/maps/laberinto_simple.yaml"
```

```bash
# Terminal 2
source ~/rosmaster_ws/install/setup.bash
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -p autostart:=true -p node_names:="['map_server']"
```

```bash
# Terminal 3
source ~/rosmaster_ws/install/setup.bash
ros2 run localizacion campo_verosimilitud
```

En RViz, agregá un display `Map` apuntando a `/likelihood_map` (con un
`Color Scheme` que muestre gradiente, no solo blanco/negro). Si está bien,
vas a ver un "halo" difuminado creciendo alrededor de cada pared del
laberinto, en vez de líneas duras.

---

## Parte 2 — `localizador.py`

### Teoría: predicción, corrección, resampleo — con números

- **Predicción**: el movimiento entre dos lecturas de `/odom` se descompone
  en *rotar hacia el rumbo del desplazamiento* (`rot1`), *avanzar*
  (`trans`), *rotar lo que falte* (`rot2`) — el modelo de movimiento
  odométrico estándar (Thrun, *Probabilistic Robotics*). A cada componente se le suma ruido gaussiano proporcional a
  su propia magnitud, y se aplica a las N partículas a la vez.
- **Corrección**: para cada partícula, transformar los puntos del `/scan`
  a su pose, mirar qué dice `/likelihood_map` en esa posición, y combinar
  esas probabilidades en un peso por partícula.
- **Resampleo**: redibujar las N partículas con reemplazo, proporcional al
  peso (*resampling sistemático*).

Notá que acá los callbacks de `/odom` y `/scan` **sí actúan** (predicen o
corrigen), en vez de solo guardar el último dato como en semanas
anteriores — es la forma natural de un filtro de partículas: predicción
atada a la tasa de odometría, corrección atada a la tasa del sensor, sin
un timer artificial en el medio.

### Qué hay que completar

Toda la plomería está resuelta: parámetros, suscripciones, la
inicialización de las partículas (una nube gaussiana angosta alrededor de
una pose inicial conocida — **no** es localización global, ver el desafío
extra), la conversión del `/scan` a puntos en `base_footprint` (reusando
`tf2`, mismo patrón que `detector_scan.py` de semana 04), la estimación de
pose por promedio, la publicación de la nube de partículas
(`geometry_msgs/PoseArray` en el tópico `particlecloud`) y de dos caminos (`camino_odom` sin corregir, `camino_corregido`
con el filtro), y la publicación de la transformada `map → odom` por
`tf2_ros.TransformBroadcaster`. Quedan **3 funciones con `TODO`**, el
corazón del filtro:

1. **`mover_particulas()`** — el modelo de movimiento (predicción).
2. **`pesar_particulas()`** — el modelo de sensor (corrección) contra
   `/likelihood_map`. Es la más larga — el docstring la guía paso a paso.
3. **`remuestrear()`** — el resampling sistemático.

Completalas en ese orden: `mover_particulas()` la podés probar mirando que
la nube de partículas se "abra" al mover el robot (aunque todavía no
corrija nada), antes de meterte con `pesar_particulas()`.

### Parámetros

| Parámetro | Default | Qué es |
| --- | --- | --- |
| `map_frame` / `odom_frame` / `base_frame` | `map` / `odom` / `base_footprint` | Nombres de los frames. |
| `num_particulas` | 300 | Cuántas hipótesis de pose mantiene el filtro. Más partículas = más preciso, más lento. |
| `pose_inicial_x/y/theta` | 0.0 / 0.0 / 0.0 | Pose inicial conocida (el robot siempre spawnea en el origen del mundo, que coincide con el origen del mapa). |
| `dispersion_inicial_xy` / `dispersion_inicial_theta` | 0.3 / 0.3 | Qué tan dispersa arranca la nube alrededor de la pose inicial. |
| `alpha1`-`alpha4` | 0.05 cada uno | Ruido del modelo de movimiento rot1-trans-rot2 (ver teoría). |
| `submuestreo_scan` | 15 | Cada cuántos rayos del `/scan` (de 1080) se usa para pesar. Bajarlo = más preciso y más lento. |

### Cómo correrlo

Extendé el launch que armaste en semana 05 (o corré cada terminal a mano)
sumando: el simulador con `laberinto_simple.world`, `map_server` +
`nav2_lifecycle_manager` apuntando a `laberinto_simple.yaml` (mismo mapa
que en la Parte 1 — de los varios `.yaml` en
`maps/`, ese es el que corresponde a este mundo; ver `Probarlo solo` más
arriba si querés listarlos de nuevo), y los dos nodos de este paquete.
Ejemplo mínimo, terminal por terminal, para probarlo antes de meterlo en
tu launch:

```bash
# Terminal 1 — simulador
source ~/rosmaster_ws/install/setup.bash
ros2 launch yahboom_rosmaster_gazebo rosmaster_gazebo_fortress.launch.py \
  world:="$(ros2 pkg prefix yahboom_rosmaster_gazebo)/share/yahboom_rosmaster_gazebo/worlds/laberinto_simple.world" \
  motion_profile:=ideal
```

```bash
# Terminal 2 — mapa
source ~/rosmaster_ws/install/setup.bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:="$(ros2 pkg prefix yahboom_rosmaster_gazebo)/share/yahboom_rosmaster_gazebo/maps/laberinto_simple.yaml"
```

```bash
# Terminal 3 — activar el mapa
source ~/rosmaster_ws/install/setup.bash
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -p autostart:=true -p node_names:="['map_server']"
```

```bash
# Terminal 4 — nuestros nodos
source ~/rosmaster_ws/install/setup.bash
ros2 run localizacion campo_verosimilitud &
ros2 run localizacion localizador
```

```bash
# Terminal 5 — teleop
source ~/rosmaster_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

En tu config de RViz (la de semana 05), agregá displays para: `Map`
(`/map` y `/likelihood_map`), `PoseArray` (`particlecloud`), y dos `Path`
(`camino_odom` en un color, `camino_corregido` en otro) — además del
`LaserScan` y `TF` que ya tenías.

---

## Comprobación

Manejá el robot un rato por el laberinto (con giros, no solo derecho) y
mirá en RViz:

- La nube de partículas (`particlecloud`) se **abre** un poco cada vez que
  el robot se mueve, y se **contrae** cada vez que llega un `/scan` nuevo
  y corrige — se tiene que ver "respirar".
- El camino azul (`camino_odom`, odometría sin corregir) se va separando
  del camino rojo (`camino_corregido`) a medida que pasa el tiempo — así
  se ve el *drift* directamente.
- El `/scan` (contra el `/map`) se mantiene alineado con las paredes del
  laberinto, sin importar cuánto tiempo lleve andando.

> [!WARNING]
> Si arrancás el filtro *antes* de que `map_server` esté activo (lifecycle
> `active`), `/likelihood_map` nunca llega y las partículas no corrigen
> nunca — quedate solo con el ruido de la predicción. Confirmá primero con
> `ros2 topic echo /likelihood_map --once` que el campo ya está publicado.

---

## Explicación

Con `map → odom` publicada, cualquier nodo (RViz, un futuro planificador)
puede preguntar "¿dónde está el robot en el mapa?" sin saber nada de cómo
se corrigió. La diferencia
es que ahora sabés qué hay adentro: un conjunto de hipótesis, un modelo de
cómo se mueven, y un modelo de qué tan bien explican lo que ve el lidar.
Ese mismo patrón (varias hipótesis, pesarlas contra una observación,
resamplear) reaparece en muchos otros problemas de robótica más allá de
localización.

## Desafío extra

- **Localización global**: hoy el filtro arranca con una nube angosta
  alrededor de una pose conocida — es *tracking*, no relocalización. El
  mismo algoritmo sirve para localización global con un solo cambio:
  inicializar las partículas con distribución uniforme sobre las celdas
  libres de todo el mapa (en vez de una gaussiana angosta) y subir bastante
  `num_particulas`. Probalo y fijate cuánto tarda en converger, y cuántas
  partículas hacen falta para que no se pierda en un pasillo simétrico.
- **Remuestreo condicional (`Neff`)**: hoy se remuestrea en cada `/scan`
  recibido. El tamaño de muestra efectivo,
  `Neff = 1 / Σ(peso_i²)` (con pesos normalizados), mide qué tan
  concentrado está el peso entre las partículas — si `Neff` ya es alto (el
  peso está repartido parejo), remuestrear no suma nada y solo tira
  diversidad por la borda. Modificá `recibir_scan()` para remuestrear solo
  cuando `Neff` cae por debajo de, por ejemplo, `num_particulas / 2`.
