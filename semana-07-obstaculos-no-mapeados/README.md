# Semana 07 — Reconocer obstáculos que no están en el mapa

## Objetivo

El mapa que vamos a dar en la competencia es una foto fija del terreno. El
terreno real, en cambio, va a tener cosas que esa foto no tiene: escombros,
muebles corridos, hasta las propias víctimas. Este workshop compara el
`/scan` en tiempo real contra el mapa conocido, agrupa los puntos que el
mapa no esperaba en objetos discretos, y publica **la coordenada `(x, y)`
en `map` de cada uno** — no solo "hay algo ahí" relativo al robot, sino una
posición fija que tiene sentido reportar aunque el robot se siga moviendo.

Es puramente geométrico: sin cámara, sin color, solo `/scan` + el campo de
verosimilitud de semana 06 + la pose del robot. Y, de paso, te deja armado
todo lo que hace falta para combinar esto con `/scan_rojo` de semana 04 y
quedarte solo con el objeto que además es rojo — o sea, reportar la
posición de una víctima. No hay un workshop aparte para eso: es esta
semana más una máscara de color que ya tenés, combinadas por vos (ver
Desafío extra).

Necesitás las semanas [04](../semana-04-deteccion-color/) y
[06](../semana-06-localizacion/) completas: acá se reusa el patrón de
transformadas con `tf2` y el truco de "scan filtrado con `inf` en lo que no
importa" de la 04, y la pose en `map` + `/likelihood_map` de la 06.

---

## Teoría: comparar contra un campo que ya está calculado

La forma "directa" de comparar el scan contra el mapa sería
[*raycasting*](https://en.wikipedia.org/wiki/Ray_casting):
para cada rayo del lidar, simular qué debería medir un lidar ideal parado
en la pose actual del robot, marchando sobre la grilla del mapa celda por
celda. Es exactamente el mismo tipo de cuenta que semana 06 descartó por
lenta en Python puro — ahí, en vez de simular el lidar completo por cada
partícula, se precalculó **una sola vez** un campo de verosimilitud
(`/likelihood_map`) y después solo se lo indexa.

Ese campo ya es, literalmente, "qué tan probable es que el mapa diga que
hay algo en esta celda". Así que compararlo acá es gratis: transformás cada
punto del `/scan` a `map` (mismo lookup de `tf2` que ya usaste en semana
04, con la tf completa que arma semana 06), lo convertís a un índice
`[fila, columna]` de `/likelihood_map`, y leés la probabilidad — si es baja,
el mapa no tenía nada ahí.

### De puntos sueltos a un objeto con coordenada

Un obstáculo real casi nunca genera un solo punto "no mapeado": genera una
**racha** de varios rayos seguidos pegando en la misma cara del objeto,
igual que ya viste con `/scan_rojo`. Agrupar esas rachas y promediar sus
puntos en `map` alcanza para tener la coordenada del objeto — es
deliberadamente más simple que un clustering espacial genérico (tipo
[DBSCAN](https://scikit-learn.org/stable/modules/generated/sklearn.cluster.DBSCAN.html)):
al venir de un único scan ordenado angularmente, la continuidad
angular ya casi garantiza continuidad espacial real. La limitación conocida
es que dos objetos separados, vistos desde lejos, pueden quedar pegados en
una misma racha — no hace falta resolverlo con un algoritmo más pesado (ver
Desafío extra si te interesa).

### Por qué hay que descartar lo que cae fuera del mapa

Un punto del `/scan` puede transformarse a una posición `(x, y)` que cae
**afuera** del área que cubre `/likelihood_map` (por ejemplo, el robot
mirando hacia el borde del mapa conocido). Ahí no hay ninguna celda que
consultar — y es tentador tratar "no hay dato" igual que "probabilidad
baja", pero son cosas distintas: el mapa no tiene nada que decir sobre lo
que está fuera de sus límites, así que ahí **no se puede concluir nada**.
Si no separás este caso, cada corrida te va a reportar objetos fantasma
pegados al borde del mapa, sin importar cuánto ajustes el umbral de
probabilidad — ningún valor de umbral arregla esto, porque el problema no
es de umbral, es de qué celda estás mirando.

---

## Antes de empezar: creá tu paquete

Creá tu paquete, llamado por ejemplo `obstaculos_no_mapeados`, y modificá
los archivos correspondientes — mismo procedimiento que en
[semana 06](../semana-06-localizacion/). Copiá
[`detector_obstaculos.py`](detector_obstaculos.py) al paquete. Por último,
hacé `colcon build --symlink-install`, otra vez, porque vas a editar los
TODOs muchas veces.

---

## Qué hay que completar

Toda la plomería está resuelta: parámetros, suscripciones (incluida la de
`/scan` en `qos_profile_sensor_data`, como en las semanas 03, 04 y 06), el
lookup de la transformada a `map`, y la publicación del `LaserScan`
filtrado, el `PoseArray` y el `MarkerArray`. Quedan **2 funciones con
`TODO`**:

1. **`agrupar_en_rachas()`** — encontrar las rachas contiguas de puntos "no
   mapeados" en el array del scan. Es un ejercicio de arrays con numpy,
   sin librerías nuevas — el docstring te guía paso a paso.
2. **`recibir_scan()`** — el corazón del workshop: transformar el scan a
   `map`, indexar contra `/likelihood_map`, y decidir qué puntos son "no
   mapeados" (incluyendo el descarte del borde de la Teoría). Reusá acá lo
   que ya escribiste en semana 04 (polar → cartesiano) y lo que ya viste en
   semana 06 (`pesar_particulas`, para el índice de fila/columna).

Completalas en ese orden: `agrupar_en_rachas()` la podés probar aislada,
sin ROS (es una función pura sobre un array de bools), antes de meterte con
`recibir_scan()`.

## Parámetros

| Parámetro | Default | Qué es |
| --- | --- | --- |
| `map_frame` | `map` | Frame contra el que se reportan las coordenadas. |
| `umbral_probabilidad` | 25.0 | Probabilidad (0-100) por debajo de la cual un punto se considera "no mapeado". **Sin calibrar a fondo** — ver Desafío extra. |
| `racha_minima` | 3 | Cuántos puntos contiguos hacen falta para que una racha cuente como objeto (menos que eso, se descarta como ruido). |
| `distancia_minima_valida` | 0.25 | Rango mínimo (m) que se considera válido. El lidar rebota contra la propia estructura del robot (antena, soporte) y devuelve rangos cortos pero "válidos" para el sensor — este filtro descarta ese rebote. |

## Cómo correrlo

Sumá `detector_obstaculos` a tu propio launch de semana 06 (un `Node` más
en la `LaunchDescription`, mismo patrón que ya usaste para agregar
`localizador` y `campo_verosimilitud`). Dos cosas puntuales que cambian
esta semana:

- Usá un mundo `_victimas` en vez del mundo "limpio" que veniás usando —
  son el mismo laberinto, pero con un par de cubos de color puestos en el
  mundo que **no** están en el mapa que le pasás a `map_server` (que sigue
  siendo el mismo `.yaml` de siempre, sin cambios). Esos cubos son justo
  los objetos "no mapeados" que este workshop tiene que encontrar; si
  corrés el mundo sin víctimas no vas a tener nada que detectar (lo cual
  también sirve para confirmar que no te tira falsos positivos contra las
  paredes reales).

  Para este workshop hace falta el par completo (mundo
  `_victimas` **y** su `.yaml`). Para ver qué pares existen:

  ```bash
  comm -12 \
    <(ls "$(ros2 pkg prefix yahboom_rosmaster_gazebo)/share/yahboom_rosmaster_gazebo/worlds/" | grep victimas | sed 's/_victimas\.world$/.yaml/' | sort) \
    <(ls "$(ros2 pkg prefix yahboom_rosmaster_gazebo)/share/yahboom_rosmaster_gazebo/maps/" | sort)
  ```

  (`comm -12` compara dos listas ordenadas y se queda solo con lo que
  aparece en las dos — acá, los mundos `_victimas` a los que les sacás el
  sufijo y les ponés `.yaml`, contra los mapas que existen de verdad.) De
  ejemplo vamos a usar `laberinto_simple.yaml`.
- Agregale a tu config de RViz los displays nuevos: `LaserScan`
  (`scan_no_mapeado`, en Best Effort — mismo QoS que el `LaserScan` de
  `/scan` que ya tenías), `PoseArray` (`objetos_no_mapeados`) y
  `MarkerArray` (`objetos_no_mapeados_markers`).

> [!WARNING]
> Si `detector_obstaculos` arranca antes de que `map_server` esté activo o
> antes de que `localizador` haya publicado la primera `map → odom`, vas a
> ver el aviso `Todavía no hay tf laser_link->map` — es esperable, no un
> error: en cuanto exista esa tf, el nodo arranca a detectar solo, sin que
> haga falta reiniciarlo.

---

## Comprobación

Manejá el robot cerca de una de las víctimas del mundo (recordá que tienen
que estar en el `.world` pero no en el `.yaml` del mapa) y mirá en RViz:

- Aparece un marcador (esfera roja) quieto sobre la víctima, en su posición
  real dentro del mapa — **sin importar desde dónde la mire el robot**. Dale
  una vuelta alrededor y confirmá que el marcador no se mueve con la cámara
  ni con el robot.
- El `LaserScan` de `scan_no_mapeado` solo tiene puntos cerca de la víctima
  — si ves puntos pegados a las paredes reales del laberinto, revisá el
  filtro del borde de la Teoría (o bajá `umbral_probabilidad`).
- Alejate y volvé a acercarte: el marcador se mantiene en la misma
  coordenada, aunque la distancia y el ángulo relativos al robot hayan
  cambiado por completo — esa es la comprobación central: estás reportando
  una posición del mundo, no una medición relativa.

---

## Explicación

Con esto, cualquier nodo puede preguntar "¿hay algo que el mapa no tenga
registrado, y dónde?" sin necesidad de saber nada de cómo se calculó esa
respuesta — mismo patrón de desacople que ya viste con `map → odom` en
semana 06. Vale la pena notar que esto es, a mano, una versión simplificada
de lo que hace la capa de obstáculos del *local costmap* de
[Nav2](https://docs.nav2.org/): un costmap combina el mapa estático con una
ventana rodante de datos vivos del sensor, marcando como ocupado lo que el
sensor ve pero el mapa no. Cuando llegue el workshop de Nav2, esa capa
reemplaza a este nodo — pero el concepto de fondo es el mismo que acabás de
programar acá.

## Desafío extra

- **Calibrar `umbral_probabilidad` y `racha_minima` en serio**: los
  defaults de este README funcionan en `laberinto_simple_victimas`, pero no
  están calibrados a fondo contra otros mapas ni contra movimiento
  sostenido. Manejá un rato largo, contá a mano falsos positivos (objetos
  que se reportan y no existen) y falsos negativos (víctimas que no se
  detectan), y ajustá los dos parámetros buscando minimizar ambos —
  mismo criterio que `sigma_sensor` en semana 06. Preguntas para guiarte:
  ¿qué pasa con `umbral_probabilidad` muy alto? ¿Y muy bajo? ¿`racha_minima`
  en 1 o 2 te agrega falsos positivos de ruido de sensor?
- **Reportar la posición de una víctima roja**: combiná este nodo con
  `/scan_rojo` de semana 04 — en vez de comparar el `/scan` crudo contra
  `/likelihood_map`, comparar `/scan_rojo` (que ya viene filtrado a "solo
  los rayos que además pegan en algo rojo"). El resultado son las
  coordenadas de los objetos que son, a la vez, no mapeados y rojos — la
  víctima. No hace falta escribir nada nuevo: es cambiar qué tópico
  suscribís.
- **Clustering espacial en vez de por racha**: la agrupación por racha
  contigua del scan puede pegar dos objetos separados en una sola racha si
  se los ve desde lejos y en un ángulo parecido. Probá agrupar en cambio
  por distancia real entre puntos en `map` (por ejemplo, con
  [`scipy.cluster.hierarchy.fclusterdata`](https://docs.scipy.org/doc/scipy/reference/generated/scipy.cluster.hierarchy.fclusterdata.html)
  o una versión casera de DBSCAN) y compará contra qué tan seguido este
  caso realmente aparece en los mapas que tenés.
