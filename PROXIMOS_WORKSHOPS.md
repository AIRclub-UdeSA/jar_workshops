# Próximos workshops — roadmap

Esto no es un workshop en sí, es el plan de lo que falta armar y por qué,
para no perder de vista cómo se van a enganchar unos con otros. A medida
que se escribe cada uno, se muda a su propia carpeta `semana-NN-slug/` con
su propio README (ver la [plantilla del sitio](https://github.com/AIRclub-UdeSA/jar_site/blob/main/src/templates/plantilla-semana.md))
y se borra de acá.

## Por qué este orden

El orden en que están listados abajo no es arbitrario, pero tiene una
excepción a tener en cuenta: **"Reconocer obstáculos que no están en el
mapa"** se lista antes que **"Dónde estoy: localización"**, aunque en
realidad depende de él — comparar el `/scan` contra el mapa por
raycasting necesita saber la pose del robot en `map`, que es justamente lo
que da el workshop de localización. En términos de currícula probablemente
convenga que localización vaya primero. Los dejamos en el orden en que
surgieron en la conversación para no perder el razonamiento de cada uno,
pero la numeración final de las semanas debería resolver esta dependencia.

Fuera de eso, la progresión general es: la **sección transversal de
launch/RViz** va antes que todo lo que sigue — probablemente como
apéndice de la [semana 01](semana-01-talkers-listeners/) (ya escrita);
el resto son continuaciones directas de conceptos ya instalados en las
semanas 03 y 04.

---

## Sección transversal — "Armá tu launch y tu RViz"

No es un workshop con tema propio, sino una sección que se escribe una
sola vez — probablemente como apéndice de la semana 01, o como su propio
mini-workshop corto — y después se referencia desde todos los demás.

**Qué cubre:**

- Qué es un archivo `launch.py`.
- Cómo lanzar varios nodos juntos, con parámetros.
- Cómo armar una configuración de RViz que muestre lo relevante de cada
  workshop (el `LaserScan`, la imagen de la cámara, `/scan_rojo`, el mapa,
  la pose del robot, etc.).

**La gracia:** cada README posterior, en vez de reexplicar RViz o launch
cada vez, simplemente dice "agregá tu nodo al launch" y "agregá este
display a tu RViz". La skill se construye una sola vez acá y se reusa
siempre — evita repetir la misma explicación en cada semana.

## Workshop — "Reconocer obstáculos que no están en el mapa"

**Por qué hace falta:** el mapa que dan en la competencia es una foto fija
del terreno, pero el `/scan` en tiempo real va a chocar contra cosas que
ese mapa no tiene — escombros, muebles corridos, hasta las propias
víctimas. Este nodo es la base para que la máquina de estados de semana 03
esquive cosas reales del terreno y no solo lo que "cree" el mapa estático.

**Qué cubre:** comparar cada punto del `/scan` actual contra lo que
"debería" verse según el mapa y la pose del robot — básicamente,
raycasting sobre el `OccupancyGrid` — y publicar un segundo `LaserScan` (o
`PointCloud`) que contenga solo los puntos "sorpresa": los que no
coinciden con el mapa.

**Cómo se relaciona con lo que ya existe:** es el mismo patrón de
semana 04 (filtrar un scan según un criterio y publicar el resultado),
pero acá el criterio es geométrico contra un mapa en vez de visual contra
una máscara de color. **Depende de tener pose en `map`** — ver la nota de
orden más arriba.

## Workshop — "Dónde estoy: localización en el mapa"

**Por qué hace falta:** es el que hoy falta y es crítico. Todo lo hecho
hasta acá (esquivar, detectar rojo, detectar obstáculos nuevos) vive en el
frame del robot o del lidar, pero para reportar una posición en la
competencia hace falta saber dónde está el robot dentro del `map`.

**Qué cubre:**

- La diferencia entre los frames `map`, `odom` y `base_link`.
- Por qué la odometría sola deriva con el tiempo.
- Cómo usar algo como AMCL (o una versión más simple con
  partículas/ICP, si se quiere evitar la caja negra) para corregir la
  pose contra el mapa conocido usando el lidar.

**Resultado:** el robot tiene una pose confiable en `map` en todo
momento — el ingrediente que le faltaba a los dos workshops de arriba y
abajo.

## Workshop — "De 'lo veo ahí' a una coordenada en el mapa"

**Por qué hace falta:** es el eslabón que conecta semana 04 con el
workshop de localización. Hoy semana 04 da distancia y ángulo al cuadrado
rojo relativos al robot, pero eso no sirve para reportar una posición fija
si el robot se mueve.

**Qué cubre:** tomar la pose del robot en `map` (la del workshop anterior)
más la detección relativa de semana 04, hacer la transformación
geométrica correspondiente, y obtener una coordenada (x, y) en el frame
del mapa.

**Tamaño:** corto — es básicamente una cuenta de transformadas — pero es
el paso que falta para poder decir "encontré una víctima en tal posición
del mapa" en vez de "veo algo rojo a 2 metros a mi izquierda".

## Workshop — "Explorar el mapa en vez de deambular"

**Por qué hace falta:** semana 03 le enseña al robot a no chocar, pero no
a buscar de forma sistemática. En una competencia con tiempo limitado, la
estrategia de búsqueda importa tanto como esquivar bien — "avanzar hasta
chocar y girar" no cubre el área de forma confiable.

**Qué cubre**, de más simple a más avanzado (la elección depende de
cuánto tiempo haya para el temario):

- Una lista de waypoints que cubran el mapa, yendo siempre al más cercano
  no visitado.
- Frontier exploration.
- Usar Nav2 directamente para que planifique.
