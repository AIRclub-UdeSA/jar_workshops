# Próximos workshops — roadmap

Esto no es un workshop en sí, es el plan de lo que falta armar y por qué,
para no perder de vista cómo se van a enganchar unos con otros. A medida
que se escribe cada uno, se muda a su propia carpeta `semana-NN-slug/` con
su propio README (ver la [plantilla del sitio](https://github.com/AIRclub-UdeSA/jar_site/blob/main/src/templates/plantilla-semana.md))
y se borra de acá.

## Por qué este orden

El orden en que están listados abajo no es arbitrario, pero tiene una
excepción a tener en cuenta: **"Reconocer obstáculos que no están en el
mapa"** se lista antes que la localización, aunque en realidad depende de
ella — comparar el `/scan` contra el mapa por raycasting necesita saber la
pose del robot en `map`, que es justamente lo que da
[semana 06](semana-06-localizacion/). Los dejamos en el orden en que
surgieron en la conversación para no perder el razonamiento de cada uno,
pero en currícula real semana 06 ya resuelve esa dependencia antes de
llegar a este workshop.

Fuera de eso, la progresión general es: la sección transversal de
launch/RViz iba antes que todo lo que sigue, y ya está escrita — quedó
como su propio mini-workshop en
[semana 05](semana-05-launch-rviz/). Los workshops de acá abajo pueden
apoyarse en ella: en vez de reexplicar launch o RViz, alcanza con decir
"agregá tu nodo al launch" y "agregá este display a tu RViz". El resto
son continuaciones directas de conceptos ya instalados en las semanas 03
y 04.

---

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
una máscara de color. **Depende de tener pose en `map`** — ver
[semana 06](semana-06-localizacion/).

## Workshop — "De 'lo veo ahí' a una coordenada en el mapa"

**Por qué hace falta:** es el eslabón que conecta semana 04 con
[semana 06](semana-06-localizacion/). Hoy semana 04 da distancia y ángulo
al cuadrado rojo relativos al robot, pero eso no sirve para reportar una
posición fija si el robot se mueve.

**Qué cubre:** tomar la pose del robot en `map` (la de semana 06)
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

## Workshop — "Nav2: reemplazar lo manual por el stack real"

**Por qué hace falta:** las últimas cuatro semanas (evasión de
obstáculos, localización, obstáculos que no están en el mapa,
exploración) se resolvieron a mano para entender qué hace cada pieza por
dentro. Pero eso mismo — localización, costmaps y planificación — es
justo lo que resuelve [Nav2](https://docs.nav2.org/) de fábrica, de forma
mucho más robusta que las versiones caseras. Este workshop es el punto de
"graduación": de acá en adelante, Nav2 reemplaza a los módulos manuales
en vez de convivir con ellos.

**Qué cubre:**

- **AMCL** reemplaza el filtro de partículas manual de
  [semana 06](semana-06-localizacion/).
- El **costmap global + local** (mapa estático + ventana rodante de datos
  vivos del lidar) reemplaza tanto la evasión de obstáculos de semana 03
  como el workshop de "obstáculos que no están en el mapa" — la
  superposición de las dos capas es exactamente ese problema, ya resuelto.
- Un **controller** (DWB o regulated pure pursuit) reemplaza la máquina de
  estados de evasión reactiva.
- **`NavigateToPose`** (o `explore_lite` si se quiere explorar sin un
  objetivo fijo) reemplaza la exploración manual por waypoints.

**Qué NO reemplaza:** el workshop de "De 'lo veo ahí' a una coordenada en
el mapa" (detección de víctimas + transformada a `map`) sigue igual —
es lógica propia de la tarea de búsqueda y rescate, Nav2 no tiene opinión
sobre eso. De acá en más, ese nodo corre sobre el stack de Nav2 en vez de
sobre la pila manual.

**Cómo se relaciona con lo que ya existe:** depende de haber hecho las
cuatro semanas manuales anteriores — la comparación directa ("esto que
armé en cuatro semanas, Nav2 lo arma con configuración") es el punto
pedagógico central, así que este workshop va después de todas ellas, no
antes.
