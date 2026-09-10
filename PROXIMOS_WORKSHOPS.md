# Próximos workshops — roadmap

Esto no es un workshop en sí, es el plan de lo que falta armar y por qué,
para no perder de vista cómo se van a enganchar unos con otros. A medida
que se escribe cada uno, se muda a su propia carpeta `semana-NN-slug/` con
su propio README (ver la [plantilla del sitio](https://github.com/AIRclub-UdeSA/jar_site/blob/main/src/templates/plantilla-semana.md))
y se borra de acá.

## Por qué este orden

La sección transversal de launch/RViz iba antes que todo lo que sigue, y
ya está escrita — quedó como su propio mini-workshop en
[semana 05](semana-05-launch-rviz/). Los workshops de acá abajo pueden
apoyarse en ella: en vez de reexplicar launch o RViz, alcanza con decir
"agregá tu nodo al launch" y "agregá este display a tu RViz".

[Semana 06](semana-06-localizacion/) (localización),
[semana 07](semana-07-obstaculos-no-mapeados/) (obstáculos que no están en
el mapa) y [semana 08](semana-08-cobertura-mapa/) (cobertura del mapa) ya
están escritas — semana 07 depende de tener pose en `map`, que es justo lo
que resuelve la 06, y de paso absorbe lo que hubiera sido un workshop
aparte de "coordenada de víctima": al ya transformar cada punto a `map`
para comparar contra el mapa, reportar una coordenada es la conclusión
natural del mismo pipeline (combinarlo con `/scan_rojo` de semana 04 para
reportar la víctima en particular queda como ejercicio de cada equipo, no
como workshop nuevo). Semana 08 no depende de la 07 — son ramas hermanas,
las dos parten de la 06 — y es, por su propio README, el último workshop
**manual** del roadmap.

---

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
  como [semana 07](semana-07-obstaculos-no-mapeados/) (obstáculos que no
  están en el mapa) — la superposición de las dos capas es exactamente ese
  problema, ya resuelto.
- Un **controller** (DWB o regulated pure pursuit) reemplaza la máquina de
  estados de evasión reactiva.
- **`NavigateToPose`** (o `explore_lite` si se quiere explorar sin un
  objetivo fijo) reemplaza la exploración manual por waypoints de
  [semana 08](semana-08-cobertura-mapa/).

**Qué NO reemplaza:** la parte de [semana 07](semana-07-obstaculos-no-mapeados/)
que combina la coordenada del objeto no mapeado con `/scan_rojo` de
semana 04 para reportar una víctima sigue igual — es lógica propia de la
tarea de búsqueda y rescate, Nav2 no tiene opinión sobre eso. De acá en
más, ese nodo corre sobre el stack de Nav2 en vez de
sobre la pila manual.

**Cómo se relaciona con lo que ya existe:** depende de haber hecho las
cuatro semanas manuales anteriores — la comparación directa ("esto que
armé en cuatro semanas, Nav2 lo arma con configuración") es el punto
pedagógico central, así que este workshop va después de todas ellas, no
antes.
