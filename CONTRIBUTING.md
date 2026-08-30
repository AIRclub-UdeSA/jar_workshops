# Contribuir a jar_workshops

Gracias por estar acá. `jar_workshops` tiene el código de los workshops
asincrónicos del [Challenge JAR](https://github.com/AIRclub-UdeSA/jar_site)
(AIR Club UdeSA, JAR 2026) — el ejercicio semanal que cada equipo resuelve
contra el simulador antes de tocar el robot físico. Mejora cuando más
gente lo revisa, lo corrige y lo extiende.

## Cómo empezar

Cloná este repo dentro de `src/` de tu workspace ROS (el mismo donde
tenés `yahboom_rosmaster`, ver la
[guía de setup](https://airclub-udesa.github.io/jar_site/setup/simulador/)):

```bash
cd ~/rosmaster_ws/src
git clone https://github.com/AIRclub-UdeSA/jar_workshops.git
cd ~/rosmaster_ws
colcon build --packages-select <paquete_de_la_semana>
source install/setup.bash
```

## Buenas primeras contribuciones

- **Sumar un workshop del roadmap.** Ver
  [`PROXIMOS_WORKSHOPS.md`](PROXIMOS_WORKSHOPS.md) para lo que sigue.
- **Mantener sincronizados README y sitio.** Cada semana tiene su
  explicación acá (en el `README.md` de su carpeta) y en
  [jar_site](https://airclub-udesa.github.io/jar_site/workshops/). Si
  cambia el código o el enunciado de una, conviene revisar que la otra
  diga lo mismo.
- **Completar un README que falte.** No todas las semanas tienen uno
  todavía — si encontrás una carpeta sin `README.md`, es una buena
  oportunidad de sumarlo siguiendo el formato de las demás.
- **Fijate en los TODO de cada semana.** Los ejercicios ya resueltos por
  el equipo (branches viejas, soluciones de referencia) suelen revelar
  ambigüedades en la consigna o en los docstrings de guía — si algo te
  costó entender resolviéndolo, probablemente valga la pena aclararlo acá.

## Estructura del repo

Cada carpeta `semana-NN-slug/` contiene un paquete ROS 2 (`ament_python`)
con el mini-proyecto de esa semana:

| Semana | Paquete | Tema |
| --- | --- | --- |
| 01 | `talkers_listeners` | Talkers y listeners |
| 02 | `zigzag_mecanum` | Zigzag mecanum |
| 03 | `evasion_obstaculos` | Evasión de obstáculos (máquina de estados) |
| 04 | `deteccion_color` | Detección de color con cámara + lidar |
| 05 | `launch_rviz` | Armá tu launch y tu RViz (sección transversal) |

## Estilo de código

- Cada paquete es `ament_python`: declarar dependencias con `<depend>` en
  `package.xml` y registrar ejecutables en `entry_points` de `setup.py` —
  sin eso, `colcon build` puede fallar o el ejecutable no existir aunque
  el código esté perfecto.
- El patrón de ejercicio de estas semanas es TODOs guiados por docstring:
  el archivo trae resuelta toda la "plomería" (parámetros, publishers,
  subscribers, funciones de apoyo) y deja 3-4 funciones con `TODO` y una
  guía de qué tienen que hacer. Mantené ese patrón si agregás un ejercicio
  nuevo.
- Separación sensor/decisión: el callback de un sensor solo guarda el
  último dato en una variable, nunca decide ni actúa — la lógica vive en
  un timer callback aparte, a frecuencia fija y conocida. Es la
  convención que se repite desde la semana 03 en adelante.
- Preferí commits chicos y revisables.

## Pull requests

La rama `main` está protegida en GitHub (branch protection rule, classic).
Todo cambio a `main` tiene que pasar por un pull request:

1. Creá una branch para tu cambio (`git checkout -b feature/mi-cambio`) y
   subila.
2. Abrí un PR contra `main`.
3. Se necesita al menos 1 aprobación de otra persona (distinta del autor)
   antes de poder mergear.
4. Resolvé todos los comentarios de revisión antes de mergear.
5. Si se suben commits nuevos después de una aprobación, esa aprobación se
   invalida y hay que volver a pedir revisión.
6. No se puede pushear directo a `main` por default; el rol de admin del
   repo puede saltear esta regla, pero conviene reservarlo para
   emergencias, no para el flujo normal.

## Reglas generales

- Estos workshops los resuelve en simultáneo cada equipo de JAR 2026 —
  un cambio en un enunciado o en un default afecta a todos a la vez.
  Avisá en la descripción del PR si tu cambio altera el comportamiento
  esperado de una semana ya publicada.
- Sé buena onda. Asumí buena fe, mantené todo constructivo.
