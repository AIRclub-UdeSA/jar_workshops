# jar_workshops

Código de los workshops asincrónicos del [Challenge JAR](https://github.com/AIRclub-UdeSA/jar_site) (AIR Club UdeSA, JAR 2026).

Cada carpeta `semana-NN-slug/` contiene un paquete ROS 2 (`ament_python`) con el mini-proyecto de esa semana. La explicación de cada workshop — objetivo, cómo correrlo contra el simulador, qué observar — vive en el sitio: https://airclub-udesa.github.io/jar_site/workshops/

## Uso

Cloná este repo dentro de `src/` de tu workspace ROS (el mismo donde tenés `yahboom_rosmaster`, ver la [guía de setup](https://airclub-udesa.github.io/jar_site/setup/simulador/)):

```bash
cd ~/rosmaster_ws/src
git clone https://github.com/AIRclub-UdeSA/jar_workshops.git
cd ~/rosmaster_ws
colcon build --packages-select <paquete_de_la_semana>
source install/setup.bash
```

## Workshops

| Semana | Paquete | Tema |
| --- | --- | --- |
| 01 | `talkers_listeners` | Talkers y listeners |
| 02 | `zigzag_mecanum` | Zigzag mecanum |
| 03 | `evasion_obstaculos` | Evasión de obstáculos (máquina de estados) |
| 04 | `deteccion_color` | Detección de color con cámara + lidar |
| 05 | `launch_rviz` | Armá tu launch y tu RViz (sección transversal) |

Ver [`PROXIMOS_WORKSHOPS.md`](PROXIMOS_WORKSHOPS.md) para el roadmap de lo que sigue.

## Contribuir

Ver [CONTRIBUTING.md](CONTRIBUTING.md) para el proceso de pull request y las guías de contribución.
