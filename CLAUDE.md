# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

Teaching material for the **Challenge JAR** (AIR Club UdeSA, 2026): a series of asynchronous ROS 2 workshops, each in its own `semana-NN-slug/` folder. Each folder is a self-contained `ament_python` ROS 2 package with one or more nodes that students complete by filling in `TODO`s. The prose explanation of each workshop (objective, theory, how to run it against the simulator) lives both in each folder's `README.md` and on the companion site https://airclub-udesa.github.io/jar_site/workshops/ — the two should stay consistent when editing a workshop.

This repo is meant to be cloned into `src/` of a ROS 2 workspace that already has `yahboom_rosmaster` (the simulated robot description/gazebo packages) built — see `README.md`. There is no standalone build/test tooling here; everything is driven by `colcon` from the outer workspace.

## Repo layout

```
semana-NN-slug/<paquete_ros>/          # el paquete ament_python
  package.xml                          # dependencias ROS (<depend>)
  setup.py                             # entry_points (qué ejecutables expone)
  setup.cfg
  resource/<paquete_ros>
  <paquete_ros>/                       # el código Python real
    __init__.py
    <nodo>.py                          # uno o más nodos por paquete
semana-NN-slug/README.md               # teoría + qué completar + cómo correrlo
```

Workshops so far (see `README.md` for the up-to-date table):

| Semana | Paquete | Nodo(s) | Tema |
| --- | --- | --- | --- |
| 01 | `talkers_listeners` | `talker.py`, `listener.py` | pub/sub básico |
| 02 | `zigzag_mecanum` | `zigzag.py` | control de movimiento (ya resuelto, sin TODOs) |
| 03 | `evasion_obstaculos` | `evasor.py` | máquina de estados sobre lidar + odometría |
| 04 | `deteccion_color` | `detector.py`, `detector_scan.py` | visión (OpenCV/HSV) + fusión cámara-lidar |
| 05 | `launch_rviz` | (sin nodos) `launch/*.launch.py`, `rviz/*.rviz` | sección transversal: launch files y configs de RViz |

`PROXIMOS_WORKSHOPS.md` is a living roadmap/design doc for unwritten workshops, not a workshop itself — when a workshop described there gets written, it should move into its own `semana-NN-slug/` folder (with a README following the site's template) and be deleted from that file.

## Build and run

There is no local build system — packages are built via `colcon` from the parent ROS workspace:

```bash
cd ~/rosmaster_ws
colcon build --packages-select <paquete>   # e.g. talkers_listeners, evasion_obstaculos
source install/setup.bash
ros2 run <paquete> <ejecutable>
```

Workshops 03 and 04 additionally need the simulator running (`ros2 launch yahboom_rosmaster_gazebo ...`, world varies per workshop — see that workshop's README for the exact command). Workshop 01 runs standalone, no simulator or robot needed. Useful checks while iterating: `ros2 topic echo <topic>`, `ros2 topic hz <topic>`.

There are no automated tests in this repo (only the standard `ament_copyright`/`ament_flake8`/`ament_pep257`/`pytest` `test_depend` entries colcon adds by default — no actual test files exist per package).

## The pedagogical pattern (important when editing or adding a workshop)

Every node file follows the same shape, and it's the central teaching point repeated across READMEs — preserve it when touching node code:

- **Sensor callbacks only store the latest message** (e.g. `recibir_scan`, `recibir_odom`, `recibir_imagen`) — they never decide or act.
- **All decision logic lives in a single timer callback** (e.g. `maquina_de_estados`, `procesar_imagen`) running at a fixed `FRECUENCIA_HZ`, decoupled from sensor publish rates.
- State machines (semana 03) separate the **transition** check from the **action** dispatch, with excluding states (`ESTADO_AVANZAR`/`ESTADO_GIRAR`) and logging on every transition.

Node files that are workshop exercises leave real gaps in three places, each following an established convention:

1. **Inside the node's `.py` file** — either commented-out lines to uncomment (semana 01 style, e.g. [talker.py](semana-01-talkers-listeners/talkers_listeners/talkers_listeners/talker.py)) or a `pass`/stub function with a docstring that walks through the steps to implement (semana 03/04 style, e.g. `hay_obstaculo()` in [evasor.py](semana-03-evasion-obstaculos/evasion_obstaculos/evasion_obstaculos/evasor.py)).
2. **`setup.py`'s `entry_points`** — the `console_scripts` line mapping an executable name to `paquete.modulo:main` is commented out; students uncomment it. Without it `ros2 run` can't find the executable even if the code is correct.
3. **`package.xml`'s `<depend>` tags** — commented out; students add one per ROS import in the node code (non-ROS Python deps like numpy map to a different name, e.g. `numpy` → `python3-numpy`). Without these, `colcon build`/`rosdep` may fail to resolve dependencies.

When adding a new workshop or exercise, follow whichever TODO style (uncomment vs. stub-with-docstring) matches the complexity of that piece, and always leave TODOs 2 and 3 above so the "paquete ROS 2" mechanics get practiced too — see semana 03/04 READMEs' "El paquete ROS 2" section for how this is explained to students. Semana 05 varies the shape of those two because it is a launch-only package: `data_files` (installing `launch/` and `rviz/` into `share/`) instead of `entry_points`, and `<exec_depend>` instead of `<depend>`.

**Ground-truth solutions**: solved versions live alongside the exercise files with a `_gth` suffix (`evasor_gth.py`, `package_gth.xml`, `setup_gth.py`, `evasion_gth.launch.py`, `deteccion_color_gth.rviz`) and are gitignored — they are for local testing only, and what ships is always the TODO version at the un-suffixed name. When adding a new exercise file, add its `_gth` counterpart, and check `.gitignore` actually covers the new naming shape.

## Language and audience

READMEs, code comments, parameter names, and TODOs are all in **Spanish** (Argentine) — keep new content consistent with that. The audience is students with no prior ROS 2 experience; explanations build progressively, with later workshops (`README.md`) explicitly referencing concepts established in earlier ones (e.g. semana 03/04 READMEs link back to semana 01's "sensor callbacks only store data" rule) rather than re-explaining them. Preserve those cross-references when editing.
