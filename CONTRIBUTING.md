# Contribuir a jar_workshops

¡Gracias por querer contribuir! Este repo contiene el código de los workshops asincrónicos del [Challenge JAR](https://github.com/AIRclub-UdeSA/jar_site) (AIR Club UdeSA).

## Índice

- [Quick Start](#quick-start)
- [Proceso de Pull Request](#proceso-de-pull-request)
- [Guías](#guías)

## Quick Start

```bash
# 1. Cloná el repo dentro de src/ de tu workspace ROS
#    (el mismo donde tenés yahboom_rosmaster, ver la guía de setup:
#    https://airclub-udesa.github.io/jar_site/setup/simulador/)
cd ~/rosmaster_ws/src
git clone https://github.com/AIRclub-UdeSA/jar_workshops.git
cd jar_workshops

# 2. Creá una branch para tu cambio
git checkout -b feature/mi-cambio

# 3. Hacé tu cambio y probalo localmente
cd ~/rosmaster_ws
colcon build --packages-select paquete_de_la_semana  # reemplazá por el paquete real
source install/setup.bash

# 4. Subí la branch y abrí un PR contra main
git add .
git commit -m "feat: mi cambio"
git push -u origin feature/mi-cambio
```

## Proceso de Pull Request

La rama `main` está protegida en GitHub (branch protection rule, classic). Todo cambio a `main` tiene que pasar por un pull request:

1. Creá una branch para tu cambio (`git checkout -b feature/mi-cambio`)
2. Subí la branch y abrí un PR contra `main`
3. Se necesita al menos 1 aprobación de otra persona (distinta del autor) antes de poder mergear
4. Resolvé todos los comentarios de revisión antes de mergear
5. Si se suben commits nuevos después de una aprobación, esa aprobación se invalida y hay que volver a pedir revisión
6. El último commit del PR también tiene que estar aprobado por alguien distinto de quien lo subió
7. No se puede pushear directo a `main`, ni siquiera los admins del repo pueden saltearse estas reglas

## Guías

**Hacé:**

- Mantené los cambios enfocados en un solo workshop o tema
- Probá tu cambio contra el simulador antes de abrir el PR
- Seguí la estructura de paquete existente (`ament_python`)

**No hagas:**

- No subas datos sensibles (tokens, paths locales, credenciales, etc.)
- No mezcles cambios de varios workshops en un mismo PR sin necesidad
