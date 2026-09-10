#!/usr/bin/env python3
"""Verifica que los entry points (console_scripts) de cada paquete
carguen, sin ejecutarlos.

`colcon build` no falla si un entry point de setup.py apunta a una
función que no existe: ese error recién aparece al correr `ros2 run`.
Este chequeo importa cada entry point (equivalente a lo que hace
`ros2 run` antes de llamar a `main()`) sin invocarlo, así no hace falta
resolver los TODOs del ejercicio para validar el wiring del paquete.

Requiere correrse después de `colcon build`, con el workspace ya
sourceado (`source install/setup.bash`).
"""
import importlib.metadata
import subprocess
import sys


def paquetes_del_workspace() -> list[str]:
    salida = subprocess.run(
        ["colcon", "list", "--names-only"],
        capture_output=True,
        text=True,
        check=True,
    )
    return sorted(salida.stdout.split())


def main() -> int:
    fallo = False
    for paquete in paquetes_del_workspace():
        try:
            dist = importlib.metadata.distribution(paquete)
        except importlib.metadata.PackageNotFoundError:
            print(f"ERROR: {paquete} no quedó instalado en el workspace")
            fallo = True
            continue

        for ep in dist.entry_points.select(group="console_scripts"):
            try:
                ep.load()
            except Exception as exc:
                print(
                    f"ERROR: {paquete} - entry point '{ep.name}' no se "
                    f"pudo cargar: {exc}"
                )
                fallo = True
            else:
                print(f"OK: {paquete} - {ep.name}")

    return 1 if fallo else 0


if __name__ == "__main__":
    sys.exit(main())
