#!/usr/bin/env python3
"""Verifica que cada package.xml del repo sea XML bien formado.

colcon descubre paquetes con un parseo laxo de package.xml, así que un
error de sintaxis ahí (una etiqueta sin cerrar, por ejemplo) no alcanza
para que `colcon build` falle. Este chequeo lo cubre explícitamente.
"""
import pathlib
import sys
import xml.etree.ElementTree as ET


def main() -> int:
    raiz = pathlib.Path(__file__).resolve().parents[2]
    archivos = sorted(
        a for a in raiz.rglob("package.xml") if ".git" not in a.parts
    )

    fallo = False
    for archivo in archivos:
        relativo = archivo.relative_to(raiz)
        try:
            ET.parse(archivo)
        except ET.ParseError as exc:
            print(f"ERROR: {relativo} no es XML válido: {exc}")
            fallo = True
        else:
            print(f"OK: {relativo}")

    return 1 if fallo else 0


if __name__ == "__main__":
    sys.exit(main())
