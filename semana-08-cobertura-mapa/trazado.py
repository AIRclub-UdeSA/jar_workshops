"""Geometría de grilla pura, sin ROS — separada para poder testearla aislada
(mismo criterio que agrupar_en_rachas() en semana 07)."""


def trazar_rayo(fila_desde: int, col_desde: int, fila_hasta: int, col_hasta: int):
    """
    TODO: trazar la línea de Bresenham entre dos celdas de una grilla 2D.

    Por qué hace falta esto en particular, y no por ejemplo muestrear la
    línea a pasos fijos con `np.linspace`: un paso fijo puede saltearse
    una celda si no se elige lo bastante fino, sobre todo en rayos en
    diagonal — y "lo bastante fino" es un parámetro más para calibrar mal.
    Bresenham no tiene ese problema: por construcción visita **todas** las
    celdas que la línea atraviesa, sin huecos ni celdas repetidas, con
    aritmética entera nada más.

    Devolvé la lista de celdas `(fila, columna)` del camino, desde
    `(fila_desde, col_desde)` hasta `(fila_hasta, col_hasta)` (las dos
    incluidas), en orden.

    Pasos sugeridos (es el algoritmo clásico — la idea es que lo
    implementes siguiendo estos pasos, no que derives por qué funciona):
      1. Convertí los 4 argumentos a `int` y guardalos como `f0, c0` (el
         origen) y `f1, c1` (el destino).
      2. Calculá `dc = abs(c1 - c0)` y `df = abs(f1 - f0)`: cuánto hay que
         moverse en cada eje, sin importar el signo.
      3. Calculá el signo de cada paso: `sc = 1 if c0 < c1 else -1`, y lo
         mismo para `sf` a partir de `f0`/`f1`.
      4. Inicializá `error = dc - df`, y `f, c = f0, c0` (la celda actual
         del recorrido, arranca en el origen).
      5. Bucle (`while True`): en cada vuelta, agregá `(f, c)` a la lista
         de celdas. Si `(f, c)` ya es el destino, cortá el bucle (`break`).
      6. Si no, calculá `e2 = 2 * error`. Si `e2 > -df`, restale `df` a
         `error` y sumale `sc` a `c` (avanza en columna). Si `e2 < dc`,
         sumale `dc` a `error` y sumale `sf` a `f` (avanza en fila) — ojo
         que las dos condiciones pueden darse en la misma vuelta (ahí es
         cuando el paso es diagonal).
      7. Devolvé la lista de celdas una vez que el bucle corta.
    """
    pass
