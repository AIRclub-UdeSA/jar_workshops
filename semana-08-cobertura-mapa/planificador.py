"""Planificación de caminos sobre una grilla, sin ROS — separada para poder
testearla aislada (mismo criterio que trazado.py)."""
import heapq
import math

# (delta_fila, delta_columna, costo)
_VECINOS = [
    (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
    (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
    (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2)),
]


def a_estrella(libre, inicio, objetivo, grilla_costo=None):
    """
    A* sobre una grilla booleana `libre[fila, col]` (True = transitable),
    con vecindad de 8 (permite diagonales: costo base 1 recto, sqrt(2)
    diagonal). Encuentra el camino óptimo de una vez en vez de reaccionar
    solo a lo que el robot tiene enfrente — evita el mínimo local clásico
    de un controlador puramente reactivo (atraído por el objetivo,
    repelido por la pared, oscilando en el lugar sin decidirse a rodear el
    obstáculo).

    `grilla_costo` (opcional) es una grilla `float[fila, col]` con la misma
    forma que `libre` -- un costo extra por celda (0 = gratis, más alto =
    más caro). Sirve para que A* *prefiera* alejarse de las paredes cuando
    hay margen para hacerlo, sin prohibirlo (eso ya lo hace `libre` en
    forma dura): mismo mecanismo que un costmap inflado.

    Ya está resuelta toda la "plomería" del algoritmo: la cola de
    prioridad (`heapq`), el diccionario del mejor costo conocido por celda
    (`costo_g`), el registro de quién llevó a quién (`vino_de`) para
    reconstruir el camino al final, y el bucle principal que saca de la
    cola el candidato más prometedor.

    TODO: completar el cuerpo del `for` que recorre los 8 vecinos de la
    celda que se está expandiendo en esta vuelta del bucle (`actual`, con
    costo acumulado `g`; `df, dc, costo` ya vienen del vecino actual).

    Pasos sugeridos:
      1. Calculá la celda vecina: `fila_v, col_v = fila_a + df, col_a + dc`
         (`fila_a, col_a` son las coordenadas de `actual`).
      2. Si `fila_v, col_v` cae afuera de la grilla (`0 <= fila_v < alto`
         y `0 <= col_v < ancho`), `continue` -- no hay nada que evaluar.
      3. Si `libre[fila_v, col_v]` es `False` (es pared), `continue`.
      4. Si el movimiento es diagonal (`df != 0 and dc != 0`), chequeá que
         las dos celdas ortogonales adyacentes (`libre[fila_a + df, col_a]`
         y `libre[fila_a, col_a + dc]`) también sean transitables -- si
         alguna no lo es, `continue`: la diagonal estaría "cortando la
         esquina" de dos paredes que se tocan en diagonal, un hueco que en
         la realidad no existe.
      5. Armá `vecino = (fila_v, col_v)`.
      6. Calculá el costo de esta arista: arrancá con `costo_arista = costo`
         (el costo geométrico base, ya viene del vecino). Si `grilla_costo`
         no es `None`, multiplicalo por
         `1.0 + 0.5 * (grilla_costo[fila_a, col_a] + grilla_costo[fila_v, col_v])`.
      7. Calculá `nuevo_g = g + costo_arista` -- el costo total de llegar
         a `vecino` pasando por acá.
      8. Si `nuevo_g` es menor que el mejor costo conocido hasta ahora para
         `vecino` (`costo_g.get(vecino, math.inf)`), es un camino mejor que
         cualquiera que se haya visto antes: actualizá
         `costo_g[vecino] = nuevo_g`, guardá `vino_de[vecino] = actual`, y
         empujá `(nuevo_g + heuristica(vecino), nuevo_g, vecino)` a la cola
         con `heapq.heappush(abiertos, ...)`.
    """
    alto, ancho = libre.shape

    def heuristica(celda):
        return math.hypot(celda[0] - objetivo[0], celda[1] - objetivo[1])

    abiertos = [(heuristica(inicio), 0.0, inicio)]
    vino_de = {}
    costo_g = {inicio: 0.0}
    visitados = set()

    while abiertos:
        _, g, actual = heapq.heappop(abiertos)
        if actual in visitados:
            continue
        visitados.add(actual)

        if actual == objetivo:
            camino = [actual]
            while actual in vino_de:
                actual = vino_de[actual]
                camino.append(actual)
            camino.reverse()
            return camino

        fila_a, col_a = actual
        for df, dc, costo in _VECINOS:
            pass  # TODO: completar (ver docstring de a_estrella)

    return None
