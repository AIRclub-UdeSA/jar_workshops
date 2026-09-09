import math

import numpy as np
import scipy.ndimage as nd

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                        QoSReliabilityPolicy, qos_profile_sensor_data)
from rclpy.time import Time

from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Quaternion, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from tf2_ros import Buffer, TransformListener

from cobertura_mapa.planificador import a_estrella

FRECUENCIA_HZ = 10.0
REPLANIFICAR_CADA_N_TICKS = int(FRECUENCIA_HZ * 2)  # cada ~2s, aunque el objetivo no haya cambiado

ESTADO_INICIALIZANDO = 'inicializando'
ESTADO_EXPLORANDO = 'explorando'
ESTADO_TERMINADO = 'terminado'


def yaw_de_quaternion(q: Quaternion) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Explorador(Node):
    """
    Máquina de estados de 3 estados que gobierna la misión completa:

        INICIALIZANDO -> EXPLORANDO -> TERMINADO

    - **INICIALIZANDO**: espera a que un humano confirme la pose inicial
      con "2D Pose Estimate" en RViz (el robot real no arranca sabiendo
      dónde está) y a que el filtro de partículas tenga un rato para
      asentarse con el robot quieto antes de arrancar a moverse —
      moverse mientras la pose todavía no convergió compone el error.
    - **EXPLORANDO**: sigue el camino hasta la celda libre no cubierta más
      cercana. Transición a TERMINADO cuando no queda ninguna (volver a
      la base, o a cualquier otro punto, queda como desafío extra — ver
      el README).
    - **TERMINADO**: se queda quieto.

    Dentro de EXPLORANDO, el movimiento en sí **no** es
    reactivo puro: en vez de ir directo al objetivo sumando un vector
    atractivo y uno repulsivo (lo que en un pasillo angosto puede quedar
    oscilando en el lugar — el mínimo local clásico de un controlador
    puramente reactivo), primero se **planifica** con A* el camino óptimo
    (semana 08 depende de tener `a_estrella()` completa en
    `planificador.py`), y después se lo sigue waypoint a waypoint. El
    vector repulsivo del /scan queda como capa de seguridad secundaria
    (obstáculos que el mapa estático no conoce), no como mecanismo
    principal de esquive.

    A* corre sobre el `/map` original (resolución fina), no sobre la
    grilla de cobertura: esa grilla agrupa varias celdas del mapa en una
    sola a resolución más gruesa para que cubrir el área entera sea
    alcanzable, y una pared delgada puede ocupar menos de la mitad de un
    bloque agrupado — quedaría marcada "libre" y A* la atravesaría sin
    darse cuenta. La grilla de cobertura solo se usa para decidir **qué
    zona** visitar (la celda libre no cubierta más cercana); una vez
    elegida esa zona, el camino hasta ahí se planifica contra el mapa de
    verdad.

    Contra ese mapa fino se aplican además dos capas, mismo mecanismo que
    un costmap inflado (tipo Nav2, o el que arma el TP Final con
    `distance_transform_edt` + campana de Gauss sobre `/likelihood_map`,
    ya resueltas acá): un **filtro duro** descarta cualquier celda a menos
    de `radio_robot + margen_seguridad` de una pared (así A* nunca elige
    un pasillo más angosto que el propio chasis), y un **costo suave** que
    decae con la distancia a la pared hace que, entre las celdas que sí
    pasan el filtro, A* prefiera las más alejadas cuando hay margen para
    elegir.
    """

    def __init__(self):
        super().__init__('explorador')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('velocidad_maxima', 0.3)
        self.declare_parameter('ganancia_atractiva', 0.6)
        self.declare_parameter('ganancia_repulsiva', 0.15)
        self.declare_parameter('radio_repulsion', 0.4)
        self.declare_parameter('suavizado', 0.3)
        self.declare_parameter('tiempo_asentamiento_s', 3.0)
        # Donatello mide 0.30 x 0.20 m (rosmaster_x3_base.urdf.xacro); el
        # radio circunscripto (mitad de la diagonal) es ~0.18 m -- usamos
        # ese valor para no ligar una esquina del chasis contra la pared
        # sin importar la orientación con la que pase (es holonómico).
        self.declare_parameter('radio_robot', 0.18)
        self.declare_parameter('margen_seguridad', 0.05)
        self.declare_parameter('sigma_costo', 0.15)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.velocidad_maxima = self.get_parameter('velocidad_maxima').value
        self.ganancia_atractiva = self.get_parameter('ganancia_atractiva').value
        self.ganancia_repulsiva = self.get_parameter('ganancia_repulsiva').value
        self.radio_repulsion = self.get_parameter('radio_repulsion').value
        # Peso del comando nuevo contra el anterior (media móvil exponencial),
        # para que el resultado de seguir el camino no salte bruscamente de
        # un waypoint a otro.
        self.suavizado = self.get_parameter('suavizado').value
        self.tiempo_asentamiento_s = self.get_parameter('tiempo_asentamiento_s').value
        self.radio_robot = self.get_parameter('radio_robot').value
        self.margen_seguridad = self.get_parameter('margen_seguridad').value
        self.sigma_costo = self.get_parameter('sigma_costo').value
        self.vx_anterior = 0.0
        self.vy_anterior = 0.0
        self.pose_inicial_confirmada = False
        self.momento_confirmacion = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.libre_fino = None      # bool[alto, ancho], resolución del /map: transitable, para A*
        self.costo_fino = None      # float[alto, ancho]: costo extra por cercanía a una pared
        self.info_mapa = None       # MapMetaData del /map (resolución fina)
        self.no_cubiertas = None    # bool[alto, ancho], resolución de grilla_cobertura
        self.info_grilla = None     # MapMetaData de grilla_cobertura (resolución gruesa)
        self.ultimo_scan = None
        self.estado = ESTADO_INICIALIZANDO

        self.objetivo_celda = None  # (fila, col) al que corresponde self.camino
        self.camino = None          # lista de (fila, col), en orden, sin recorrer
        self.ticks_desde_replan = REPLANIFICAR_CADA_N_TICKS  # fuerza plan en el primer tick

        qos_grilla = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, 'map', self.recibir_mapa, qos_grilla)
        self.create_subscription(OccupancyGrid, 'grilla_cobertura', self.recibir_grilla, qos_grilla)
        self.create_subscription(LaserScan, 'scan', self.recibir_scan, qos_profile_sensor_data)
        self.create_subscription(
            PoseWithCovarianceStamped, 'initialpose', self.recibir_pose_inicial, 10)

        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_objetivo = self.create_publisher(Marker, 'objetivo_actual', 10)
        self.pub_camino = self.create_publisher(Path, 'camino_planificado', 10)
        self.pub_estado = self.create_publisher(String, 'estado', 10)

        self.timer = self.create_timer(1.0 / FRECUENCIA_HZ, self.controlar)

    # ---------- callbacks ----------

    def recibir_mapa(self, msg: OccupancyGrid):
        alto, ancho = msg.info.height, msg.info.width
        datos = np.array(msg.data, dtype=np.int16).reshape((alto, ancho))
        ocupado = datos != 0  # pared o desconocido: no confiar

        # Distancia real (en metros) de cada celda a la pared más cercana.
        dist_a_pared = nd.distance_transform_edt(~ocupado) * msg.info.resolution

        # Filtro duro: ninguna celda a menos de radio_robot + margen de una
        # pared es "libre" para A* -- sin esto, A* podría mandar a
        # Donatello por un pasillo más angosto que su propio chasis sin
        # saber que no entra (ver README de esta semana).
        radio_total = self.radio_robot + self.margen_seguridad
        self.libre_fino = (datos == 0) & (dist_a_pared >= radio_total)

        # Costo suave por encima del filtro duro: entre las celdas que sí
        # pasan, A* va a preferir las más alejadas de la pared cuando haya
        # margen para elegir -- mismo mecanismo (campana de Gauss sobre la
        # distancia a la pared) que un costmap inflado tipo Nav2.
        self.costo_fino = 100.0 * np.exp(-(dist_a_pared ** 2) / (2.0 * self.sigma_costo ** 2))

        self.info_mapa = msg.info

    def recibir_grilla(self, msg: OccupancyGrid):
        alto, ancho = msg.info.height, msg.info.width
        datos = np.array(msg.data, dtype=np.int16).reshape((alto, ancho))
        self.no_cubiertas = (datos == 100)
        self.info_grilla = msg.info

    def recibir_scan(self, msg: LaserScan):
        self.ultimo_scan = msg

    def recibir_pose_inicial(self, msg: PoseWithCovarianceStamped):
        # Reinicia el asentamiento cada vez (incluso si ya se había
        # confirmado antes): si el humano corrige la pose a mitad de
        # recorrido, el robot debe volver a quedarse quieto un momento
        # para que el filtro converja de nuevo antes de seguir moviéndose.
        self.pose_inicial_confirmada = True
        self.momento_confirmacion = self.get_clock().now()
        if self.estado != ESTADO_INICIALIZANDO:
            self.estado = ESTADO_INICIALIZANDO
            self.camino = None
            self.get_logger().info(f'Transición: * -> {ESTADO_INICIALIZANDO} (pose corregida desde RViz)')

    # ---------- geometría ----------

    def obtener_pose_robot(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except Exception:
            return None
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        yaw = yaw_de_quaternion(tf.transform.rotation)
        return x, y, yaw

    def celda_a_mundo(self, fila: int, col: int, info):
        x = info.origin.position.x + (col + 0.5) * info.resolution
        y = info.origin.position.y + (fila + 0.5) * info.resolution
        return x, y

    def celda_de(self, x: float, y: float, info):
        col = int((x - info.origin.position.x) / info.resolution)
        fila = int((y - info.origin.position.y) / info.resolution)
        return fila, col

    def celda_fina_segura(self, x: float, y: float):
        """Como celda_de(), pero para la grilla fina del /map (la que usa
        A*) y recortando al rango válido -- un punto justo en el borde del
        mapa no debería poder indexar afuera del array."""
        fila, col = self.celda_de(x, y, self.info_mapa)
        alto, ancho = self.libre_fino.shape
        fila = int(np.clip(fila, 0, alto - 1))
        col = int(np.clip(col, 0, ancho - 1))
        return fila, col

    def celda_no_cubierta_mas_cercana(self, fila_r: int, col_r: int):
        """
        TODO: de todas las celdas de `self.no_cubiertas` que son `True`
        (libres y todavía no vistas), devolver la más cercana a
        `(fila_r, col_r)` -- la celda donde está el robot ahora, en la
        grilla de cobertura.

        Devolvé una tupla `(fila, col)`, o `None` si no queda ninguna
        celda no cubierta (cobertura completa).

        Pasos sugeridos:
          1. `np.where(self.no_cubiertas)` te da dos arrays paralelos,
             `filas` y `cols`, con los índices de todas las celdas
             `True`.
          2. Si `len(filas) == 0`, no queda nada por cubrir: devolvé
             `None`.
          3. Calculá la distancia (al cuadrado, no hace falta la raíz
             para comparar) de cada candidata a `(fila_r, col_r)`:
             `dist2 = (filas - fila_r)**2 + (cols - col_r)**2` --
             vectorizado, una cuenta para las N celdas a la vez.
          4. `np.argmin(dist2)` te da el índice, dentro de `filas`/`cols`,
             de la candidata más cercana.
          5. Devolvé `(int(filas[i]), int(cols[i]))` con ese índice.
        """
        pass

    def vector_hacia(self, x_r, y_r, yaw_r, punto_mundo):
        """
        TODO: vector que apunta desde la pose del robot hacia
        `punto_mundo` (una tupla `(x, y)` en `map`), expresado en el
        **frame del robot** -- porque Donatello es holonómico (mecanum,
        ver semana 02): puede moverse en diagonal sin girar el chasis
        primero, así que el controlador manda `linear.x`/`linear.y`
        directo en vez de "girar hasta apuntar, avanzar" como una base
        diferencial. Para eso, el vector "hacia dónde ir" tiene que estar
        expresado relativo a hacia dónde mira el robot ahora, no en
        coordenadas de `map`.

        Devolvé una tupla `(vx, vy, dist)`: la componente x e y del
        vector en el frame del robot, y la distancia en línea recta hasta
        `punto_mundo` (esto último ya lo necesitás calcular en el camino,
        y lo usa quien te llama para saber si ya llegaste).

        Pasos sugeridos:
          1. Separá `punto_mundo` en `gx, gy`.
          2. Calculá el vector en `map`: `dx, dy = gx - x_r, gy - y_r`.
          3. Calculá `dist = math.hypot(dx, dy)`. Si es menor a `1e-3`
             (prácticamente en el punto), devolvé `(0.0, 0.0, dist)`
             directo -- dividir por una distancia ~0 en el paso siguiente
             rompería todo.
          4. Frenar a medida que se acerca, en vez de mantener velocidad
             constante hasta pegar contra el punto y frenar en seco:
             `escala = min(dist, 1.0) * self.ganancia_atractiva`, y de ahí
             `dir_x_mundo, dir_y_mundo = (dx / dist) * escala, (dy / dist) * escala`.
          5. Rotá ese vector de `map` al frame del robot usando `yaw_r`
             (mismo tipo de rotación que ya usaste transformando puntos
             del scan a `map` en semanas anteriores, pero en sentido
             inverso -- acá es de `map` al frame del robot):
               `c, s = math.cos(yaw_r), math.sin(yaw_r)`
               `vx = c * dir_x_mundo + s * dir_y_mundo`
               `vy = -s * dir_x_mundo + c * dir_y_mundo`
          6. Devolvé `(vx, vy, dist)`.
        """
        pass

    def vector_repulsivo(self):
        """Capa de seguridad secundaria: empuja lejos de cualquier punto
        del /scan más cerca que radio_repulsion. El camino de A* ya evita
        las paredes que conoce la grilla; esto cubre lo que la grilla no
        sabe (resolución gruesa, obstáculos dinámicos)."""
        if self.ultimo_scan is None:
            return 0.0, 0.0

        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.ultimo_scan.header.frame_id, Time())
        except Exception:
            return 0.0, 0.0

        msg = self.ultimo_scan
        rangos = np.array(msg.ranges)
        n = len(rangos)
        angulos = msg.angle_min + np.arange(n) * msg.angle_increment
        validos = np.isfinite(rangos) & (rangos > msg.range_min)
        if not np.any(validos):
            return 0.0, 0.0

        r = rangos[validos]
        a = angulos[validos]
        xs_laser = r * np.cos(a)
        ys_laser = r * np.sin(a)

        tx, ty = tf.transform.translation.x, tf.transform.translation.y
        tyaw = yaw_de_quaternion(tf.transform.rotation)
        c, s = math.cos(tyaw), math.sin(tyaw)
        xs_base = tx + c * xs_laser - s * ys_laser
        ys_base = ty + s * xs_laser + c * ys_laser

        dist = np.maximum(np.hypot(xs_base, ys_base), 1e-6)
        cerca = dist < self.radio_repulsion
        if not np.any(cerca):
            return 0.0, 0.0

        magnitud = (1.0 / dist[cerca] - 1.0 / self.radio_repulsion)
        rx = -magnitud * (xs_base[cerca] / dist[cerca])
        ry = -magnitud * (ys_base[cerca] / dist[cerca])
        return float(np.sum(rx)) * self.ganancia_repulsiva, float(np.sum(ry)) * self.ganancia_repulsiva

    # ---------- planificación ----------

    def replanificar_si_hace_falta(self, objetivo_grueso, objetivo_mundo, x_r, y_r):
        """Planifica con A* sobre la grilla fina del /map -- objetivo_grueso
        es la celda de la grilla de cobertura, solo se usa para decidir
        *cuándo* replanificar (cambió la zona elegida); el camino en sí se
        calcula entre las celdas finas del robot y del objetivo."""
        cambio_de_objetivo = objetivo_grueso != self.objetivo_celda
        self.ticks_desde_replan += 1
        toca_por_tiempo = self.ticks_desde_replan >= REPLANIFICAR_CADA_N_TICKS
        sin_camino = not self.camino

        if not (cambio_de_objetivo or toca_por_tiempo or sin_camino):
            return

        self.ticks_desde_replan = 0
        self.objetivo_celda = objetivo_grueso
        celda_robot = self.celda_fina_segura(x_r, y_r)
        celda_objetivo = self.celda_fina_segura(*objetivo_mundo)
        camino = a_estrella(self.libre_fino, celda_robot, celda_objetivo, self.costo_fino)
        if camino is None:
            self.get_logger().warn(
                f'No hay camino libre hasta la zona {objetivo_grueso}, la descarto de la cobertura.')
            self.no_cubiertas[objetivo_grueso] = False
            self.camino = None
            return

        self.camino = camino
        self.publicar_camino(camino)

    def siguiente_punto_del_camino(self, x_r, y_r):
        """Descarta del camino los waypoints ya alcanzados y devuelve el
        primero que falta, en coordenadas de `map`. El camino está en
        celdas finas, así que la tolerancia es varias celdas (si fuera una
        sola, el seguimiento sería demasiado fino/lento de recorrer)."""
        if not self.camino:
            return None
        tolerancia = max(self.info_mapa.resolution * 3.0, 0.1)
        while len(self.camino) > 1:
            wx, wy = self.celda_a_mundo(*self.camino[0], self.info_mapa)
            if math.hypot(wx - x_r, wy - y_r) < tolerancia:
                self.camino.pop(0)
            else:
                break
        return self.celda_a_mundo(*self.camino[0], self.info_mapa)

    # ---------- visualización ----------

    def publicar_objetivo(self, objetivo_mundo):
        marcador = Marker()
        marcador.header.frame_id = self.map_frame
        marcador.header.stamp = self.get_clock().now().to_msg()
        marcador.ns = 'objetivo_actual'
        marcador.id = 0
        marcador.type = Marker.SPHERE
        marcador.action = Marker.ADD
        marcador.pose.position.x = objetivo_mundo[0]
        marcador.pose.position.y = objetivo_mundo[1]
        marcador.pose.orientation.w = 1.0
        marcador.scale.x = marcador.scale.y = marcador.scale.z = 0.25
        marcador.color.r = 1.0
        marcador.color.g = 0.6
        marcador.color.a = 1.0
        self.pub_objetivo.publish(marcador)

    def publicar_camino(self, camino):
        msg = Path()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for fila, col in camino:
            x, y = self.celda_a_mundo(fila, col, self.info_mapa)
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.pub_camino.publish(msg)

    # ---------- máquina de estados ----------

    def transicionar(self, pose):
        """
        TODO: mismo criterio que la máquina de estados de semana 03 --
        primero se decide la transición (esta función), después se actúa
        según el estado ya actualizado (`actuar()`, ya resuelta).

        Completá los 2 `if` de más abajo, uno por estado (el bloque
        `if self.estado != anterior: ...` del final, que loguea cada
        transición, ya está resuelto -- no lo toques).

          - **Si `self.estado == ESTADO_INICIALIZANDO`**: pasar a
            `ESTADO_EXPLORANDO` solo cuando se cumplen TODAS estas
            condiciones a la vez:
              a. `pose is not None` (hay tf).
              b. `self.no_cubiertas is not None` (ya llegó
                 `/grilla_cobertura`).
              c. `self.libre_fino is not None` (ya llegó `/map`).
              d. `self.pose_inicial_confirmada` es `True` (un humano ya
                 clickeó "2D Pose Estimate").
              e. Además, pasó al menos `self.tiempo_asentamiento_s`
                 segundos desde esa confirmación -- calculalo con
                 `(self.get_clock().now() - self.momento_confirmacion).nanoseconds / 1e9`.
            Si se cumplen las 5, cambiá `self.estado` a
            `ESTADO_EXPLORANDO`.
          - **Si `self.estado == ESTADO_EXPLORANDO`**: pasar a
            `ESTADO_TERMINADO` cuando no queda ninguna celda no cubierta
            -- `self.no_cubiertas is not None and not np.any(self.no_cubiertas)`.
        """
        anterior = self.estado

        # TODO: completar los 2 `if`/`elif` (ver docstring de arriba)

        if self.estado != anterior:
            self.get_logger().info(f'Transición: {anterior} -> {self.estado}')

    def objetivo_de_la_mision(self, fila_r, col_r):
        """Celda (gruesa, de la grilla de cobertura) libre no cubierta más
        cercana al robot -- solo se llama en EXPLORANDO, así que siempre
        hay al menos una (si no, transicionar() ya pasó a TERMINADO)."""
        return self.celda_no_cubierta_mas_cercana(fila_r, col_r)

    def actuar(self, pose):
        """Movimiento en EXPLORANDO: planifica con A* y sigue el camino
        hasta la zona objetivo."""
        x_r, y_r, yaw_r = pose
        fila_r_gruesa, col_r_gruesa = self.celda_de(x_r, y_r, self.info_grilla)

        objetivo_grueso = self.objetivo_de_la_mision(fila_r_gruesa, col_r_gruesa)
        objetivo_mundo = self.celda_a_mundo(*objetivo_grueso, self.info_grilla)
        self.publicar_objetivo(objetivo_mundo)

        self.replanificar_si_hace_falta(objetivo_grueso, objetivo_mundo, x_r, y_r)

        punto_seguimiento = self.siguiente_punto_del_camino(x_r, y_r) or objetivo_mundo
        vx_atr, vy_atr, _ = self.vector_hacia(x_r, y_r, yaw_r, punto_seguimiento)
        vx_rep, vy_rep = self.vector_repulsivo()

        vx, vy = vx_atr + vx_rep, vy_atr + vy_rep
        magnitud = math.hypot(vx, vy)
        if magnitud > self.velocidad_maxima:
            vx *= self.velocidad_maxima / magnitud
            vy *= self.velocidad_maxima / magnitud

        vx = self.suavizado * vx + (1.0 - self.suavizado) * self.vx_anterior
        vy = self.suavizado * vy + (1.0 - self.suavizado) * self.vy_anterior
        self.vx_anterior, self.vy_anterior = vx, vy

        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        self.pub_cmd_vel.publish(cmd)

    # ---------- control ----------

    def controlar(self):
        pose = self.obtener_pose_robot()

        self.pub_estado.publish(String(data=self.estado))
        self.transicionar(pose)

        if self.estado in (ESTADO_INICIALIZANDO, ESTADO_TERMINADO) or pose is None:
            self.pub_cmd_vel.publish(Twist())
            return

        self.actuar(pose)


def main(args=None):
    rclpy.init(args=args)
    nodo = Explorador()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.pub_cmd_vel.publish(Twist())
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
