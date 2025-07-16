import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import time

class LidarController(Node):
    def __init__(self):
        super().__init__('lidar_controller')

        # Publicador de velocidad
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Suscriptor al LiDAR
        self.subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            10
        )

        # Inicializamos mensaje de control
        self.control_msg = Twist()

        # Control loop rate
        self.control_rate_hz = 2 # mensajes por segundo

    def zona_menos_ocupada(self, rangos: dict, total_puntos: int):
        # Dividimos en 3 zonas: izquierda (0-1/3), centro (1/3-2/3), derecha (2/3-3/3)
        izquierda = []
        centro = []
        derecha = []

        for idx, dist in rangos.items():
            if idx < total_puntos // 3:
                izquierda.append(dist)
            elif idx < 2 * total_puntos // 3:
                centro.append(dist)
            else:
                derecha.append(dist)

        # Calcular el promedio de distancia de cada zona
        zonas = {
            'izquierda': np.mean(izquierda) if izquierda else 0,
            'centro': np.mean(centro) if centro else 0,
            'derecha': np.mean(derecha) if derecha else 0,
        }

        # Devolver la zona con más espacio
        zona_libre = max(zonas.keys(), key=lambda k: zonas[k])
        return zona_libre

    def lidar_callback(self, msg: LaserScan):
        # Acá tenés los datos del LiDAR en msg.ranges
        # Podés implementar tu lógica de control usando esos datos
        # Acá va tu algoritmo:
        # ------------------------------
        # Podés usar: self.control_msg.linear.x = ...
        #             self.control_msg.angular.z = ...
        # ------------------------------
        # ESTE ES EL LUGAR PARA PONER TU LÓGICA 👇
        
        ###Desestimo mediciones menores al minimo o mayores al maximo###
        ranges = np.array(msg.ranges)

        # Reemplazamos los inf por 5.0
        ranges = np.where(np.isinf(ranges), 5.9, ranges)

        # Aplicamos filtro: solo valores dentro de un rango razonable (0.1 a 6.0)
        mask = (ranges >= 0.1) & (ranges <= 6.0)

        indices_validos = np.where(mask)[0]
        valores_validos = ranges[mask]

        # Diccionario con índice:distancia
        rangos_filtrados = dict(zip(indices_validos, valores_validos))

        # Ejemplo: verificar si algún valor filtrado es menor a 1.5
        ocupado = any(dist < 2 for dist in rangos_filtrados.values())
        
        self.control_msg.linear.x = 1.0
        self.control_msg.angular.z = 0.0

        if ocupado:
            zona = self.zona_menos_ocupada(rangos_filtrados, len(msg.ranges))
            self.get_logger().info(f"Obstáculo detectado. Zona más libre: {zona}")

            if zona == 'izquierda':
                self.control_msg.linear.x = 0.0
                self.control_msg.angular.z = 1.0  # Gira izquierda
            elif zona == 'derecha':
                self.control_msg.linear.x = 0.0
                self.control_msg.angular.z = -1.0  # Gira derecha
            else:
                self.control_msg.linear.x = 0.0
                self.control_msg.angular.z = 0.0  # Gira lento al centro
        else:
                self.control_msg.linear.x = 1.0
                self.control_msg.angular.z = 0.0

    def run(self):
        try:
            while rclpy.ok():
                self.publisher.publish(self.control_msg)
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(1.0 / self.control_rate_hz)
        except KeyboardInterrupt:
            self.get_logger().info("Nodo interrumpido con Ctrl+C")
        finally:
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LidarController()
    node.run()
    rclpy.shutdown()




if __name__ == '__main__':
    main()
    