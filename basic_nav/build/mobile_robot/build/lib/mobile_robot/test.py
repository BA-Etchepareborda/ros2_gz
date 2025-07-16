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
        self.locked = False
        self.previous_state = ''

    def zona_menos_ocupada(self, rangos: dict, total_puntos: int):
        # Dividimos en 3 zonas: izquierda (0-1/3), centro (1/3-2/3), derecha (2/3-3/3)
        izquierda = []
        centro = []
        derecha = []


        if(self.locked == True):
            for idx, dist in rangos.items():
                if idx < total_puntos // 2:
                    derecha.append(dist)
                else:
                    izquierda.append(dist)

        else:
            for idx, dist in rangos.items():
                if idx < total_puntos // 3:
                    derecha.append(dist)
                elif idx < 2 * total_puntos // 3:
                    centro.append(dist)
                else:
                    izquierda.append(dist)


        # Calcular el promedio de distancia de cada zona
        zonas = {
            'izquierda': np.mean(izquierda) if izquierda else 0,
            'centro': np.mean(centro) if centro else 0,
            'derecha': np.mean(derecha) if derecha else 0,
        }

        # Devolver la zona con más espacio
        zona_libre = max(zonas.keys(), key=lambda k: zonas[k])
        return zona_libre


    def giro(self,dir,vel):
        lin = 0.15
        if(self.locked == True):
             lin = -0.15
        if(dir == 'izquierda'):
            self.control_msg.linear.x = lin
            self.control_msg.angular.z = vel
        elif (dir == 'derecha'):
            self.control_msg.linear.x = lin
            self.control_msg.angular.z = -1.0 * vel
        else:
            self.control_msg.linear.x = vel
            self.control_msg.angular.z = 0.0
        return 0
    

    def lidar_callback(self, msg: LaserScan):        
        ###Desestimo mediciones menores al minimo o mayores al maximo###
        ranges = np.array(msg.ranges)
        # Reemplazamos los inf por 5.9
        ranges = np.where(np.isinf(ranges), 5.9, ranges)
        # Ignorar las 3 mediciones de cada lado
        if len(ranges) > 6:
            ranges[:1] = 0.0     # izquierda extrema
            ranges[-1:] = 0.0    # derecha extrema
        # Aplicamos filtro: solo valores dentro de un rango razonable (0.1 a 6.0)
        mask = (ranges >= 0.1) & (ranges <= 6.0)
        indices_validos = np.where(mask)[0]
        valores_validos = ranges[mask]
        # Diccionario con índice:distancia
        rangos_filtrados = dict(zip(indices_validos, valores_validos))
        # Ejemplo: verificar si algún valor filtrado es menor a 1.5
        ocupado = any(dist < 2 for dist in rangos_filtrados.values())
        muy_cerca = any(dist < 1.2 for dist in rangos_filtrados.values())

        if(sum(rangos_filtrados.values()) < 22 or muy_cerca):
            self.locked = True  #Estado de bloqueo, cuando la suma es < 20 y ademas tengo el estado previo en algo, giro para el estado previo
        else:
            self.locked = False

        if (self.locked == False):
            self.previous_state = ''
        
        vel = 0.5
        if (ocupado and (self.locked and self.previous_state == '')): #Si esta bloqueado y no se cual es la direccion que voy a estar tomando
                zona = self.zona_menos_ocupada(rangos_filtrados, len(msg.ranges)) #Consigo la zona mas disponible
                self.previous_state = zona
                self.get_logger().info(f"Bloqueado. Zona más libre: {zona}")

        elif(ocupado and (self.locked and self.previous_state != '')):
                self.giro(self.previous_state,vel)
                self.get_logger().info(f"Bloqueado. Zona más libre guardada: {self.previous_state}")

        elif(ocupado and self.locked == False):
                zona = self.zona_menos_ocupada(rangos_filtrados, len(msg.ranges)) #Consigo la zona mas disponible
                self.giro(zona,vel) #Gira hacia el 
                self.get_logger().info(f"Ocupado. Giro hacia: {zona}")
        else:
                self.giro('centro',1.0) #No esta ocupado, va para adelante nomas
                self.get_logger().info(f"Libre. Giro hacia: el medio")

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
    