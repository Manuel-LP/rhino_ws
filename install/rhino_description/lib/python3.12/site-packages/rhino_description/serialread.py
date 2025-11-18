import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import time
import math

class SerialToJointState(Node):
    def __init__(self):
        super().__init__('serial_to_joint_state')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        # self.timer = self.create_timer(0.05, self.timer_callback)  # Publicar cada 0.1 segundos
        self.ser = serial.Serial(
            port='/dev/ttyUSB0',  # Cambia esto al puerto adecuado para tu sistema
            baudrate=9600,
            bytesize=serial.SEVENBITS,
            parity=serial.PARITY_ODD,
            stopbits=serial.STOPBITS_TWO,
            timeout=0.5
        )
        self.motor_ids = ['F', 'E', 'D', 'C', 'B', 'A']#self.motor_ids = ['F', 'E', 'D', 'C', 'B']  # Identificadores de los motores
        self.motor_specs = {
            'B': (-4558, 4558, 360),
            'C': (-1300, 11000, 250),#250
            'D': (6473,-3690,  180),#180
            'E': (-580, 4670, 150),
            'F': (-3005, 3005, 350),
            'A': (-240, 0, 25)
            # 'C': (-2500, 4500, 250),
            # 'D': (4173, -2629, 180),
            # 'E': (-2500, 2500, 150),
            # 'F': (-3091, 2968, 350)
        }
    def position_to_radians(self, position, min_position, max_position, degrees_of_rotation):
        # Calcular grados centrando el rango en 0
        degrees = (position ) * degrees_of_rotation / (max_position -   min_position)
        # Convertir de grados a radianes
        radians = math.radians(degrees)
        # Redondear a la resolución más cercana de 0.2π
        resolution = 0.0001 * math.pi
        radians_rounded = round(radians / resolution) * resolution
        return radians_rounded
    


    def leer_y_publicar(self):

                # Leer cada motor y almacenar la posición
                posiciones = []
                for motor_id in self.motor_ids:
                    comando = f'PA,{motor_id}\n'.encode('utf-8')
                    self.ser.write(comando)
                    respuesta = self.ser.readline().decode('utf-8').strip()

                    try:
                        posicion = float(respuesta)
                        min_pos, max_pos, degrees_rot = self.motor_specs[motor_id]
                        posicion_radianes = self.position_to_radians(posicion, min_pos, max_pos, degrees_rot)

                        self.get_logger().info(f'Respuesta del motor {motor_id}: {respuesta}')

                        posiciones.append(posicion_radianes)
                        if motor_id == 'A':
                            posiciones.append(posicion_radianes)
                            self.get_logger().info(f'2Respuesta del motor en radianes {motor_id}: {respuesta}')                   
                    except ValueError:
                        posiciones.append(0.0)  # Si no hay respuesta, asignar 0.0
                        self.get_logger().info('NULL')

                joint_state_msg = JointState()
                joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                joint_state_msg.name = ['base_tronco', 'hombro_brazo', 'codo_antebrazo', 'muneca', 'engranaje_giro', 'gripper_izquierdo', 'gripper_right']

                joint_state_msg.position = posiciones
                self.publisher_.publish(joint_state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SerialToJointState()

    try:
        # --- BUCLE PRINCIPAL ---
        while rclpy.ok():
            # 1. Ejecutamos tu lógica de hardware
            node.leer_y_publicar()
            
            # 2. Permitimos que ROS procese cosas internas brevemente (sin bloquear)
            rclpy.spin_once(node, timeout_sec=0)
            
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
