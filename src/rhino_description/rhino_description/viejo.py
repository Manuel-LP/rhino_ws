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
        self.timer = self.create_timer(0.1, self.timer_callback)  # Publicar cada 0.1 segundos
        self.ser = serial.Serial(
            port='/dev/ttyUSB0',  # Cambia esto al puerto adecuado para tu sistema
            baudrate=9600,
            bytesize=serial.SEVENBITS,
            parity=serial.PARITY_ODD,
            stopbits=serial.STOPBITS_TWO,
            timeout=0.5
        )
        self.motor_ids = ['F', 'E', 'D', 'C', 'B', 'A']  # Identificadores de los motores
        self.motor_specs = {
            'B': (-2300, 2300, 360),
            'C': (-2500, 4500, 250),
            'D': (4173, -2629, 180),
            'E': (-2500, 2500, 150),
            'F': (-3091, 2968, 350),
            'A': (-240, 0, 25)
        }

    def position_to_radians(self, position, min_position, max_position, degrees_of_rotation):
        # Calcular el centro del rango
        mid_position = (max_position + min_position) / 2
        # Calcular grados centrando el rango en 0
        degrees = (position - mid_position) * degrees_of_rotation / (max_position -   min_position)
        # Convertir de grados a radianes
        radians = math.radians(degrees)
        # Redondear a la resolución más cercana de 0.2π
        resolution = 0.0001 * math.pi
        radians_rounded = round(radians / resolution) * resolution
        return radians_rounded

    def timer_callback(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [motor_id for motor_id in self.motor_ids]  # Nombres de las articulaciones

        posiciones = []
        for motor_id in self.motor_ids:
            comando = f'PA,{motor_id}\n'.encode('utf-8')
            self.ser.write(comando)
            time.sleep(0.1)
            if self.ser.in_waiting > 0:
                respuesta = self.ser.readline().decode('utf-8').strip()
                self.get_logger().info(f'Respuesta del motor {motor_id}: {respuesta}')
                try:
                    posicion = float(respuesta)
                    min_pos, max_pos, degrees_rot = self.motor_specs[motor_id]
                    posicion_radianes = self.position_to_radians(posicion, min_pos, max_pos, degrees_rot)
                    posiciones.append(posicion_radianes)
                    if motor_id == 'A':
                        posiciones.append(posicion_radianes)
                        self.get_logger().info(f'2Respuesta del motor en radianes {motor_id}: {respuesta}')
                except ValueError:
                    posicion_radianes = 0.0  # Manejar errores en la conversión
                posiciones.append(posicion_radianes)
            else:
                posiciones.append(0.0)  # Si no hay respuesta, asignar 0.0

        joint_state_msg.name = ['base_tronco', 'hombro_brazo', 'codo_antebrazo', 'muneca', 'engranaje_giro', 'gripper_izquierdo', 'gripper_right']

        joint_state_msg.position = posiciones
        self.publisher_.publish(joint_state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SerialToJointState()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()
