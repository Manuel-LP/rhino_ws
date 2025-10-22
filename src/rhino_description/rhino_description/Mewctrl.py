import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import time
import math
import serial.tools.list_ports


class NewConrol(Node):

    def __init__(self):
        super().__init__('serial_to_joint_state')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(1, self.timer_callback)  # Publicar cada 1 segundo
        self.timer = self.create_timer(0.2, self.motor_callback)  # Publicar cada 0.02 segundos

        self.ser = serial.Serial(
            port='/dev/ttyUSB0',
            baudrate=9600,
            bytesize=serial.SEVENBITS,
            parity=serial.PARITY_ODD,
            stopbits=serial.STOPBITS_TWO,
            timeout=1
        )
        self.motor_ids_index = 0
        self.posiciones = []
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
        # Calcular el centro del rango
        #mid_position = (max_position + min_position) / 2
        # Calcular grados centrando el rango en 0
        degrees = (position ) * degrees_of_rotation / (max_position -   min_position)
        # Convertir de grados a radianes
        radians = math.radians(degrees)
        # Redondear a la resolución más cercana de 0.2π
        resolution = 0.0001 * math.pi
        radians_rounded = round(radians / resolution) * resolution
        return radians_rounded
    
    

    def motor_callback(self):
            # if self.motor_ids[self.motor_ids_index] == 'F':
            #     joint_state_msg.name.append('base_tronco')
            # elif self.motor_ids[self.motor_ids_index] == 'E':
            #     joint_state_msg.name.append('hombro_brazo')
            # elif self.motor_ids[self.motor_ids_index] == 'D':
            #     joint_state_msg.name.append('codo_antebrazo')
            # elif self.motor_ids[self.motor_ids_index] == 'C':
            #     joint_state_msg.name.append('muneca')
            # elif self.motor_ids[self.motor_ids_index] == 'B':
            #     joint_state_msg.name.append('engranaje_giro')
            # elif self.motor_ids[self.motor_ids_index] == 'A':
            #     joint_state_msg.name.append('gripper_izquierdo')
            #     joint_state_msg.name.append('gripper_right')
        motor = self.motor_ids[self.motor_ids_index]
        comando = f'PA,{motor}\n'.encode('utf-8')
        self.ser.write(comando)
        if self.ser.in_waiting > 0:
                respuesta = self.ser.readline().decode('utf-8').strip()
                self.get_logger().info(f'Respuesta del motor {motor}: {respuesta}')
                try:
                    posicion = float(respuesta)
                    min_pos, max_pos, degrees_rot = self.motor_specs[motor]
                    posicion_radianes = self.position_to_radians(posicion, min_pos, max_pos, degrees_rot)
                except ValueError:
                    posicion_radianes = 0.0
                self.posiciones.append(posicion_radianes)
                self.get_logger().info(f'Respuesta del motor en radianes {motor}: {posicion_radianes}')
                if motor == 'A':
                    self.posiciones.append(posicion_radianes)
                    self.get_logger().info(f'2Respuesta del motor en radianes {motor}: {posicion_radianes}')
                else:
                    self.posiciones.append(0.0)  # Si no hay respuesta, asignar 0.0
                    self.get_logger().info(f'Respuesta del motor cuando no toma {motor}')


    def timer_callback(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        # joint_state_msg.name = [motor_id for motor_id in self.motor_ids]  # Nombres de las articulaciones

        # for motor_id in self.motor_ids:
            # if motor_id == 'F':
                # joint_state_msg.name.append('base_tronco')
            # elif motor_id == 'E':
                # joint_state_msg.name.append('hombro_brazo')
            # elif motor_id == 'D':
                # joint_state_msg.name.append('codo_antebrazo')
            # elif motor_id == 'C':
                # joint_state_msg.name.append('muneca')
            # elif motor_id == 'B':
                # joint_state_msg.name.append('engranaje_giro')
            # elif motor_id == 'A':{
                # joint_state_msg.name.append('gripper_izquierdo'),
                # joint_state_msg.name.append('gripper_right')}

        # posiciones = []
        # for motor_id in self.motor_ids:
            # comando = f'PA,{motor_id}\n'.encode('utf-8')
            # self.ser.write(comando)
            # time.sleep(0.0175)
            # if self.ser.in_waiting > 0:
            #     respuesta = self.ser.readline().decode('utf-8').strip()
            #     self.get_logger().info(f'Respuesta del motor {motor_id}: {respuesta}')
            #     try:
            #         posicion = float(respuesta)
            #         min_pos, max_pos, degrees_rot = self.motor_specs[motor_id]
            #         posicion_radianes = self.position_to_radians(posicion, min_pos, max_pos, degrees_rot)
            #     except ValueError:
            #         posicion_radianes = 0.0
            #     posiciones.append(posicion_radianes)
            #     self.get_logger().info(f'Respuesta del motor en radianes {motor_id}: {posicion_radianes}')
            #     if motor_id == 'A':
            #         posiciones.append(posicion_radianes),
            #         self.get_logger().info(f'2Respuesta del motor en radianes {motor_id}: {posicion_radianes}')
            
            # else:
            #     posiciones.append(0.0)  # Si no hay respuesta, asignar 0.0
            #     self.get_logger().info(f'Respuesta del motor cuando no toma {motor_id}')
        joint_state_msg.name = ['base_tronco', 'hombro_brazo', 'codo_antebrazo', 'muneca', 'engranaje_giro', 'gripper_izquierdo', 'gripper_right']
        joint_state_msg.position = self.posiciones
        self.publisher_.publish(joint_state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = NewConrol()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()


