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
            timeout=0.04
        )
        self.motor_ids = ['B', 'C', 'D', 'E', 'F']  # Identificadores de los motores

    def timer_callback(self):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [motor_id for motor_id in self.motor_ids]  # Nombres de las articulaciones

        # Leer cada motor y almacenar la posición
        posiciones = []
        for motor_id in self.motor_ids:
            comando = f'PA,{motor_id}\n'.encode('utf-8')
            self.ser.write(comando)
            # time.sleep(0.1)
            # if self.ser.in_waiting > 0:
            respuesta = self.ser.readline().decode('utf-8').strip()
                #respuesta = int(respuesta)
                #if motor_id == 'E':
               # 	respuesta = respuesta - 1381
                #elif motor_id == 'D':
                #	respuesta = respuesta - 772
                #elif motor_id == 'C':
               # 	respuesta = respuesta -2022
            self.get_logger().info(f'Respuesta del motor {motor_id}: {respuesta}')
            try:
        
                posicion = float(respuesta)
                    # posicion = 0.0  # Manejar errores en la conversión
                posiciones.append(posicion)
            except ValueError:
            # else:
                posiciones.append(0.0)  # Si no hay respuesta, asignar 0.0

        joint_state_msg.position = posiciones
        self.publisher_.publish(joint_state_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SerialToJointState()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
