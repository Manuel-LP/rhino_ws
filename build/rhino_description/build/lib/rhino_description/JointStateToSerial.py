import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import time

class JointStateToSerial(Node):
    def __init__(self):
        super().__init__('joint_state_to_serial')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
# Abre el puerto serial con la configuración deseada
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10
        )

        self.ser = serial.Serial(
            port='/dev/ttyUSB0',  # Cambia esto al puerto adecuado para tu sistema
            baudrate=9600,         # La velocidad en baudios, asegúrate de que coincida con la del dispositivo
            bytesize=serial.SEVENBITS,    # 7 bits de datos
            parity=serial.PARITY_ODD,     # Paridad impar
            stopbits=serial.STOPBITS_TWO, # 2 bits de parada
            timeout=0.1                   # Tiempo de espera para operaciones de lectura
        )

    def generar_comando(self, motor, rango):
    # Generar el comando PD
        comando_pd = f"PD,{motor},{rango*1000}\n".encode()  
        return comando_pd
    comando=[]
    def listener_callback(self,msg: JointState):
        for nombre, valor in zip(msg.name, msg.position):
            comando = self.generar_comando(nombre, valor)
            self.ser.write(comando)
            self.ser.write(b'MI\n')
            self.get_logger().info(f'Respuesta del motor {nombre}: {valor*1000}')

            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateToSerial()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()