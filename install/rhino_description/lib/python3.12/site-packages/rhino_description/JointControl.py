import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointControl(Node):
    def __init__(self):
        super().__init__('joint_control')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.joint_positions = {
            'base_tronco': 0.0,
            'codo_de_brazo': 0.0,
            'hombro_brazo': 0.0,
            'codo_antebrazo': 0.0,
            'antebrazo_muneca': 0.0,
            'gripper_izquierdo': 0.0,
            'gripper_right': 0.0
        }
        self.get_logger().info("JointControl node has been started.")

    def set_joint_position(self, joint_name, position):
        if joint_name in self.joint_positions:
            self.joint_positions[joint_name] = position
            self.publish_joint_states()
        else:
            self.get_logger().error(f'Joint {joint_name} erroneo.')

    def publish_joint_states(self):
        joint_state = JointState()
        joint_state.name = list(self.joint_positions.keys())
        joint_state.position = list(self.joint_positions.values())
        joint_state.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(joint_state)
        self.get_logger().info(f'Published joint states: {self.joint_positions}')

def main(args=None):
    rclpy.init(args=args)
    node = JointControl()

    try:
        while rclpy.ok():
            print("Select a joint to move: ")
            print("1: base_tronco. (-1.57 <= position <= 1.570)\n")
            print("2: hombro_brazo. (-0.9 <= position <= 1.570)\n")
            print("3: codo_antebrazo. (1.7 <= position <= 2.5)\n")
            print("4: antebrazo_muneca, -0.9 <= position <= 1.570\n")
            print("5: gancho. (-0.38 <= position <= 0.0)\n")

            choice = input("Enter the number of the joint (or 'q' to quit): ").strip()
            if choice.lower() == 'q':
                break

            if choice == '1':
                joint_name = 'base_tronco'
            elif choice == '2':
                joint_name = 'hombro_brazo'
            elif choice == '3':
                joint_name = 'codo_antebrazo'
            elif choice == '4':
                joint_name = 'antebrazo_muneca'
            elif choice == '6':
                joint_name = 'codo_de_brazo'
            elif choice == '5':
                joint_name = 'gripper_izquierdo'
                joint_name_right = 'gripper_right'
            else:
                print("Invalid choice, please select a valid joint number.\n")
                continue

            position = input(f"\nIngresar la posición para {joint_name} (o 'q' para salir): ").strip()

            if position.lower() == 'q':
                break

            try:
                position = float(position)

                if choice == '1' and -1.57 <= position <= 1.570:
                    node.set_joint_position(joint_name, position)
                elif choice == '2' and -0.9 <= position <= 1.570:
                    node.set_joint_position(joint_name, position)
                elif choice == '3' and 1.7 <= position <= 2.5:
                    node.set_joint_position(joint_name, position)
                    node.set_joint_position(joint_name, position)
                elif choice == '4' and -0.9 <= position <= 1.570:
                    node.set_joint_position(joint_name, position)
                elif choice == '5' and -0.38 <= position <= 0.0:
                    node.set_joint_position(joint_name, position)
                    node.set_joint_position(joint_name_right, position)
                else:
                    print("\nPosicion invalida, ingresa una posicion valida para el rango.\n")
            except ValueError:
                print("Invalid input, please enter a valid float value.\n")
    finally:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
