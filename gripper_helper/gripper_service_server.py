import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool  
import socket
import time


class Gripper:
    def __init__(self, robot_ip: str, gripper_port: int = 63352):
        self.robot_ip = robot_ip
        self.gripper_port = gripper_port
        self.g = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.g.connect((robot_ip, gripper_port))
            self.g.sendall(b'GET POS\n')
            g_recv = str(self.g.recv(10), 'UTF-8')
            if g_recv:
                self.g.send(b'SET ACT 1\n')
                self.g.recv(10)
                time.sleep(3)
                self.g.send(b'SET GTO 1\n')
                self.g.send(b'SET SPE 255\n')
                self.g.send(b'SET FOR 255\n')
                print('[Gripper] Activated and ready')
        except Exception as e:
            print(f'[Gripper] Connection failed: {e}')

    def control_gripper(self, activate: bool):
        """If activate=True → close, else → open"""
        try:
            pos = 255 if activate else 0
            self.g.send(f"SET POS {pos}\n".encode("UTF-8"))
            time.sleep(0.2)
            return True
        except Exception as e:
            print(f'[Gripper] Control error: {e}')
            return False

    def setup_gripper(self):
        """Optional re-initialization"""
        try:
            self.g.send(b'SET ACT 1\n')
            self.g.recv(10)
            time.sleep(3)
            self.g.send(b'SET GTO 1\n')
            print('[Gripper] Re-setup complete')
            return True
        except Exception as e:
            print(f'[Gripper] Setup error: {e}')
            return False


class GripperService(Node):
    def __init__(self):
        super().__init__('robotiq_gripper_service')
        self.declare_parameter('robot_ip', '10.10.0.60')  # Example IP
        robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value

        self.gripper = Gripper(robot_ip)

        # Create a service for controlling the gripper
        self.srv = self.create_service(SetBool, 'control_gripper', self.control_callback)
        self.get_logger().info('Robotiq Gripper service ready. Use True=close, False=open.')

    def control_callback(self, request, response):
        """
        request.data = True → close gripper
        request.data = False → open gripper
        """
        success = self.gripper.control_gripper(request.data)
        response.success = success
        response.message = 'Gripper command executed' if success else 'Gripper command failed'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GripperService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down gripper service...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
