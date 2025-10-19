import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket
import json

class TCPServer(Node):
    def __init__(self):
        super().__init__('tcp_server')
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.callback, 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind(('0.0.0.0', 9090))
        self.sock.listen(1)
        self.get_logger().info('Waiting for Unity to connect...')
        self.conn, _ = self.sock.accept()
        self.get_logger().info('Unity connected via TCP!')

    def callback(self, msg):
        data = {
            'linear_x': msg.linear.x,
            'angular_z': msg.angular.z
        }
        self.conn.sendall((json.dumps(data) + "\n").encode())

def main():
    rclpy.init()
    node = TCPServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
