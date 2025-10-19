#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RoverControlPublisher(Node):
    def __init__(self):
        super().__init__('rover_control_publisher')
        self.publisher_ = self.create_publisher(String, '/rover_control', 10)
        self.get_logger().info('🚀 Rover Control Node Started!')

        # Interactive command loop
        self.timer = self.create_timer(0.5, self.publish_command)
        self.cmd = ""

    def publish_command(self):
        if self.cmd != "":
            msg = String()
            msg.data = self.cmd
            self.publisher_.publish(msg)
            self.get_logger().info(f"📡 Sent command: {msg.data}")
            self.cmd = ""  # clear after sending

def main(args=None):
    rclpy.init(args=args)
    node = RoverControlPublisher()

    try:
        while rclpy.ok():
            print("\nEnter a command:")
            print("➡️ forward | ⬅️ backward | ⬆️ left | ➡️ right | 🔄 rotate | 🛑 stop")
            cmd = input(">>> ").strip().lower()
            if cmd in ["forward", "backward", "left", "right", "rotate", "stop"]:
                node.cmd = cmd
            else:
                print("⚠️ Invalid command.")
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
