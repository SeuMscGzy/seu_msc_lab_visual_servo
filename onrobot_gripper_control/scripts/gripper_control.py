#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
import time
import argparse
from onrobot import RG

class GripperControlNode(Node):
    def __init__(self, gripper, ip, port):
        super().__init__('gripper_control_node')
        self.subscription = self.create_subscription(
            Int64,
            'gripper_control',
            self.gripper_control_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.rg = RG(gripper, ip, port)
        self.get_logger().info('init is ok')
    
    def gripper_control_callback(self, msg):
        if not self.rg.get_status()[0]:  # not busy
            if msg.data == 1:
                self.get_logger().info('Closing gripper')
                self.rg.close_gripper()
            elif msg.data == 0:
                self.get_logger().info('Opening gripper')
                self.rg.open_gripper()
            else:
                self.get_logger().info('Invalid command received')

            while True:
                time.sleep(0.5)
                if not self.rg.get_status()[0]:
                    break
        else:
            self.get_logger().info('Gripper is busy')

    def close_connection(self):
        self.rg.close_connection()

def get_options():
    parser = argparse.ArgumentParser(description='Set options for gripper control.')
    parser.add_argument('--gripper', dest='gripper', type=str, default="rg2", choices=['rg2', 'rg6'], help='set gripper type, rg2 or rg6')
    parser.add_argument('--ip', dest='ip', type=str, default="192.168.1.1", help='set ip address')
    parser.add_argument('--port', dest='port', type=str, default="502", help='set port number')
    return parser.parse_args()

def main(args=None):
    rclpy.init(args=args)
    options = get_options()
    gripper_control_node = GripperControlNode(options.gripper, options.ip, options.port)
    
    try:
        rclpy.spin(gripper_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        gripper_control_node.close_connection()
        gripper_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':

    main()