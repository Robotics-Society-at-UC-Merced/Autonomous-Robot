import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from sensor_msgs.msg import Joy

from odrive_can.srv import AxisState
from odrive_can.msg import ControlMessage
from odrive_can.srv import AxisState
from odrive_can.msg import ControlMessage
from rclpy.qos import QoSProfile

class WatchdogFeeder_Node(Node):
    def __init__(self):
        super().__init__('WatchdogFeeder_Node')

        self.test = 'test'


def main(args=None):
    rclpy.init(args=args)
    node = WatchdogFeeder_Node()
    while True:
        node.get_logger().info('Watchdog node started')
        #watchdog.publish()


if __name__ == '__main__':
    main()