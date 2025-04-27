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
        self.placeholder = 'placeholder'

        # ODrive node communication (axis0 // motor #0)
        self.motor_publisher0 = self.create_publisher(ControlMessage, '/odrive_axis0/control_message', 10)
        self.motor_subscription_control_message0 = self.create_subscription(
            ControlMessage, '/odrive_axis0/control_message',
            self.motor_control_message_callback0, 10)

        # ODrive node communication (axis1 // motor #1)
        self.motor_publisher1 = self.create_publisher(ControlMessage, '/odrive_axis1/control_message', 10)
        self.motor_subscription_control_message1 = self.create_subscription(
            ControlMessage, '/odrive_axis1/control_message',
            self.motor_control_message_callback1, 10)


    # ROS2 Topic Subscriber callback to ODrive node of axis0
    def motor_control_message_callback0(self, msg):
        #self.get_logger().info(f'Received ControlMessage: {msg.control_mode}, {msg.input_mode}, {msg.input_vel})
        pass
    
    # ROS2 Topic Subscriber callback to ODrive node of-axis1
    def motor_control_message_callback1(self, msg):
        #self.get_logger().info(f'Received ControlMessage: {msg.control_mode}, {msg.input_mode}, {msg.input_vel})
        pass


    #ROS2 Topic Publisher to ODrive node of axis0 or axis1
    def publish_watchdog_feed(self, control_mode=2, input_mode=1, input_pos=0.0, input_vel=0.0, input_torque=0.0, axis=0):
        if axis == 0: publisher = self.motor_publisher0
        else: publisher = self.motor_publisher1

        msg = ControlMessage()
        #msg.control_mode = control_mode
        #msg.input_mode = input_mode
        #msg.input_pos = input_pos
        #msg.input_vel = input_vel
        #msg.input_torque = input_torque
        publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WatchdogFeeder_Node()

    while True:
        rclpy.spin_once(node)
        node.get_logger().info('Watchdog node started')
        node.publish_watchdog_feed()


if __name__ == '__main__':
    main()