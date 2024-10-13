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

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.client = self.create_client(AxisState, '/odrive_axis0/request_axis_state')
        self.publisher = self.create_publisher(ControlMessage, '/odrive_axis0/control_message', 10)
        self.subscription_control_message = self.create_subscription(
            ControlMessage,
            '/odrive_axis0/control_message',
            self.control_message_callback,
            10
        )
        self.joy_subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.joy_r_vert = 0.0
        self.joy_r_horiz = 0.0

    def control_message_callback(self, msg):
        #self.get_logger().info(f'Received ControlMessage: {msg.control_mode}, {msg.input_mode}, {msg.input_vel})
        pass

    def send_request(self, axis_requested_state):
        req = AxisState.Request()
        req.axis_requested_state = axis_requested_state
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Received response: {future.result()}')
        else:
            self.get_logger().error('Exception while calling service: %r' % future.exception())

    def publish_message(self, control_mode, input_mode, input_pos, input_vel, input_torque):
        msg = ControlMessage()
        msg.control_mode = control_mode
        msg.input_mode = input_mode
        msg.input_pos = input_pos
        msg.input_vel = input_vel
        msg.input_torque = input_torque
        self.publisher.publish(msg)

    def joy_callback(self, msg):
        axes = [float(val) for val in msg.axes]
        buttons = [float(val) for val in msg.buttons]

        self.joy_r_horiz = axes[2]
        self.joy_r_vert = axes[3]

        #self.get_logger().info(f'Received Joy message: axes={axes}, buttons={buttons}')

    def get_joystick(self):
        return self.joy_r_vert, self.joy_r_horiz


def main(args=None):
    rclpy.init(args=args)

    controller = MyNode()

    print("Sending motor controller request")
    controller.send_request(axis_requested_state=8)
    print("Motor controller request accepted")

    while True:
        rclpy.spin_once(controller)
        vert, horiz = controller.get_joystick()
        #print(f"Vertical: {vert}, Horizontal: {horiz}")
        
        max_vel = 2.0
        velocity = abs(vert * max_vel)

        if velocity >= 0.0 or velocity <= 2.0:
            controller.publish_message(
                control_mode=2, 
                input_mode=1, 
                input_pos=0.0, 
                input_vel=velocity, 
                input_torque=0.0)
            print(f"Motor0: {velocity} m/s")
        else:
            print("Velocity out of range")



if __name__ == '__main__':
    main()