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
        '''
        This node is responsible for communication to both ODrive motor controller nodes
        and the joystick node. All three nodes must be running prior to running this node.
        '''

        # ODrive node communication (axis0)
        self.client0 = self.create_client(AxisState, '/odrive_axis0/request_axis_state')
        self.publisher0 = self.create_publisher(ControlMessage, '/odrive_axis0/control_message', 10)
        self.subscription_control_message0 = self.create_subscription(
            ControlMessage, '/odrive_axis0/control_message',
            self.control_message_callback0, 10)

        # ODrive node communication (axis1)
        self.client1 = self.create_client(AxisState, '/odrive_axis1/request_axis_state')
        self.publisher1 = self.create_publisher(ControlMessage, '/odrive_axis1/control_message', 10)
        self.subscription_control_message1 = self.create_subscription(
            ControlMessage, '/odrive_axis1/control_message',
            self.control_message_callback1, 10)

        # Joystick node communication
        self.joy_subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.joy_r_vert, self.joy_r_hori = 0.0, 0.0

    def control_message_callback0(self, msg):
        #self.get_logger().info(f'Received ControlMessage: {msg.control_mode}, {msg.input_mode}, {msg.input_vel})
        pass
    def control_message_callback1(self, msg):
        #self.get_logger().info(f'Received ControlMessage: {msg.control_mode}, {msg.input_mode}, {msg.input_vel})
        pass

    def send_request(self, axis_requested_state, axis=0):
        if axis == 0: client = self.client0
        else: client = self.client1

        req = AxisState.Request()
        req.axis_requested_state = axis_requested_state
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Received response: {future.result()}')
        else:
            self.get_logger().error('Exception while calling service: %r' % future.exception())

    def publish_message(self, control_mode=2, input_mode=1, input_pos=0.0, input_vel=0.0, input_torque=0.0, axis=0):
        if axis == 0: publisher = self.publisher0
        else: publisher = self.publisher1

        msg = ControlMessage()
        msg.control_mode = control_mode
        msg.input_mode = input_mode
        msg.input_pos = input_pos
        msg.input_vel = input_vel
        msg.input_torque = input_torque
        publisher.publish(msg)

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

    # Send request to both motor controllers to enter closed loop control mode
    print("Sending motor controller0 request")
    controller.send_request(axis_requested_state=8, axis=0)
    print("Sending motor controller1 request")
    controller.send_request(axis_requested_state=8, axis=1)
    time.sleep(1)

    # Main loop to control the robot
    while True:
        rclpy.spin_once(controller)
        vert, horiz = controller.get_joystick()

        max_vel = 2.0
        v_x = horiz * max_vel  # Horizontal input for forward/backward motion
        v_y = vert * max_vel  # Vertical input for turning

        if -max_vel <= v_x <= max_vel:
            motor0_vel = v_x - v_y  # Adjust for turning
            motor1_vel = v_x + v_y  # Adjust for turning

            controller.publish_message(
                control_mode=2, 
                input_mode=1, 
                input_vel=motor0_vel,
                axis=0)

            controller.publish_message(
                control_mode=2, 
                input_mode=1, 
                input_vel=motor1_vel,
                axis=1)

            #print(f"Veritcal: {vert:.2f}\tHorizontal: {horiz:.2f}")
            print(f"Motor0: {motor0_vel:.2f} m/s\tMotor1: {motor1_vel:.2f} m/s")
        else:
            print("Velocity out of range")


if __name__ == '__main__':
    main()