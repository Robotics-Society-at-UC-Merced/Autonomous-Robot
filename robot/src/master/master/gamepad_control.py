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

class GamePadControl_Node(Node):
    def __init__(self):
        super().__init__('GamePadControl_Node')
        '''
        This node is responsible for communication to both ODrive motor controller nodes
        and the joystick node. All three nodes must be running prior to running this node.
        '''

        # ODrive node communication (axis0 // motor #0)
        self.motor_client0 = self.create_client(AxisState, '/odrive_axis0/request_axis_state')
        self.motor_publisher0 = self.create_publisher(ControlMessage, '/odrive_axis0/control_message', 10)
        self.motor_subscription_control_message0 = self.create_subscription(
            ControlMessage, '/odrive_axis0/control_message',
            self.motor_control_message_callback0, 10)

        # ODrive node communication (axis1 // motor #1)
        self.motor_client1 = self.create_client(AxisState, '/odrive_axis1/request_axis_state')
        self.motor_publisher1 = self.create_publisher(ControlMessage, '/odrive_axis1/control_message', 10)
        self.motor_subscription_control_message1 = self.create_subscription(
            ControlMessage, '/odrive_axis1/control_message',
            self.motor_control_message_callback1, 10)

        # Joystick node communication (gamepad)
        self.joy_subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.joy_r_vert, self.joy_r_hori = 0.0, 0.0

    # ROS2 Topic Subscriber callback to ODrive node of axis0
    def motor_control_message_callback0(self, msg):
        #self.get_logger().info(f'Received ControlMessage: {msg.control_mode}, {msg.input_mode}, {msg.input_vel})
        pass
    
    # ROS2 Topic Subscriber callback to ODrive node of-axis1
    def motor_control_message_callback1(self, msg):
        #self.get_logger().info(f'Received ControlMessage: {msg.control_mode}, {msg.input_mode}, {msg.input_vel})
        pass
    
    # ROS2 Service call to ODrive node of axis0 or axis1
    def request_motor_control_state(self, axis_requested_state, axis=0):
        if axis == 0: client = self.motor_client0
        else: client = self.motor_client1

        req = AxisState.Request()
        req.axis_requested_state = axis_requested_state
        self.get_logger().info(f"Requesting axis {axis} state: {axis_requested_state}")
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        while future.result() is None:
            self.get_logger().info('Waiting for service response...')
            time.sleep(0.1)
        if future.result() is not None:
            self.get_logger().info(f'Received response: {future.result()}')
        else:
            self.get_logger().error('Exception while calling service: %r' % future.exception())

    #ROS2 Topic Publisher to ODrive node of axis0 or axis1
    def publish_motor_command(self, control_mode=2, input_mode=1, input_pos=0.0, input_vel=0.0, input_torque=0.0, axis=0):
        if axis == 0: publisher = self.motor_publisher0
        else: publisher = self.motor_publisher1

        msg = ControlMessage()
        msg.control_mode = control_mode
        msg.input_mode = input_mode
        msg.input_pos = input_pos
        msg.input_vel = input_vel
        msg.input_torque = input_torque
        publisher.publish(msg)

    #ROS2 Topic Subscriber callback to joystick node
    def joy_callback(self, msg):
        axes = [float(val) for val in msg.axes]
        buttons = [float(val) for val in msg.buttons]
        self.joy_r_horiz = axes[0]
        self.joy_r_vert = axes[3]
        #                 horizontal verical
        # left Joystick   axes[0]    axes[1]
        # right Joystick  axes[2]    axes[3]

        #self.get_logger().info(f'Received Joy message: axes={axes}, buttons={buttons}')

    #Helper function to get joystick values from callback
    def get_joystick(self):
        return self.joy_r_vert, self.joy_r_horiz


def main(args=None):
    rclpy.init(args=args)

    node = GamePadControl_Node()    

    # odrive nodes are not sending a service request completion notice
    # the program will hang after sending a request
    # manually

    # run them manually with this (crun two times, one with axis0 and with axis1)
    # then run the code normally

    # bug is the service callback wont terminate


    # Send a service request to both motor controllers to enter closed loop control mode
    #print("Sending motor controller0 request")
    #node.request_motor_control_state(axis_requested_state=8, axis=0)
    #print("Sending motor controller1 request")
    #node.request_motor_control_state(axis_requested_state=8, axis=1)
    time.sleep(1)

    # Main loop to control the robot
    while True:
        rclpy.spin_once(node)
        vert, horiz = node.get_joystick()

        max_vel = 1.0   # maximum velocity in m/s
        v_x = horiz * max_vel  # Horizontal input for forward/backward motion
        v_y = vert * max_vel  # Vertical input for turning

        if -max_vel <= v_x <= max_vel:
            motor0_vel = v_x - v_y  # Adjust for turning
            motor1_vel = v_x + v_y  # Adjust for turning

            # Publish velocity commands to motor #0
            node.publish_motor_command(
                control_mode=2, 
                input_mode=1, 
                input_vel=motor0_vel,
                axis=0)

            # Publish velocity commands to motor #1
            node.publish_motor_command(
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