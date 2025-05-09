# Autonomous Robot

Robotics Society at UC Merced's project Autonomous Robot. Running ROS2 Humble.

## Getting started

Install the needed ROS2 Humble packages

```
sudo apt-get install ros-humble-joy ros2-humble-joystick-drivers
sudo apt install ros-humble-depthai-ros
```

## System Diagram

This is a sytem diagram of how the robot (currently 4/27/25) functions. The corresponding information pertains to the package `master`. The other two packages `ros_odrive` and `joysick `are community packages.

 ![ROS2 System Diagram](ros2diagram.png)

## Running ROS2 Code

Set up CANBus for the motors (only needed once per computer startup)

```
 sudo ip link set can0 up type can bitrate 250000
 candump can0 -xct z -n 10    #verify can setup correctly
```

Source and build the ROS2 workspace
Navigate to the top level of the ROS2 workspace folder (Autonomous-Robot/robot/) and run

```
 colcon build --packages-select master odrive_can joystick
 source ./install/setup.zsh
```

Now you can run the demo to move the wheels
Open up two different terminals at enter

```
source ./install/setup.zsh
ros2 launch master robot_launch.py  # first terminal
```

```
source ./install/setup.zsh
ros2 run master gamepad_control    # second terminal
```
