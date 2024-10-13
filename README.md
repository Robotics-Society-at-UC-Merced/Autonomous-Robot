# Auronomous Robot
Robotics Society at UC Merced's project Autonomous Robot. Running ROS2 Humble. 

## Getting started
```
sudo apt-get install ros-humble-joy ros2-humble-joystick-drivers
sudo apt install ros-humble-depthai-ros
```

### robot
The ROS2 workspace pertaining to the overall structure of the robot is contained

## Running 
Set up CANBus for the motors
 ```
 sudo ip link set can0 up type can bitrate 250000
 candump can0 -xct z -n 10    #verify can setup correctly
 ```
Source and build the ROS2 workspace
 ```
 source ./install/setup.zsh
 cd Autonomous-Robot/robot/
 colcon build --packages-select odrive_can master joystick 
 ```
 Run the demo
```
...
```