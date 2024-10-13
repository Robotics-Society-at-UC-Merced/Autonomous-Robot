# Auronomous Robot
Robotics Society at UC Merced's project Autonomous Robot. Running ROS2 Humble. 

## Getting started
```
sudo apt-get install ros-humble-joy ros2-humble-joystick-drivers
sudo apt install ros-humble-depthai-ros
```

## Running ROS2 Code
Set up CANBus for the motors
 ```
 sudo ip link set can0 up type can bitrate 250000
 candump can0 -xct z -n 10    #verify can setup correctly
 ```
Source and build the ROS2 workspace
 ```
 source ./install/setup.zsh
 cd Autonomous-Robot/robot/
 colcon build --packages-select master odrive_can joystick 
 ```
 Run the demo
```
...
```
