Name: Jayden Cheng

This is a controller that uses the keyboard as an interface.

Controls:
w key: move forward
s key: move backward
a key: rotate (positive direction)
d key: rotate (negative direction)
q key: reset angular and linear velocity

0, 1, 2, 3 keys: toggle acceleration type

Acceleration types:
0: Zero acceleration (except changing directions)
1: Constant (nonzero) acceleration
2: Linear Acceleration
3: Quadratic Acceleration

How to build:
source ~/.bashrc
colcon build --packages-select keyboard_controller

launching the nodes:
go to path: keyboard_controller/robot_build/launch
ros2 launch keyboard_launch.py
