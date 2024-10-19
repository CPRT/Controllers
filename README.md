# Controllers

## IMU Controller

 Adam's Inertial Measurement Unit (IMU) controller uses a BCI160 connected to a Raspberry Pi to provide a sensor that can detect motion in the *x*, *y*, and *z* directions.

IMU data is read from the i2c bus by node *imu_node.py* using a python library  and then published as a sensor_msgs/Imu.msg to /imu_msg/raw. 

The node *imu_twist.py* subscribes to /imu_msg/raw and converts the data into a geometry_msgs/Twist.msg on /cmd_vel.

### Equipment
- Raspberry Pi 4 4GB
- RPI4 case
- BCI160 module from DigiKey
- Monitor/Keyboard/Mouse/Power-supply

### Setup
-   Install Ubuntu 22.04 Desktop with the RPi imager
	- user *adam* added
-   Followed instructions to install ros2 humble
	-   https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
-   Setup for the BMI160 module following the setup in this document
	-   https://robofoundry.medium.com/using-bmi160-imu-with-ros2-ecb550851efa
-   add user to the i2c and dialout groups (the file /dev/i2c-1 is readable by people in ‘dialout’ group)
	```$ sudo adduser adam i2c```
	```$ sudo adduser adam dialout```
-   ensure dtparam=i2c_arm=on is uncommented in /boot/firmware/config.txt (it was)
```sudo apt install i2c-tools python3-pip```
```sudo pip3 install BMI160-i2c smbus2```
-   tested hardware with sudo i2cdetect -r -y 1
-   the device was in the output as 0x69
-   downloaded (cloned) software that uses the module with ros2
	```$ git clone https://github.com/robofoundry/aws-deepracer-imu-pkg.git```
	```$ cd aws-deepracer-imu-pkg```
	```./imu_pkg/scripts/load_ros2_imu_tools_repo.sh```
-   from the imu_pkg in this project we have taken the imu_node.py and the associated constants.py files to use in our controller to generate the raw IMU topic messages
-   Watched videos
-   Tried to setup visual studio on the Rpi but and ARM version is not available.

### Create package
-   Clone the Controller repository and create branch
	```$ git clone git@github.com:CPRT/Controllers.git Controllers```
	```$ cd Controllers```
   ```$ git checkout -b Adam-A```
   ```$ mkdir adam_ws```
	```$ cd adam_ws```
	```$ mkdir src```
	```$ cd src```

-   Create a package **AA_imu** with a node *imu_twist.py*. Another node, *imu_node.py* will be copied in after.
	```$ ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name imu_twist AA_imu```
	-   In **AA_imu** copy in the *imu_node.py* and *constants.py* from the aws-deepriver-imu_pkg above
	-   Update the *setup.py* file to reference our two nodes:
```
			'console_scripts': [
			'imu_node = AA_imu.imu_node:main',
			'imu_twist = AA_imu.imu_twist:main',
			],
```

-   added dependencies in *package.xml* from the nodes:
```
			<depend>rclpy</depend>
			<depend>sensor_msgs</depend>
			<depend>geometry_msgs</depend>
			<depend>std_msgs</depend>
```

-   took launch file from aws-deepriver-imu_pkg and added imu_twist node and placed in ‘launch’ dir
-   took the config YAML file from aws-deepriver-imu_pkg and added a section for the imu_twist node with config:
```
	imu_node:
		ros__parameters:
			~device_address: 105
			~bus_id: 1
			~imu_frame: 'imu_link'
			~imu_pub_topic: '/imu_msg/raw'
			~publish_rate: 60

	imu_twist:
		ros__parameters:
			~imu_pub_topic: '/imu_msg/raw'
			~twist_pub_topic: '/cmd_vel'
			~publish_rate: 20
			~max_speed: 10
			~max_angular_speed: 10
			~jitter: 0.1
```
  
-   Updated setup.py to add sections to put the yaml and launch files in the install directory during build
```
(os.path.join('share', package_name, 'launch'),
glob('launch/*.launch.py')),
(os.path.join('share', package_name, 'config'),
glob('config/*.yaml')),
```

-   IMU twist node

This node takes the messages from the /imu_msg/raw topic and publishes messages on the /cmd_vel topic

-   need to convert the Imu message data to Twist data
-   currently, when a raw message is received it is stored
-   when the publish timer goes the last raw message is converted and published.
-   probably much work could be done in here.
-   currently just ensures absolute value of the x/y/z values are less than the defined max values and greater than the jitter
-   if value less than jitter then set to 0.0
-   if value > max then set to max

### To Build the package
```
$ cd Controllers/adam_ws/src
$ colcon build
```


### To Run

The easiest way to run is using the launch file
```
$ cd Controllers/adam_ws/src
$ ros2 launch AA_imu AA_imu.launch.py
```

Individual nodes can be started like this:
```
$ ros2 run AA_imu imu_node
$ ros2 run AA_imu imu_twist
```
Note: the yaml file is not used unless specified like:
```
$ ros2 run AA_imu imu_node --ros-args --params-file ./AA_imu/config/imu_parameters.yaml
```

### Turtle Power
The Twist messages from the controller can control the turtlesim
```$ sudo apt install ros-humble-turtlesim ```

Run the imu controller and the turtlesim. The imu controller publishes on /cmd_vel so remap turtlesim to subscribe to that  
```
$ cd ~/Controllers/adam_ws/src
$  ros2 run turtlesim turtlesim_node --ros-args --remap turtle1/cmd_vel:=/cmd_vel
$ ros2 launch AA_imu AA_imu.launch.py
```
