In Terminal 1
roscore

In Terminal 2
roscd ardros
roslaunch ardros default.launch

Note: Without using a launch file it could be started this way:
rosrun ardros arduino.py

roslaunch ardros teleop.launch
roslaunch ardros teleopRobot.launch


In Terminal 3:
rosrun rviz rviz

add grid add tf

In a new terminal:
rostopic echo /odom
or
rostopic echo /serial

or to get the frequency of messages being sent:

rostopic hz /odom

In a new terminal:
rxgraph



Manually publish speed command
In a new terminal:
rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '{ linear: { x: 0.02, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0}  }'

-1 means: just one message is published



Also consider showing rxplot
rxplot /odom/pose/pose/position/x  /odom/pose/pose/position/y

Showing an invoking service
rosservice list
rosservice call /setDriveControlGains 1 0 0



rosparam list

Setup joystick

sudo bash
rosrun ps3joy ps3joy.py
rosrun joy joy_node

