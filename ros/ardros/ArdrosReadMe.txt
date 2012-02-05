roscd ardros
roslaunch ./launch/ardros_standalone.launch

Note: Without using a launch file it could be started this way:
rosrun ardros arduino.py

roslaunch ardros teleop.launch
roslaunch ardros teleopRobot.launch

For teleop install qtsixa from http://qtsixa.sourceforge.net/:
sudo add-apt-repository ppa:falk-t-j/qtsixa
sudo apt-get update
sudo apt-get install qtsixa

Then: http://ubuntuforums.org/showpost.php?p=10360389&postcount=789


roscd ardros
rosrun ardros DeadReckoning.py


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
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist -- '{ linear: { x: 0.02, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0}  }'

-1 means: just one message is published



Also consider showing rxplot
rxplot /odom/pose/pose/position/x  /odom/pose/pose/position/y

Showing an invoking service
rosservice list
rosservice call /setDriveControlGains 0.1 0.1 0.1 0.1



rosparam list


ls /dev/input/
sudo jstest /dev/input/jsX

Axes:  0:     0  1:     0  2:     0  3:     0  4:  8340  5: -1192  6:-32767  7:     0  8:     0  9:     0 10:     0 11:     0 12:     0 13:     0 14:     0 15:     0 16:     0 17:     0 18:     0 19:     0 20:     0 21:     0 22:     0 23:     0 24:     0 25:     0 26:     0 27:     0 28:     0 Buttons:  0:off  1:off  2:off  3:off  4:off  5:off  6:off  7:off  8:off  9:off 10:off 11:off 12:off 13:off 14:off 15:off 16:off 

rosparam set joy_node/dev "/dev/input/js1"
rosrun joy joy_node

New terminal:
rostopic echo joy


Logging:
rxloggerlevel &
rxconsole &


Kinect Only:

Part 1:
roslaunch openni_camera openni_node.launch

Part 2:
roscd ardros
rosrun rviz rviz -d ./rviz/kinect.vcg




SLAM:
Part 1:
roscd ardros
roslaunch ./launch/slam.launch

Part 2: Launch rviz
roscd ardros
rosrun rviz rviz -d ./rviz/slam.vcg

Part 3:
After driving around for a while, save the generated map:

roscd ardros/maps
rosrun map_server map_saver -f ./map



Navigation:
Part 1:

roslaunch `rospack find ardros`/launch/navigation.launch

Note: You might need to change the path to the map in move_base.launch

Part 2: Launch rviz
rosrun rviz rviz -d `rospack find ardros`/rviz/navigation.vcg

Set current pose and goal. Optionally monitor status:
rostopic echo /move_base/status

[Previously was:
roscd ardros
roslaunch ./launch/ardros_configuration.launch
roslaunch ./launch/move_base.launch
]


Increase frequency of laser scan publishing:
/home/rainer/ni/ni/pointcloud_to_laserscan/launch/kinect_laser.launch

    <param name="max_rate" value="10"/>

Originally was 2


Listen to goals:
rostopic echo /move_base/goal
save to file: rostopic echo /move_base/goal > /home/rainer/dev/drh-robotics-ros/ros/ardros/goals.txt


Navigate through a list of goals
roscd ardros
rosrun ardros GoalsSequencer.py ./maps/simpleGoals.txt
or
rosrun ardros GoalsSequencer.py ./maps/recordedGoals.txt


