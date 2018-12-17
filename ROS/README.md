# Compact Warehouse Robot files for ROS/Gazebo

Model contains a chassis, two caster wheels, differential drive.

Terminal:

    cd catkin_ws
    
    catkin_make

    source /home/<USER_NAME>/catkin_ws/devel/setup.bash

    roslaunch warebot_gazebo main.launch

We can now move the robot IRT.

Control Method #1: cmd_vel:

    rostopic pub /cmd_vel geometry_msgs/Twist "linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0"

Note: Change values to test differential drive.

Control Method #2: TurtleBot3_teleop
(Requires TurtleBot3 packages)

    roslaunch warebot_gazebo keyboard_teleop.launch

TODO:

 - XACRO 
 - rviz visualisation
 -change plugins used

