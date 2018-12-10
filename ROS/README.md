# Compact Warehouse Robot files for ROS/Gazebo

Model contains a chassis, two caster wheels, differential drive.

Terminal:

    catkin_make

    source /home/<USER_NAME>/catkin_ws/devel/setup.bash

    roslaunch warebot_gazebo main.launch


New terminal:

    rostopic pub /cmd_vel geometry_msgs/Twist "linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0"

Change values to test differential drive.

TODO:

 - Improve physics to better model actual robot

