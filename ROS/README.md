# Compact Warehouse Robot files for ROS/Gazebo
Model contains a chassis, two caster wheels, differential drive, lidar.

Requirements:

* joint_state_publisher

## How to use
Copy `warebot` folder to your catkin workspace. Navigate to your catkin workspace then:
    
    catkin_make

    source /home/<USER_NAME>/catkin_ws/devel/setup.bash

To launch in Gazebo:

    roslaunch warebot gazebo.launch
To launch in rviz

    roslaunch warebot display.launch

## Differential drive movement
We can now move the robot via cmd_vel

    rostopic pub /cmd_vel geometry_msgs/Twist "linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0"

Change values to test differential drive.

## Keyboard movement via TurtleBot3_teleop
(Requires TurtleBot3 package installed)

    roslaunch warebot keyboard.launch

TODO:

 - XACRO 
 - Implement SLAM
