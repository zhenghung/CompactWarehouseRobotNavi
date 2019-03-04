# Compact Warehouse Robot files for ROS/Gazebo (WIP)
Model contains a chassis, two caster wheels, differential drive, lidar.

Required packages:

* joint_state_publisher
* laser_filters
* TurtleBot3 (for keyboard teleop)

## Normal usage (basic spawn)
Copy `warebot` folder to your catkin workspace. Navigate to your catkin workspace then:

    catkin_make

    source /home/<USER_NAME>/catkin_ws/devel/setup.bash

To spawn robot in an empty world in Gazebo:

    roslaunch warebot_gazebo empty.launch

Optional: Several other worlds are also included. Modify .launch parameter accordingly.

* basic_maze
* playground

To view robot in rviz:

    roslaunch warebot_description rviz.launch

  Optional: Several other rviz setups are also icluded.

* rviz_amcl
* rviz_mapping

## Differential drive movement
We can now move the robot via cmd_vel.
Open a new terminal

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

    roslaunch warebot_navigation keyboard.launch

Use WSADX keys to move robot.

## Filtering the LIDAR
Due to the placement of the LIDAR, LaserScan registers hits on the robot itself, which complicates mapping. We filter this out using a `laser_filters` node.

    roslaunch warebot_description filter.launch

## Creating a map
Mapping is done via gmapping. We create a map so that the robot can use it for navigation.

Launch gazebo and rviz files.
Launch `filter.launch` file.

Launch `mapping.launch` file.

    roslaunch warebot_navigation mapping.launch

Launch teleop of choice to move robot to generate a map.
Save map in `warebot_navigation/maps` to save a .pgm and .yaml file.

    rosrun map_server map_saver -h <DIRECTORY>

## Navigation (WIP)
Launch gazebo and rviz files.
Launch `filter.launch` file.

Launch `navigation.launch` file.

    roslaunch warebot_navigation navigation.launch

Use Rviz to set waypoints for navigation or the provided Move Base Action GUI package!

## Setting waypoints via GUI
Implemented using Tkinter and the ROS Action Library via Python.

Run the `warebot_goal` package.

    rosrun warebot_goal move_client.py

Reminder that move_base has a finite length (3m) for goals! Needs to be fixed, but when close enough, works perfectly.
