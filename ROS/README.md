# Compact Warehouse Robot files for ROS
Model contains a chassis, two caster wheels, differential drive, LIDAR. 
When used for the real robot, differential drive is removed (LIDAR not yet removed).

Required packages:

* joint_state_publisher
* laser_filters
* TurtleBot3 (for keyboard teleop)

## Setup

Copy `warebot` folder to your catkin workspace. Navigate to your catkin workspace then:

    catkin_make

    source /home/<USER_NAME>/catkin_ws/devel/setup.bash

# Using the Simulated Robot

## Basic spawn (in an empty world)

To spawn robot in an empty world in Gazebo:

    roslaunch warebot_gazebo empty.launch

Optional: Several other worlds are also included. Modify `.launch` parameter accordingly.
Note that some worlds require you to have the proper models in Gazebo. Many can be downloaded at http://data.nvision2.eecs.yorku.ca/3DGEMS/

* basic_maze (A very basic maze with solid walls)
* playground (The turtlebot playground world)
* walled (An empty walled room)
* warehouse (A simulated warehouse using actual shelves. Untested!)
* warehouse_wall (The simulated warehouse but using solid walls for the shelves. This is used throughout testing)

To view the simulated robot in rviz:

    roslaunch warebot_description rviz_simgui.launch

The simulated robot will appear in rviz as well as a GUI to control the two differential drive wheels.

You can also run the simulated robot without the controller GUI by running:

    roslaunch warebot_description rviz_sim.launch
   

## Differential drive movement
We can now move the robot via `cmd_vel`.

Open a new terminal:

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

In a new terminal:

    roslaunch warebot_navigation keyboard.launch

Use WSADX keys to move robot.

# Using the Actual Robot
It is assumed that a real LIDAR and differential drive is already in place.
LIDAR needs to publish to `/scan` and differential drive to `/cmd_vel`.

# After initial steps

## Using Rviz

To view the  robot in rviz:

    roslaunch warebot_description rviz.launch

Note that this uses a different URDF model (`real.urdf`) compared to the simulated robot that has the `lidar_generic` frame changed to a blank `laser` frame which is typically needed for a real Lidar as well as removal of the differential drive frames.

## Filtering the LIDAR
Due to the placement of the LIDAR, LaserScan registers hits on the simulated robot itself, which complicates mapping. We filter this out using a `laser_filters` node.

    roslaunch warebot_description filter.launch

This node has to be launched which will create a `/laser_filtered` topic.
To view in rviz, change LaserScan topic accordingly.

## Mapping or Creating a map
Mapping is done via gmapping. We create a map so that the robot can use it for navigation.

Note that the laser filter node has to be run first (/laser_filtered topic is referred)! This is now done automatically through the launch file itself!

Launch gazebo and rviz files.
Launch `filter.launch` file.

Launch `mapping.launch` file:

    roslaunch warebot_navigation mapping.launch

Launch teleop of choice to move robot to generate a map.
Save map in `warebot_navigation/maps` to save a .pgm and .yaml file.

    rosrun map_server map_saver -h <DIRECTORY>

## Navigation
This allows the robot to autonomously navigate to a waypoint using the navigation stack.

Again, note that the laser has to be filtered beforehand to prevent LaserScan hits on the robot itself from creating an obstacle within the robot itself. This is automatically run when you launch the navigation launch file.

Launch gazebo and rviz files.

Launch `navigation.launch` file.

    roslaunch warebot_navigation navigation.launch

Use Rviz to set waypoints for navigation or the provided Move Base Action GUI package!


## Using the Laser Scan Matcher
This uses the `laser_Scan_matcher` package to improve odometry. Use only when a real LIDAR is used!

To run:

    roslaunch warebot_lidar lsm.launch

## Setting waypoints via a GUI
Implemented using Tkinter and the ROS Action Library via Python.

Run the `warebot_goal` package:

    rosrun warebot_goal move_client.py

A new window containing the `warehouse_walled` map will appear. Click on any point in the map to set navigation goals for the robot.

Currently no feedback is implemented as this is causing holdups. May be changed in the future.

Note: Reminder that move_base has a finite length (3m) for goals! Needs to be fixed, but when close enough, works perfectly.
