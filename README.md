# Project Overview

The following documentation outlines the steps required to run the CompactWarehouseRobotNavi (UCL EEE Final Year Project).

# Contents

- [Clone Repository](https://github.com/zhenghung/CompactWarehouseRobotNavi#clone-repository)
- [Project structure](https://github.com/zhenghung/CompactWarehouseRobotNavi#project-structure)
- [Installation and Setup](https://github.com/zhenghung/CompactWarehouseRobotNavi#installation-and-setup)
- [Running the Program]()
    - [Running and controlling robot manually](https://github.com/zhenghung/CompactWarehouseRobotNavi#running-and-controlling-robot-manually)
    - [Running robot with navigation](https://github.com/zhenghung/CompactWarehouseRobotNavi#running-the-robot-with-navigation)
- FAQ / Troubleshooting


# Clone Repository

Git clone from the GitHub Repository and checkout to the master branch
```
$ git clone https://github.com/zhenghung/CompactWarehouseRobotNavi
$ git checkout master
```

# Project Structure



# Installation and Setup 
## Packages and Libraries Required

1. ROS-kinetic
2. Arduino
3. rosserial


## Setting up ROS addresses for remote ros server
**Required**: Local Network allowing port forwarding, if none is available a locally hosted hotspot works just fine.

## On server device (ROS Master)
1. Find out your local IP address.
```
$ hostname -I
```

2. Add the config to your .bashrc file, replacing `<Remote Computer IP>` with the local IP address, e.g. `10.42.0.1`. 
```
$ echo "export ROS_IP=0.0.0.0" >> ~/.bashrc
$ echo "export ROS_HOSTNAME=<Remote Computer IP>" >> ~/.bashrc
$ echo "export ROS_MASTER_URI=http://<Remote Computer IP>:11311" >> ~/.bashrc
```

## On remote device (NUC)
1. Connect to the same local network as ROS Master
2. Find your local IP address.
```
$ hostname -I
```

3. Add the config to your .bashrc file, 
   * Replace `<Remote Computer IP>` with the local IP address, e.g. `10.42.0.41`.
   * Replace `<ROS Master IP>` with the ROS Master IP address, e.g. `10.42.0.1`. (Must be same as the server device ROS_MASTER_URI)
```
$ echo "export ROS_IP=0.0.0.0" >> ~/.bashrc
$ echo "export ROS_HOSTNAME=<Remote Computer IP>" >> ~/.bashrc
$ echo "export ROS_MASTER_URI=http://<ROS Master IP>:11311" >> ~/.bashrc
```

**Note that your local IP changes based on the network your computer is connected to, so edit the ~/.bashrc file as needed, then source the file**


# Running and controlling robot manually
## Terminal 1 - Server Device
Start ROS Master.
```
$ roscore
```

## Terminal 2 - Robot Device (ssh into NUC)
Start arduino serial node.
Replace `<Arduino Serial Port>` with the serial port assigned. (e.g. `/dev/ttyACM0`) 
```
$ sudo chmod 666 <Arduino Serial Port>
$ rosrun rosserial_python serial_node.py <Arduino Serial Port>
```

## Terminal 3 - Any remote device, can be server device
From within the repository directory,
```
$ python python/robotmover/robotmover_manual.py
```
Enter a command and press the `<ENTER>` key. Robot will execute last command until told otherwise. 

**Note: Robot will not stop automatically** 

Commands:
* `w`  : Robot moves forward
* `a`  : Robot turns left
* `s`  : Robot stops
* `d`  : Robot turns right


## Terminal 4 (Optional) - Server device
To view the robot position in RVIZ. Republish the `/CompressedMsg` topic into `/odom`

From within the repository directory,
```
$ python python/robotmover/pubsub.py
```

## Terminal 5 (Optional - Server device)
Then open rviz to view `/odom`
```
$ rosrun rviz rviz
```


# Running the robot with navigation
## Terminal 1 - Server device
Start ROS Master.
```
$ roscore
```

## Terminal 2 - Robot Device (ssh into NUC)
Start arduino serial node.
Replace `<Arduino Serial Port>` with the serial port assigned. (e.g. `/dev/ttyACM0`) 
```
$ sudo chmod 666 <Arduino Serial Port>
$ rosrun rosserial_python serial_node.py <Arduino Serial Port>
```

## Terminal 3 - Robot Device (ssh into NUC)
Start lidar serial node.
Replace `<Lidar Serial Port>` with the serial port assigned. (e.g. `/dev/ttyUSB0`) 
```
$ sudo chmod 666 <Lidar Serial Port>
$ roslaunch rplidar_ros rplidar.launch
```

## Terminal 4 - Server device
Republish the `/CompressedMsg` topic into `/odom`.

From within the repository directory,
```
$ python python/robotmover/pubsub.py
```
