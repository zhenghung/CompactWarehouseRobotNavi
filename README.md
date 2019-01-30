# Project Overview

The following documentation outlines the steps required to run the CompactWarehouseRobotNavi (UCL EEE Final Year Project).

# Contents

- Clone Repository
- Project structure
- Installation and Setup
- Running the Program
    - Running on Gazebo Simulation
    - Running on a Real Robot
- FAQ / Troubleshooting


# Clone Repository

Git clone from the GitHub Repository and checkout to the master branch
```
$ git clone https://github.com/zhenghung/CompactWarehouseRobotNavi
$ git checkout master
```


# Installation and Setup 
## Packages and Libraries Required
---
### Arduino
1. rosserial


### Python
1. ROS-kinetic
2. rospy


## Setting up ROS addresses for remote ros server
**Required**: Local Network allowing port forwarding, if none is available a locally hosted hotspot works just fine.

## On server device (hosts roscore)
1. Find out your local IP address.
```
$ hostname -I
```

2. Add the config to your .bashrc file, replacing `<Remote Computer IP>` with the local IP address, e.g. `10.42.0.1`. 
```
$ echo "export ROS_IP=0.0.0.0" >> ~/.bashrc
$ echo "export ROS_HOSTNAME={YOUR IP ADD}" >> ~/.bashrc
$ echo "export ROS_MASTER_URI=http://{YOUR IP ADD}:11311" >> ~/.bashrc
```

**Note that your local IP changes based on the network your computer is connected to, so edit the ~/.bashrc file as needed**

