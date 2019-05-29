# Local Hotspot
Create a new private hotspot on Ubuntu Linux
1. Click the wifi icon on the top right corner of the screen next to the battery levels and clock.
2. Select `Create New Wi-Fi Network` 
3. Type in any network name you like and a security key.
4. Press OK
5. Click on the wifi icon again.
6. Select `Connect to Hidden Wi-Fi Network...`
7. Under Connection, select the one you just created
8. Go to Edit Connections, choose the one you just made.
9. Change the mode to Hotspot
10. Change the Wi-Fi Security to use WPA and WPA2 Personal
11. Ensure you're connected to your own hotspot
12. Check with a wireless device (like a smartphone)

# SSH to NUC
## On NUC
1. Hook the NUC up to a monitor, keyboard and mouse
2. Power Adapter is in the blue tray
3. Password is `123456`
4. Go to wireless networks in the NUC
5. Forget the Zheng spot
6. Connect to your own hotspot
7. Once connected, open the terminal(On NUC) and find out your local IP
```
hostname -I
```
or
```
ifconfig
```

## On your laptop (hosting the hotspot)
1. Open terminal and run this:
```
ssh -X -C robotics@<IP Address of NUC discovered earlier>
```
for example
```
ssh -X -C robotics@10.42.0.39
```

2. It may ask some security verification thingy just type `Y` for yes
3. Once connected you will have to type the password `123456`
4. The terminal should now be connected to the NUC
5. Try typing in the SSHed terminal and if you experience significant lag (>1 second), disconnect (type exit or just close it), restart you wifi adapter and reconnect
```
sudo service network-manager restart
```


# Matching the time
## On Laptop
Use 
```
date
```
on the laptop to find out the current time

## On SSHed NUC terminal
1. Change the date on the NUC first, it resets the clock to 00:00:00

Example:
```
sudo date +%Y%m%d -s "20120418"
```

2. Change the time right down to the second to match the laptop's

```
sudo date +%T -s "XX:XX:XX"
```
Changing the X to the corresponding time, example:
```
sudo date +%T -s "14:29:20"
```


# ROS Setup
## On Laptop 
1. Connect to your local hotspot (disconnected from internet)
2. Note down your local IP address (e.g. 10.42.0.1)
```
hostname -I
```
or
```
ifconfig
```
3. Modify the .bashrc file in your home folder (gedit or vim)
```
vim ~/.bashrc
```
Change the ROS settings (at the bottom of the .bashrc file) to 

Press `i` in the vim environment to edit it.
```
# ROS Master
export ROS_IP=0.0.0.0

# Personal Hotspot (Local Network)
export ROS_HOSTNAME=<LAPTOP LOCAL IP ADDRESS>
export ROS_MASTER_URI=http://<LAPTOP LOCAL IP ADDRESS>:11311
```

To save and quit after editing the file, press `ESC` to exit editing mode, then type `:wq` followed by `ENTER` to write and quit to terminal.

4. Try starting ROS Master
```
roscore
```

The ROS Master address shown in the terminal should be the your laptop's local address


## On NUC (SSH from laptop)
1. Same as the laptop, use vim to modify the .bashrc file
```
vim ~/.bashrc
```
But this time, for `ROS_HOSTNAME`, input the local IP of the NUC instead
```
# ROS Master
export ROS_IP=0.0.0.0

# Personal Hotspot (Local Network)
export ROS_HOSTNAME=<NUC LOCAL IP ADDRESS>
export ROS_MASTER_URI=http://<LAPTOP LOCAL IP ADDRESS>:11311
```

2. Restart the terminal, try starting roscore from the laptop, then type 
```
rostopic list
```
To check if it is indeed connected to the same ROS network


# Update Arduino Code (Any computer with Arduino with Custom Message installed)
1. Update your local git repository
```
git pull https://github.com/zhenghung/CompactWarehouseRobotNavi
```

2. Checkout to the `master-test` branch
```
git checkout master-test
```

3. Open the arduino code in 

`/CompactWarehouseRobotNavi/arduino/Project_Robot_ROS/Project_Robot_ROS.ino`

4. In the Arduino IDE, change the board type to `Arduino MEGA 2560`
5. Connect the arduino to the USB port, select the correct serial port and upload the code
6. May throw errors but if the progress bar is still there, just wait a bit. It will say 'Done Uploading' if it succeeds
7. If not, reconnect the Arduino double check all settings and try again.

# Running the Robot (Ignoring the Navigation Stuff)
NOTE: IF THERE ARE PROBLEMS, TRY RECONNECTING THE PHYSICAL HARDWARE

Connect to hotspot

## Terminal 1
```
roscore
```

## Terminal 2 (SSH to NUC)
Run the serial node (arduino)
```
serial0
```
This is the one that often crashes, in which case just `Ctrl-C` and run `serial0` again to reset it.

## Terminal 3 (SSH to NUC)
Run the lidar node
```
lidar0
```
or 
```
lidarport0
```
Soz, can't quite remember, just use `TAB` to autocomplete it

If there are problems, it's probably the wires.

## Terminal 4 (SSH to NUC)
Run the IMU node
```
imulaunch
```
I think..., again just use `TAB` for autocomplete.

## Terminal 5 (any)
Optional - Manual control
```
python ~/CompactWarehouseRobotNavi/python/RobotMover/robotmover_gui.py
```
Or change the directory to wherever you put the files. You can even just `cd` to that directory and python run the script.

## Terminals X (Laptop)
ALL DEM NAVIGATION STUFF

