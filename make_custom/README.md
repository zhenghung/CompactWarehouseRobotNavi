This is the entire catkin workspace for generating a Custom Msg package for Arduino.
Also contains publisher subscriber node within ROS package folder imu_read

Instructions for running:
1. Run IMU_rosserial.ino file in Arduino folder for reading IMU values
2. Create arduino library for custom msg: rosrun rosserial_arduino make_libraries.py <arduino libraries path> imu_read
3. Start roscore
4. Publish to /imu_read by running rosrun rosserial_python serial_node.py <your USB port>
5. rosrun imu_read pubsub.py to subscribe to /imu_read and publish to /Odom, /tf and /Imu
6. check rostopics are present (/imu_read, /Imu, /Odom etc.)
