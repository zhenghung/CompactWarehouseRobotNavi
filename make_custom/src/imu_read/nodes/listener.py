#!/usr/bin/env python
import roslib; roslib.load_manifest('imu_read')
import rospy
from imu_read.msg import imu_read

def msgCallback(data):
   #msgString = "I read values (%.3f,%.3f,%.3f,%.3f,%.3f,%.3f)" % (data.x,
   #     data.y, data.gz, data.theta, data.gx, data.gy)
    
    msgString = "10"
    rospy.loginfo(rospy.get_name()+msgString)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("imu_read", imu_read, msgCallback)
    rospy.spin()

if __name__ == '__main__':
    listener()


