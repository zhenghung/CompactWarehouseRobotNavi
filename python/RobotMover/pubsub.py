#!/usr/bin/env python
import roslib; roslib.load_manifest('imu_read')
import rospy
import tf
import math
from tf.transformations import quaternion_from_euler
from imu_read.msg import imu_read
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

# Odomerty publisher

global x,y


def listener():
        rospy.init_node('Republiser', anonymous=False)
        rospy.Subscriber("CompressedMsg", imu_read, msgCallback)
        while not rospy.is_shutdown():
		rospy.spin()      	

def msgCallback(msg):
	x,y,t = msg.x, msg.y, msg.theta
        gx,gy,gz = msg.gx, msg.gy, msg.gz
        ax,ay,az = msg.ax, msg.ay, msg.az
        
	odom_publish(x,y,t)
        imu_publish(gx,gy,gz,ax,ay,az)
        rospy.loginfo(rospy.get_name()+"(%.3f)"%(x))

def odom_publish(x,y,t):
        quat = quaternion_from_euler(0, 0, t)


        odom = rospy.Publisher("/odom", Odometry, queue_size=1)
        odomMsg = Odometry()

        odomMsg.header.stamp = rospy.get_rostime()
        odomMsg.header.frame_id = "odom"
        odomMsg.child_frame_id = "chassis"

        odomMsg.pose.pose.position.x = x
        odomMsg.pose.pose.position.y = y
        odomMsg.pose.pose.position.z = 0

        odomMsg.pose.pose.orientation.x = quat[0]
        odomMsg.pose.pose.orientation.y = quat[1]
        odomMsg.pose.pose.orientation.z = quat[2]
        odomMsg.pose.pose.orientation.w = quat[3]

        odomMsg.twist.twist.linear.x = 0.1
        odomMsg.twist.twist.linear.y = 0
        odomMsg.twist.twist.angular.z = 0.1

	odom.publish(odomMsg)   

def imu_publish(gx,gy,gz,ax,ay,az):	
        imu = rospy.Publisher("/Imu", Imu, queue_size=1)
        imuMsg = Imu()

        imuMsg.header.stamp = rospy.get_rostime()
        imuMsg.orientation_covariance[0] = -1
        imuMsg.angular_velocity.x = math.radians(gx)
        imuMsg.angular_velocity.y = math.radians(gy)
        imuMsg.angular_velocity.z = math.radians(gz)
        imuMsg.linear_acceleration.x = ax*9.81
        imuMsg.linear_acceleration.y = ay*9.81
        imuMsg.linear_acceleration.z = az*9.81

        imu.publish(imuMsg)      

if __name__ == '__main__':
	listener()

