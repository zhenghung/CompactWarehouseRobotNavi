#!/usr/bin/env python
import roslib; roslib.load_manifest('imu_read')
import rospy
import tf
from tf.transformations import quaternion_from_euler
from imu_read.msg import imu_read
from nav_msgs.msg import Odometry

# Odomerty publisher

global x,y


def listener():
        rospy.init_node('RepublishOdom', anonymous=False)
        rospy.Subscriber("imu_read", imu_read, msgCallback)
        while not rospy.is_shutdown():
		rospy.spin()      	

def msgCallback(msg):
	x,y,t = msg.x, msg.y, msg.theta
	odom_publish(x,y,t)
        tf_publish(x,y,t)
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
        odomMsg.twist.twist.linear.y = -0.1
        odomMsg.twist.twist.angular.z = 0.1

	odom.publish(odomMsg)   

def tf_publish(x,y,t): 	
        odom_broadcaster = tf.TransformBroadcaster()
        odom_broadcaster.sendTransform(
        	(x/1000, y/1000, 0.),
        	tf.transformations.quaternion_from_euler(0, 0, t),
        	rospy.Time.now(),
        	"chassis",
        	"odom")   

def imu_publish(x,y,t): 	
	imu = rospy.Publisher("/Imu", Imu, queue_size=1)
        imu.publish()   

if __name__ == '__main__':
	listener()

