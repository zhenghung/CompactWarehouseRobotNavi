#!/usr/bin/env python
import roslib; roslib.load_manifest('imu_read')
import rospy
import tf
from imu_read.msg import imu_read
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

# Odomerty publisher

global x,y


def listener():
        rospy.init_node('RepublishOdom', anonymous=False)
        rospy.Subscriber("imu_read", imu_read, msgCallback)
        while not rospy.is_shutdown():
		rospy.spin()      	

def msgCallback(msg):
	x,y,t,gx,gy,gz,ax,ay,az,mx,my,mz = msg.x, msg.y, msg.theta, msg.gx, msg.gy, msg.gz,msg.ax, msg.ay, msg.az,msg.mx, msg.my, msg.mz
	odom_publish(x,y)
        tf_publish(x,y,t)
        imu_publish(gx,gy,gz,ax,ay,az)
        rospy.loginfo(rospy.get_name()+"(%.3f)"%(x))

def odom_publish(x,y):
        odom = rospy.Publisher("/odom", Odometry, queue_size=1)
        odomMsg = Odometry()

        odomMsg.header.stamp = rospy.get_rostime()
        odomMsg.header.frame_id = "odom"
        odomMsg.child_frame_id = "chassis"

        odomMsg.pose.pose.position.x = x
        odomMsg.pose.pose.position.y = y
        odomMsg.pose.pose.position.z = 0

        odomMsg.pose.pose.orientation.x = 0
        odomMsg.pose.pose.orientation.y = 0
        odomMsg.pose.pose.orientation.z = 1
        odomMsg.pose.pose.orientation.w = 0

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

def imu_publish(gx,gy,gz,ax,ay,az): 	
	imu = rospy.Publisher("/Imu", Imu, queue_size=1)
        imuMsg = Imu()
	
        imuMsg.header.stamp = rospy.get_rostime()
	imuMsg.orientation_covariance[0] = -1;
	imuMsg.angular_velocity.x = gx*3.1415/180.0;
	imuMsg.angular_velocity.y = gy*3.1415/180.0;
	imuMsg.angular_velocity.z = gz*3.1415/180.0;
	imuMsg.linear_acceleration.x = ax*9.8;
	imuMsg.linear_acceleration.y = ay*9.8;
	imuMsg.linear_acceleration.z = az*9.8;

        imu.publish(imuMsg)   

if __name__ == '__main__':
	listener()