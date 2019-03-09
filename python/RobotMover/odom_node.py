import rospy
import tf
from nav_msgs.msg import Odometry

class Odom:
    def __init__(self):
        rospy.init_node('RepublishOdom', anonymous=False)
        self.odom = rospy.Publisher("/odom", Odometry, queue_size=1)
        self._tf_subscribe()

    def _tf_subscribe(self):
        listener = tf.TransformListener()

        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform('/odom', '/chassis', rospy.Time(0))
                self._odom_publish(trans, rot)
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
    
            except KeyboardInterrupt:
                print "Ended with Keyboard Interrupt"

    def _odom_publish(self, trans, rot):
        odomMsg = Odometry()

        odomMsg.header.stamp = rospy.get_rostime()
        odomMsg.header.frame_id = "odom"
        odomMsg.child_frame_id = "chassis"

        odomMsg.pose.pose.position.x = trans[0]
        odomMsg.pose.pose.position.y = trans[1]
        odomMsg.pose.pose.position.z = trans[2]

        odomMsg.pose.pose.orientation.x = rot[0]
        odomMsg.pose.pose.orientation.y = rot[1]
        odomMsg.pose.pose.orientation.z = rot[2]
        odomMsg.pose.pose.orientation.w = rot[3]

        odomMsg.twist.twist.linear.x = 0.1
        odomMsg.twist.twist.linear.y = -0.1
        odomMsg.twist.twist.angular.z = 0.1

        self.odom.publish(odomMsg)


if __name__ == '__main__':
    myOdom = Odom()