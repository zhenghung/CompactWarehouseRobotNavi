import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class Localization:
    def __init__(self):
        try:
            rospy.init_node('get_odom', anonymous=False)
        except rospy.exceptions.ROSException as e:
            print (e)
        self.positionX = 0
        self.positionY = 0
        self.positionPitch = 0
        path = '/odom'
        rospy.Subscriber(path, Odometry, self.callback)
        self.rate = rospy.Rate(1)

    def callback(self,msg):
        #print msg.pose.pose.position.x
        self.positionX=msg.pose.pose.position.x
        self.positionY=msg.pose.pose.position.y
        roll, pitch, yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        self.positionPitch=yaw
        #self.rate.sleep()
