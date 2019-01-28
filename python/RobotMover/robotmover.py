import time
import rospy
from geometry_msgs.msg import Twist


class RobotMover:
    def __init__(self):
        rospy.init_node('GoForward', anonymous=False)

    def move_forward(self, duration):
        cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        move_cmd = Twist()
        move_cmd.linear.x = 1.0 
        move_cmd.angular.z = 0

        begin_time = time.time()

        while time.time() - begin_time < duration:
            cmd_vel.publish(move_cmd)

        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        cmd_vel.publish(move_cmd)

    def rotate_left(self, duration):
        cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 1.0

        begin_time = time.time()

        while time.time() - begin_time < duration:
            cmd_vel.publish(move_cmd)

        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        cmd_vel.publish(move_cmd)

    def rotate_right(self, duration):
        cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = -1.0

        begin_time = time.time()

        while time.time() - begin_time < duration:
            cmd_vel.publish(move_cmd)

        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        cmd_vel.publish(move_cmd)


if __name__ == '__main__':
    mover = RobotMover()

    mover.move_forward(5)
    time.sleep(1)
    mover.rotate_right(2)
    time.sleep(1)
    mover.move_forward(5)

