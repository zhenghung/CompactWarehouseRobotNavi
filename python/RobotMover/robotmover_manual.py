import time
import rospy
import math
from pose import Localization 
from geometry_msgs.msg import Twist


class RobotMover:
    def __init__(self):
        rospy.init_node('Mover', anonymous=True)
        self.robotpose = Localization()

    def move_forward(self):
        cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        move_cmd = Twist()
        move_cmd.linear.x = 1.0 

        cmd_vel.publish(move_cmd)


    def rotate_left(self):
        cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        move_cmd = Twist()
        move_cmd.angular.z = 1.0

        cmd_vel.publish(move_cmd)


    def rotate_right(self):
        cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        move_cmd = Twist()
        move_cmd.angular.z = -1.0

        cmd_vel.publish(move_cmd)


    def stop(self):
        cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        move_cmd = Twist()
        cmd_vel.publish(move_cmd)


if __name__ == '__main__':
    mover = RobotMover()
    mover.move_forward()   # Bug: First publish doesnt work

    instruction=[]
    while instruction != ['q']:
        print "[cmd]: ",
        instruction = raw_input()

        cmd = instruction

        if cmd == 'w':
            mover.move_forward()
        elif cmd == 'a':
            mover.rotate_left()
        elif cmd == 'd':
            mover.rotate_right()
        elif cmd == 's':
            mover.stop()




