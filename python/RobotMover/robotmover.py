import time
import rospy
from geometry_msgs.msg import Twist


class RobotMover:
    def __init__(self):
        rospy.init_node('Mover', anonymous=True)

    def move_forward(self, duration):
        cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        move_cmd = Twist()
        move_cmd.linear.x = 1.0 

        cmd_vel.publish(move_cmd)
        time.sleep(duration)

        move_cmd.linear.x = 0.0
        cmd_vel.publish(move_cmd)

    def rotate_left(self, duration):
        cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        move_cmd = Twist()
        move_cmd.angular.z = 1.0

        cmd_vel.publish(move_cmd)
        time.sleep(duration)

        move_cmd.angular.z = 0.0
        cmd_vel.publish(move_cmd)

    def rotate_right(self, duration):
        cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        move_cmd = Twist()
        move_cmd.angular.z = -1.0

        cmd_vel.publish(move_cmd)
        time.sleep(duration)

        move_cmd.angular.z = 0.0
        cmd_vel.publish(move_cmd)


if __name__ == '__main__':
    mover = RobotMover()
    mover.move_forward(0)   # Bug: First publish doesnt work

    instruction=[]
    while instruction != ['q']:
        print "[cmd dur]: ",
        instruction = raw_input().split()

        if len(list(instruction))>1:
            cmd, dur = instruction
        else:
            cmd = instruction

        if cmd == 'w':
            mover.move_forward(float(dur))
        elif cmd == 'a':
            mover.rotate_left(float(dur))
        elif cmd == 'd':
            mover.rotate_right(float(dur))



