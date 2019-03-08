import time
import rospy
import math
from pose import Localization 
from geometry_msgs.msg import Twist


class RobotMover:
    def __init__(self):
        rospy.init_node('Mover', anonymous=True)
        self.robotpose = Localization()

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

    def stop(self):
        cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        move_cmd = Twist()
        cmd_vel.publish(move_cmd)

    def rotate_angle(self, angle):
        cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        move_cmd = Twist()
        if angle > 0:
            move_cmd.angular.z = 1.0
        elif angle < 0:
            move_cmd.angular.z = -1.0

        cmd_vel.publish(move_cmd)
                        
        def _convert_within_2pi(phi):
            if phi > math.pi:
                return phi - 2*math.pi
            elif phi < -math.pi:
                return phi + 2*math.pi
            else:
                return phi

        target_radians = angle * math.pi/180
        target = _convert_within_2pi(self.robotpose.positionPitch + target_radians)
        print target_radians, target
        theta = self.robotpose.positionPitch
        
        while (abs(_convert_within_2pi(theta - target)) > 0.1):
            theta = self.robotpose.positionPitch
            print "Current Angle", theta

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
        elif cmd == 's':
            mover.stop()
        elif cmd == 'x':
            mover.rotate_angle((float(dur)))



