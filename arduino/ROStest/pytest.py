import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
#def talker():
#    pub = rospy.Publisher('/brightness', Int32, queue_size=10)
#    rospy.init_node('talker', anonymous=False)
#    #rate = rospy.Rate(10) # 10hz
#    while not rospy.is_shutdown():
#	hello_str = int(raw_input())
#    	rospy.loginfo(hello_str)
#    	pub.publish(hello_str)
#        #rate.sleep()

if __name__ == '__main__':
#    try:
#        talker()
#    except rospy.ROSInterruptException:
#        pass
    brightnesspub = rospy.Publisher('/brightness', Int32, queue_size=1)
    ledpub = rospy.Publisher('/led',Int32, queue_size=1)
    statepub = rospy.Publisher('/state', Int32, queue_size=1)
    rospy.init_node('python', anonymous=False)
    
    while True:
        brightness, led, state = map(int, raw_input('[brightness led state]: ').split())
        #brightness = int(raw_input('brightness: '))
        #led = int(raw_input('led: '))
        #state = int(raw_input('state: '))
        brightnesspub.publish(brightness)
        ledpub.publish(led)
        statepub.publish(state)
