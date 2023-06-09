#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String
from joystick_input.msg import JoyStickOut
from JoyStickController import JoyStick

def talker():

    pub = rospy.Publisher('JoyStickOut', JoyStickOut, queue_size=10)
    rospy.init_node('joystickreader', anonymous=True)
    rate = rospy.Rate(120)   # 10hz
    joystick = JoyStick()
    while not rospy.is_shutdown():
        joystick.read_ros()
        
        pub.publish(joystick.msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
