#!/usr/bin/env python
import rospy
# need install by pip
import keyboard
# TODO: add custom message
from std_msgs.msg import ColorRGBA

def keylogger():
    pub = rospy.Publisher('drive', ColorRGBA, tcp_nodelay=True, queue_size=1)
    rospy.init_node('keylogger', anonymous=True)
    rate = rospy.Rate(2000)
    last = ColorRGBA(0, 0, 0, 0)
    while not rospy.is_shutdown():
        hello_str = ColorRGBA(keyboard.is_pressed('up')-keyboard.is_pressed('down'), keyboard.is_pressed('right')-keyboard.is_pressed('left'), 3, 4)
        if hello_str != last:
            last = hello_str
            rospy.loginfo(hello_str)
            pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        keylogger()
    except rospy.ROSInterruptException:
        pass
