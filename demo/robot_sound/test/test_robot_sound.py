#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def main():
    rospy.init_node('test_robot_sound', anonymous=True)
    pub = rospy.Publisher('/robot_speak', String, queue_size=1)
    r = rospy.Rate(2)
    while not rospy.is_shutdown():
        pub.publish("hello world")
        r.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

