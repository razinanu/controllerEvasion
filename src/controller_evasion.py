# !/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from ottocar_msgs.msg import Obstacle


class Controller_Evasion():
    def __init__(self):
        print "cntroller_evasion started"
        rospy.Subscriber("/sw/sensors/fusion/obstacle",Obstacle,obstacle_callback)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            rate.sleep()
        rospy.spin()
    def obstacle_callback(Obstacle):
        pass

if __name__ == '__main__':
    rospy.init_node('controller_evasion')
    try:
        CE=Controller_Evasion()
    except rospy.ROSInterruptException: pass


