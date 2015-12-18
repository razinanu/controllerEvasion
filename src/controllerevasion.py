# !/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from ottocar_msgs.msg import Obstacle
import base_trajectory


class ControllerEvasion():
    def __init__(self):

        print "cntroller_evasion started"
        rospy.Subscriber("/sw/sensors/fusion/obstacle", Obstacle, self.obstacle_callback())
        self.pub_angle = rospy.Publisher('/sw/controller/evasion/angle', ControllerEvasion, tcp_nodelay=True, queue_size=1)
        self.pub_speed = rospy.Publisher('/sw/controller/evasion/speed', ControllerEvasion, tcp_nodelay=True, queue_size=1)
        self.pub_state = rospy.Publisher('/sw/controller/evasion/state', ControllerEvasion, tcp_nodelay=True, queue_size=1)
        self.pub_offset_change = rospy.Publisher('/sw/controller/evasion/offset_change', ControllerEvasion, tcp_nodelay=True, queue_size=1)

        self.last_angle = None
        self.last_speed = None
        self.last_state= None
        self.last_offset=None


        self.spin()

    def obstacle_callback(self, msg):
       pass
       #with self.lock:
               #ToDo


    def spin(self):
        with self.lock:
            while not rospy.is_shutdown() and self.running:
                #ToDo
                angle, solution = None
                state=None
                offset=None
                if angle is not None:
                    self.last_angle = angle
                    self.last_speed = speed
                    self.last_state=state
                    self.last_offset=offset

                    #ToDo
                    speed = None
                    self.publish(angle, speed, offset , state)

                elif self.last_angle is not None:
                        # repeat sending last values, to avoid hw timeout which stops car
                        self.publish(self.last_angle, self.last_speed, self.last_offset, self.last_state)

    def publish(self, angle, speed, offset, state):

        self.pub_angle.publish(data = angle)
        self.pub_speed.publish(data = speed)
        self.pub_offset_change(data=offset)
        self.pub_state(data=state)




if __name__ == '__main__':
    rospy.init_node('controller_evasion')
    try:
        CE = ControllerEvasion()
    except rospy.ROSInterruptException: pass





