# !/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from ottocar_msgs.msg import Obstacle
# import fsm
# import basetrajectory
# import time
# import numpy as np
# import basetrajectory
# import controller as control
# import matplotlib.pyplot as plt
# from motion_model import NewModel


class ControllerEvasion:

    def __init__(self):

        print "cntroller_evasion started"
        rospy.Subscriber("/sw/sensors/fusion/obstacle", Obstacle, self.obstacle_callback())
        # self.pub_angle = rospy.Publisher('/sw/controller/evasion/angle', ControllerEvasion, tcp_nodelay=True, queue_size=1)
        # self.pub_speed = rospy.Publisher('/sw/controller/evasion/speed', ControllerEvasion, tcp_nodelay=True, queue_size=1)
        # self.pub_state = rospy.Publisher('/sw/controller/evasion/state', ControllerEvasion, tcp_nodelay=True, queue_size=1)
        # self.pub_offset_change = rospy.Publisher('/sw/controller/evasion/offset_change', ControllerEvasion, tcp_nodelay=True, queue_size=1)
        #rospy.Subscriber('/sw/controller/remote/speed', Speed, self.callback_speed)

        # self.mode = fsm.FSM()
        # self.lateral_length= 0.5
        # #TODO set Acceleration
        # self.lateral_acc = 2
        # self.last_angle = 0
        # self.last_speed = 0
        #self.spin()
    #
    # def callback_speed(self, msg):
    #     with self.lock:
    #         #ToDo
    #         self.speed = (msg.value.data/100)
    #
    # def obstacle_callback(self, msg):
    #      with self.lock:
    #          self.obstacl_x= msg.value.data

               #ToDo pass
   # def spin(self):
        #with self.lock:
           # while not rospy.is_shutdown() and self.running:

                    # if(self.isObstacle):
                    #     if self.mode.isstate("right"):
                    #         steer,speed, time = self.lane_change()
                    #         self.mode.lane_change()
                #     if(self.mode.current == "right"):

                #
                #
                #
                # state=None
                # offset=None
                # if angle is not None:
                #     self.last_angle = angle
                #     self.last_speed = speed
                #     self.last_state=state
                #     self.last_offset=offset
                #     #ToDo
                #     speed = None
                #     self.publish(angle, speed, offset , state)

                # elif self.last_angle is not None:
                #         # repeat sending last values, to avoid hw timeout which stops car
                #         self.publish(self.last_angle, self.last_speed, self.last_offset, self.last_state)

    # def lane_change(self):
    #
    #     controller = control.controller(NewModel, False)
    #     trajectory_conf = np.array([self.lateral_length, self.last_speed, self.lateral_acc])
    #     conf_start = np.array([0, 0, 0])
    #     conf_model = np.array([0, 0, 0, self.last_speed, 0, 0], dtype="float")
    #     long_offset = basetrajectory.BaseTrajectory(trajectory_conf, 0.01).longitudinal_length/2
    #     trajectory = basetrajectory.BaseTrajectory(trajectory_conf, 0.01).lane_change_base_trajectory(conf_start, False)
    #     steer, speed, time = controller.control(trajectory, conf_model, 0.01, 0.2, long_offset)
    #     return steer, speed, time
        # plt.plot(trajectory[:, 0], trajectory[:, 1], 'g')
        # plt.show()

    # def publish(self, angle, speed, offset, state):
    #
    #     self.pub_angle.publish(data = angle)
    #     self.pub_speed.publish(data = speed)
    #     self.pub_offset_change(data=offset)
    #     self.pub_state(data=state)
    def obstacle_callback (self, msg):

        self.obstacle_x = msg.point_left.x
        self.obstacle_y = msg.point_left.y
        print "obstacle x", self.obstacle_x
        print "obstacl y ", self.obstacle_y

if __name__ == '__main__':
    rospy.init_node('controller_evasion')

    try:
        CE = ControllerEvasion()
    except rospy.ROSInterruptException: pass
    rospy.spin()





