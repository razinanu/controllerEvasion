# !/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from ottocar_msgs.msg import Obstacle
import fsm
import basetrajectory
import time
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Int16
from ottocar_msgs.msg import controller_mode as Speed
import basetrajectory
import controller as control
import matplotlib.pyplot as plt
from motion_model import NewModel


class ControllerEvasion:

    def __init__(self):

        print "controller_evasion started"
        rospy.Subscriber("/sw/sensors/fusion/obstacle_front", Obstacle, self.obstacle_callback)
        self.pub_angle = rospy.Publisher('/hw/steering', Float32, tcp_nodelay=True, queue_size=1)
        self.pub_speed = rospy.Publisher('/hw/motor/speed', Speed, tcp_nodelay=True, queue_size=1)

        #self.pub_angle = rospy.Publisher('/sw/controller/evasion/angle', ControllerEvasion, tcp_nodelay=True, queue_size=1)
        #self.pub_speed = rospy.Publisher('/sw/controller/evasion/speed', ControllerEvasion, tcp_nodelay=True, queue_size=1)
        # self.pub_state = rospy.Publisher('/sw/controller/evasion/state', ControllerEvasion, tcp_nodelay=True, queue_size=1)
        # self.pub_offset_change = rospy.Publisher('/sw/controller/evasion/offset_change', ControllerEvasion, tcp_nodelay=True, queue_size=1)
        #rospy.Subscriber('/sw/controller/remote/speed', Speed, self.callback_speed)
        self.isObstacle = False,
        self.obstacle_x = 0
        self.obstacle_y = 0
        self.mode = fsm.FSM()
        self.lateral_length= 0.5
        # #TODO set Acceleration
        # self.lateral_acc = 2
        # self.last_angle = 0
        # self.last_speed = 0
        self.spin()

    def spin(self):
        #with self.lock:
           while not rospy.is_shutdown():
                    #print "initial mode is", self.mode
                    if self.mode.current == "right" or self.mode.current == "left":
                        print "mode", self.mode.current
                        self.drive()
                    if self.obstacle_x and self.mode.current == "right":
                        print "obstacle x", self.obstacle_x
                        print "obstacl y ", self.obstacle_y
                        self.stop_auto()
                        self.mode.lane_change()
                    if self.obstacle_y and self.mode.current == "left":
                        self.stop()
                        self.mode.merging()
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
    def drive(self):
        self.pub_angle.publish(0.2)
        #best for obstacle 75
        self.pub_speed.publish(Speed(mode=Int16(0), value=Float64(75)))
    def stop_auto (self):
        self.pub_angle.publish(0.2)
        #self.pub_speed.publish(Speed(mode=Int16(0), value=Float64(0)))
        print "stop"
    def obstacle_callback (self, msg):

        self.obstacle_x = msg.point_left.x
        self.obstacle_y = np.abs(msg.point_left.y)


if __name__ == '__main__':
    rospy.init_node('controller_evasion')

    try:
        CE = ControllerEvasion()
    except rospy.ROSInterruptException: pass
    rospy.spin()





