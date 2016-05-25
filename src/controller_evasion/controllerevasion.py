#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import numpy as np
from time import time

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Int16

#from controller_lane import cfg
#import evading_controller
import fsm
import stateMachineObstacle
from ottocar_msgs.msg import Obstacle
from ottocar_msgs.msg import controller_mode as Speed
import controller
from motion_model import DataDrivenModel
from car import Car

#TODo
#componets



mode_obstacle = stateMachineObstacle.StateMachineObstacle()
mode_traj = fsm.FSM()


class ControllerEvasion:

    def __init__(self):
        print "controller_evasion started"
        self.car = Car()
        self.car.actors.motor_power.enabled = True
        self.car.actors.steer_servo.enabled = True
        self.car.controllers.curvature_dependent_velocity.enabled = True
        self.car.actors.motor_power.enabled = True
        self.car.controllers.mpc.enabled = True
        self.car.sensors.obstacle_detection.enabled =True
        print self.car.sensors.obstacle_detection.obst_front_x
        self.obst_front()
        self.car.spin()



    def obst_front(self):
        print self.car.sensors.obstacle_detection.obst_front_x
        if not self.car.sensors.obstacle_detection.obst_front_x == None:
            print "obstacle"
            self.car.actors.motor_power.enabled = False
            self.car.actors.steer_servo.enabled = False
            self.car.controllers.mpc.enabled = False
            self.car.controllers.curvature_dependent_velocity.enabled = False


if __name__ == '__main__':
    process = ControllerEvasion()
#
#     rospy.init_node('controller_evasion')
#     try:
#         CE = ControllerEvasion()
#     except rospy.ROSInterruptException : pass
#     rospy.spin()
