# !/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from ottocar_msgs.msg import Obstacle
import fsm
import basetrajectory
from time import time
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Int16
from ottocar_msgs.msg import controller_mode as Speed
import basetrajectory
import controller
import matplotlib.pyplot as plt
from motion_model import NewModel
import evading_controller


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
        self.lateral_length = 1
        self.trj_speed = 1
        self.last_speed = 0
        self.last_steer = 0
        # #TODO set Acceleration
        self.trj_acc = 2
        self.counter = 0
        self.run_time = time()
        self.lane_controller = evading_controller.EvadingControllerLane()
        self.spin()



    def spin(self):
        #with self.lock:
           while not rospy.is_shutdown():

                    #print "initial mode is", self.mode
                    if self.mode.current == "right" or self.mode.current == "left":
                        #print self.mode.current
                        steer, speed = self.lane_controller.control_steer_and_speed()
                        self.lane_controller_publish(steer, speed)
                        #self.drive()
                    if self.obstacle_x and self.mode.current == "right":
                        # print "obstacle x", self.obstacle_x
                        # print "obstacl y ", self.obstacle_y
                        print self.mode.current
                        self.mode.lane_change()
                        self.slow_down_auto()
                        # self.pub_speed.publish(Speed(mode=Int16(0), value=Float64(-75)))
                        # self.pub_speed.publish(Speed(mode=Int16(0), value=Float64(0)))
                        steer, speed, time_traj = self.lane_change()
                        print self.mode.current
                    if self.mode.current == "firstLaneChange":
                        #print self.mode.current
                        time_back = self.publish(speed, steer, time_traj, False)
                    if self.mode.current == "secondLaneChange":
                        print self.mode.current
                        self.back_left(time_back)

                    if self.obstacle_y and self.mode.current == "left":
                        print self.mode.current
                        self.slow_down_auto()
                        self.mode.merging()
                        steer, speed, time_traj = self.merge()
                    if self.mode.current == "firstMerging":
                        #print self.mode.current
                        time_back= self.publish(speed, steer, time_traj,True)
                    if self.mode.current == "secondMerging":
                        print self.mode.current
                        self.back_right(time_back)


    def back_left(self, time_back):

        if time() - self.run_time < time_back:
            self.pub_angle.publish(0.6)
            self.pub_speed.publish(Speed(mode=Int16(0), value=Float64(85)))
        else:
             self.mode.complete_left()
    def back_right(self, time_back):

        if time() - self.run_time < time_back:
            self.pub_angle.publish(0.5)
            self.pub_speed.publish(Speed(mode=Int16(0), value=Float64(85)))
        else:
             self.mode.complete_right()



    def merge(self):
        # controler = controller.Controller(NewModel,True)
        # trajectory_conf = np.array([self.lateral_length, self.trj_speed, self.trj_acc])
        # conf_start = np.array([0, 0, 0])
        # conf_model = np.array([0, 0, 0, self.last_speed, 0, 0], dtype="float")
        # long_offset = basetrajectory.BaseTrajectory(trajectory_conf, 0.01).longitudinal_length/2
        # trajectory = basetrajectory.BaseTrajectory(trajectory_conf, 0.01).lane_change_base_trajectory(conf_start, True)
        # steer, speed, time_traj = controler.control(trajectory, conf_model, 0.01, 0.2, long_offset)
        self.run_time = time()
        steer = 0.2
        speed = 80
        time_traj = 1
        return steer, speed, time_traj


    def lane_controller_publish(self, steer, speed):

       # print"[evasion]: steer", steer
        self.pub_angle.publish(steer)

        #best for obstacle 75
        #print"[evasion]: speed", speed
        self.pub_speed.publish(Speed(mode=Int16(0), value=Float64(speed)))

    def publish(self, speed, steer, time_traj, is_merging):

        print "time", time_traj
        print "differnce time ", time()-self.run_time
        if time() - self.run_time < time_traj:

            self.pub_angle.publish(steer)
            motor_speed = speed * 100
            if motor_speed > 100:
                motor_speed = 80
                #self.run_time -= 4
            #print "publish speed", motor_speed
            self.pub_speed.publish(Speed(mode=Int16(0), value=Float64(motor_speed)))
            time_back = 2
            return time_back
        else:

            print "time", time_traj
            print "differnce time ", time()-self.run_time
            if is_merging :
                self.mode.back_steer_right()
                self.lane_controller.config["reference_trajectory_offset_x"] = 0.2
                self.run_time = time()
                time_back = 2
                print self.mode.current
                return time_back
            else:
                self.mode.back_steer_left()
                self.run_time = time()
                time_back = 1
                print self.mode.current
                return time_back



    def lane_change(self):

        # controler = controller.Controller(NewModel,False)
        # trajectory_conf = np.array([self.lateral_length, self.trj_speed, self.trj_acc])
        # conf_start = np.array([0, 0, 0])
        # conf_model = np.array([0, 0, 0, self.last_speed, 0, 0], dtype="float")
        # long_offset = basetrajectory.BaseTrajectory(trajectory_conf, 0.01).longitudinal_length/2
        # trajectory = basetrajectory.BaseTrajectory(trajectory_conf, 0.01).lane_change_base_trajectory(conf_start, False)
        # steer, speed, time_traj = controler.control(trajectory, conf_model, 0.01, 0.2, long_offset)
        # self.run_time = time()
        self.run_time = time()
        steer = -0.2
        speed = 80
        time_traj = 1
        return steer, speed, time_traj

    def drive(self):
        self.pub_angle.publish(0.2)
        #best for obstacle 75
        self.pub_speed.publish(Speed(mode=Int16(0), value=Float64(75)))
    def slow_down_auto (self,finish = False ):


        #if self.obstacle_x < 0.60 and self.obstacle_x > 0:
             #self.pub_speed.publish(Speed(mode=Int16(0), value=Float64(-75)))
        #else:
        self.pub_speed.publish(Speed(mode=Int16(0), value=Float64(75)))
        self.pub_angle.publish(0.2)

        if finish :
            self.obstacle_x = 0
            self.obstacle_y = 0

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





