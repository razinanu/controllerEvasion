#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
from ottocar_msgs.msg import Obstacle
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from ottocar_msgs.msg import controller_mode as Speed
import numpy as np
import fsm
import stateMachineObstacle
import evading_controller
from time import time



mode_obstacle = stateMachineObstacle.stateMachineObstacle()
mode_traj = fsm.FSM()


class ControllerEvasion:


        def __init__(self):

             print "controller_evasion started"
             rospy.Subscriber("/sw/sensors/fusion/obstacle_front", Obstacle, self.obstacle_callback)
             self.pub_angle = rospy.Publisher('/hw/steering', Float32, tcp_nodelay=True, queue_size=1)
             self.pub_speed = rospy.Publisher('/hw/motor/speed', Speed, tcp_nodelay=True, queue_size=1)
             self.pub_blink_right = rospy.Publisher('/sw/blinker/right', Bool, queue_size=1)
             self.pub_blink_left = rospy.Publisher('/sw/blinker/left', Bool, queue_size=1)

             self.lane_controller = evading_controller.EvadingControllerLane()
             self.lane_controller.config["v_max"] = 70
             self.rate = rospy.Rate(300)
             self.spin()


        def spin(self):
            while not rospy.is_shutdown():

                if mode_traj.current == "right" and mode_obstacle.current == "free" :
                        self.lane_controller.config["v_max"] = 70
                        steer, speed = self.lane_controller.control_steer_and_speed()
                        print "drive"
                        self.lane_controller_publisher(steer, speed)
                if mode_traj.current == "left" and mode_obstacle.current == "free":
                        self.lane_controller.config["v_max"] = 70
                        steer, speed = self.lane_controller.control_steer_and_speed()
                        print "drive"
                        self.lane_controller_publisher(steer, speed)
                if mode_obstacle.current == 'firstObstacle' or mode_obstacle.current == 'testObstacle':
                        self.slow_down
                        print "slow down"
                if mode_obstacle.current == 'confirmedObstacle' and mode_traj.current == 'right':
                        mode_traj.lane_change()
                        print "lane change"
                        steer_lane_change, speed_lane_change, time_lane_change, start_time_lane_change = self.lane_change()
                        #print "steer, speed, time , starttime", steer_lane_change, speed_lane_change, time_lane_change, start_time_lane_change
                if mode_traj.current == "firstLaneChange":
                        self.pub_blink_left.publish(True)
                        start_time_lane_change_back = self.lane_change_publisher(steer_lane_change, speed_lane_change, time_lane_change, start_time_lane_change)
                if mode_traj.current == "secondLaneChange":
                        self.back_lane_change(start_time_lane_change_back)
                if mode_traj.current == "left" and mode_obstacle.current == 'confirmedObstacle':
                        mode_traj.merging()
                        print "merging "
                        steer_mergin, speed_merging, time_merging, start_merging = self.merging()
                if mode_traj.current == "firstMerging":
                        self.pub_blink_left.publish(True)
                        start_time_merging_back = self.merging_publisher(steer_mergin, speed_merging, time_merging, start_merging, start_merging)
                if mode_traj.current == "secondMerging":
                         self.back_merging(start_time_merging_back)
                self.rate.sleep()




        def back_merging(self, start_time_back):

            if time() - start_time_back < 1:
                self.pub_angle.publish(-0.4)
                self.pub_speed.publish(Speed(mode=Int16(0), value=Float64(75)))
            else:
                # print "start time back ", start_time_back
                # print "time back ", time(), 'diff back', time() - start_time_back
                mode_traj.complete_right()
                mode_obstacle.finish()
                #print "finish second merging"

        def merging_publisher(self, steer_mergin, speed_merging, time_merging, start_merging,start_time_merging):

            if time() - start_time_merging < time_merging:
                    self.pub_angle.publish(steer_mergin)
                    self.pub_speed.publish(Speed(mode=Int16(0), value=Float64(speed_merging)))

            else:
                    # print "start_time publisher", start_time
                    # print "time ", time(), 'diff', time() - start_time
                    mode_traj.back_steer_right()
                    start_time_back = time()
                    #print "finish merging"
                    return start_time_back

        def merging(self):

                # controler = controller.Controller(NewModel,True)
                # trajectory_conf = np.array([self.lateral_length, self.trj_speed, self.trj_acc])
                # conf_start = np.array([0, 0, 0])
                # conf_model = np.array([0, 0, 0, self.last_speed, 0, 0], dtype="float")
                # long_offset = basetrajectory.BaseTrajectory(trajectory_conf, 0.01).longitudinal_length/2
                # trajectory = basetrajectory.BaseTrajectory(trajectory_conf, 0.01).lane_change_base_trajectory(conf_start, True)
                # steer, speed, time_traj = controler.control(trajectory, conf_model, 0.01, 0.2, long_offset)
                start_time_merging = time()
                #print "start time lane change", start_time_lane_change
                steer = 0.2
                speed = 75
                time_merging = 1
                return steer, speed, time_merging, start_time_merging

        def back_lane_change(self, start_time_back):

            if time() - start_time_back < 1:
                self.pub_angle.publish(0.5)
                self.pub_speed.publish(Speed(mode=Int16(0), value=Float64(75)))
            else:
                # print "start time back ", start_time_back
                # print "time back ", time(), 'diff back', time() - start_time_back
                mode_traj.complete_left()
                mode_obstacle.finish()
                #print "finish second lane change"
                #print mode_obstacle.current


        def lane_change(self):

                # controler = controller.Controller(NewModel,False)
                # trajectory_conf = np.array([self.lateral_length, self.trj_speed, self.trj_acc])
                # conf_start = np.array([0, 0, 0])
                # conf_model = np.array([0, 0, 0, self.last_speed, 0, 0], dtype="float")
                # long_offset = basetrajectory.BaseTrajectory(trajectory_conf, 0.01).longitudinal_length/2
                # trajectory = basetrajectory.BaseTrajectory(trajectory_conf, 0.01).lane_change_base_trajectory(conf_start, False)
                # steer, speed, time_traj = controler.control(trajectory, conf_model, 0.01, 0.2, long_offset)
                # self.run_time = time()
                start_time_lane_change = time()
                #print "start time lane change", start_time_lane_change
                steer = -0.2
                speed = 75
                time_lane_change = 1
                return steer, speed, time_lane_change, start_time_lane_change
        def lane_change_publisher(self, steer, speed, time_lane_change, start_time):

                # print "start_time publisher", start_time
                # print "time ", time(), 'diff', time() - start_time
                if time() - start_time < time_lane_change:
                    self.pub_angle.publish(steer)
                    self.pub_speed.publish(Speed(mode=Int16(0), value=Float64(speed)))

                else:
                    #print "start_time publisher", start_time
                    #print "time ", time(), 'diff', time() - start_time
                    mode_traj.back_steer_left()
                    start_time_back = time()
                    #print "finish first lane change"
                    return start_time_back



        def slow_down(self):

            self.pub_speed.publish(Speed(mode=Int16(0), value=Float64(74)))
            self.pub_angle.publish(0.2)


        def lane_controller_publisher(self, steer, speed):

            self.pub_angle.publish(steer)
            self.pub_speed.publish(Speed(mode=Int16(0), value=Float64(speed)))

        def obstacle_callback(self, msg):


            if msg.point_left.x and mode_obstacle.current == "free":
                obstacle_x = msg.point_left.x
                obstacle_y = np.abs(msg.point_left.y)
                #print mode_obstacle.current

                if obstacle_x:
                    self.first_obstacle = (obstacle_x, obstacle_y)
                    mode_obstacle.obstacle()
                return
            if mode_obstacle.current == "firstObstacle":
                diff = self.first_obstacle[0] - msg.point_left.x
                #print"diff, obstacle", diff, self.first_obstacle[0], msg.point_left.x

                if diff >= 0:
                    self.test_obstacle = (msg.point_left.x, np.abs(msg.point_left.y))
                    mode_obstacle.test()
                    #print mode_obstacle.current
                else:
                    mode_obstacle.searchNext()
                return

            if mode_obstacle.current == "testObstacle":
                diff = self.test_obstacle[0] - msg.point_left.x
                if diff > 0:
                    self.confirmed_obstacle = (msg.point_left.x, np.abs(msg.point_left.y))
                    mode_obstacle.confirm()
                    #print self.confirmed_obstacle


                else:
                    mode_obstacle.reject()



if __name__ == '__main__':

    rospy.init_node('controller_evasion')
    try:
        CE = ControllerEvasion()
    except rospy.ROSInterruptException : pass
    rospy.spin()
