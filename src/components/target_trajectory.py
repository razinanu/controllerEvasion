#!/usr/bin/env python2
# -*- coding: utf-8 -*-

class TargetTrajectory(object):
    def __init__(self, *args, **kwargs):
        super(TargetTrajectory, self).__init__(*args, **kwargs)

    @property
    def target_trajectory(self):
        return self.camera_trajectory


    @target_trajectory.setter
    def target_trajectory(self, value):       
        self.camera_trajectory = value
        
