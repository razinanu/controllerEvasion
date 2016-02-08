#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from components import Control,Trajectory,CurrentYawRate,CurrentSpeed,TargetTrajectory,SpeedControl
from common import Lockable,Reconfigurable
import ControllerConfig

class EvadingControllerLane(Reconfigurable,Control,Trajectory,CurrentYawRate,CurrentSpeed,TargetTrajectory, SpeedControl):
    def __init__(self,*args,**kwargs):
        self.params_changed_callbacks = list()
        super(EvadingControllerLane,self).__init__(ControllerConfig,"config.json",*args,**kwargs)
        self.params_changed()

    def params_changed(self): # called by Reconfigurable
        super(EvadingControllerLane, self).params_changed()
        for callback in self.params_changed_callbacks:
            callback(self.config)

    def notice_trajectory(self):
        pass