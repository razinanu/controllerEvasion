#!/usr/bin/env python2
# -*- coding: utf-8 -*-
from controller_lane.components import SpeedControl as SpeedControlBase

import numpy as np
from common import Utils

class SpeedControl(SpeedControlBase):
    def __init__(self, *args, **kwargs):
        super(SpeedControl, self).__init__(*args, **kwargs)

    def speed_control(self, solution):
        if solution is None:
            return self.sigmoid_v_min * self.v_max
        else:
            return self.get_sigmoid_velocity(solution)
