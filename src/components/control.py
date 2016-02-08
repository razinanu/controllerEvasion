#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from controller_lane.components import Control as ControlBase

class Control(ControlBase):
    def __init__(self, *args, **kwargs):
        super(Control, self).__init__(*args, **kwargs)


    def control(self):
        if self.target_trajectory is None: return 0, None
        return super(Control,self).control()
