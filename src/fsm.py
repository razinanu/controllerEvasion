#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from fysom import Fysom

fsm = Fysom()

class FSM(Fysom):
    def __init__(self, *args, **kwargs):
        fsm_config = {'initial': 'right',
                      'events': [
                          ('obstacle','right','right_to_left'),
                          ('lane_change_complete', 'right_to_left', 'left'),
                          ('obstacle_passed', 'left', 'left_to_right'),
                          ('obstacle_passed', 'right_to_left', 'left_to_right'),
                          ('lane_change_complete', 'left_to_right', 'right')
                          ]
                      }
        super(FSM, self).__init__(fsm_config, *args, **kwargs)
        self.obstacle()
        self.lane_change_complete()
        self.obstacle_passed()
        self.lane_change_complete()

    def onright_to_left(self, e):
        print e.event, e.src, e.dst
    def onleft_to_right(self, e):
        print e.event, e.src, e.dst
    def onleft(self, e):
        print e.event, e.src, e.dst
    def onright(self, e):
        print e.event, e.src, e.dst
    def onobstacle(self, e):
        print e.event, e.src, e.dst
    def onlane_change_complete(self, e):
        print e.event, e.src, e.dst
    def onobstacle_passed(self, e):
        print e.event, e.src, e.dst

if __name__ == '__main__':
    FSM()