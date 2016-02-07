#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from fysom import Fysom


fsm = Fysom()

class FSM(Fysom):
    def __init__(self, *args, **kwargs):
        fsm_config = {'initial': 'right',
                      'events': [
                          ('lane_change', 'right', 'first_lane_change'),
                          ('back_steer_left', 'first_lane_change', 'second_lane_change'),
                          ('complete_left', 'second_lane_change', 'left'),
                          ('merging', 'left', 'first_merging'),
                          ('back_steer_right', 'first_merging', 'second_merging'),
                          ('complete_right', 'second_merging', 'right'),

                          ]
                      }
        super(FSM, self).__init__(fsm_config, *args, **kwargs)
        # print self.isstate("right")
        # print self.current

if __name__ == '__main__':
    FSM()