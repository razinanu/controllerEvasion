#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from fysom import Fysom


fsm = Fysom()

class FSM(Fysom):
    def __init__(self, *args, **kwargs):
        fsm_config = {'initial': 'right',
                      'events': [
                          ('lane_change', 'right', 'firstLaneChange'),
                          ('back_steer_left', 'firstLaneChange', 'secondLaneChange'),
                          ('complete_left', 'secondLaneChange', 'left'),
                          ('merging', 'left', 'firstMerging'),
                          ('back_steer_right', 'firstMerging', 'secondMerging'),
                          ('complete_right', 'secondMerging', 'right'),
                          ]
                      }
        super(FSM, self).__init__(fsm_config, *args, **kwargs)

if __name__ == '__main__':
    FSM()