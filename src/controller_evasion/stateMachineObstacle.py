#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from fysom import Fysom

fsm = Fysom()


class stateMachineObstacle(Fysom):
    def __init__(self, *args, **kwargs):
        fsm_config = {'initial': 'free',
                      'events': [
                          ('obstacle', 'free', 'firstObstacle'),
                          ('test', 'firstObstacle', 'testObstacle'),
                          ('confirm', 'testObstacle', 'confirmedObstacle'),
                          ('reject', 'testObstacle', 'free'),
                          ('searchNext', 'firstObstacle', 'free'),
                          ('finish', 'confirmedObstacle', 'free')
                          ]
                      }
        super(stateMachineObstacle, self).__init__(fsm_config, *args, **kwargs)

if __name__ == '__main__':
    stateMachineObstacle()