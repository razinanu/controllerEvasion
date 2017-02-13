#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from fysom import Fysom

fsm = Fysom()


class StateMachineObstacle(Fysom):
    def __init__(self, *args, **kwargs):
        fsm_config = {'initial': 'free',
                      'events': [
                          ('obstacle', 'free', 'obstacle'),
                          ('finish', 'Obstacle', 'free')
                          ]
                      }
        super(StateMachineObstacle, self).__init__(fsm_config, *args, **kwargs)

if __name__ == '__main__':
    StateMachineObstacle()