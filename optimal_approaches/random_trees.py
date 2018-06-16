#!/usr/bin/env python
import numpy as np
from utils.plot_simulator import plot_movement
from utils.plot_simulator import pausing_plot


class RandomTrees(object):
    def __init__(self, world_state, obstacles, goal_point, robot_init, pause_sec):
        self._world_state = world_state
        self._obstacles = obstacles
        self._goal_point = goal_point
        self._robot_init = robot_init
        self._final_path = []
        
        print(self._world_state)