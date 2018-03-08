#!/usr/bin/env python
import numpy as np
from utils.plot_simulator import plot_movement
from utils.plot_simulator import pausing_plot

class AStar(object):
    """
    Method uses A star approach to find the optimal way to reach the
    goal point from the initial position

    Parameters
    ----------
    world_state: list of <tuple>
        Environment coordinates
    obstacles: list of <tuple>
        Not traversing path
    goal_point: <tuple>
        pos of the robot to be achieved by the algorithm
    robot_init: <tuple>
        starting pos of the robot 
    pause_sec: <int>
        pausing time for ploting
    """
    def __init__(self, world_state, obstacles, goal_point, robot_init, pause_sec):
        self._world_state = world_state
        self._obstacles = obstacles
        self._goal_point = goal_point
        self._robot_init = robot_init
        self._final_path = []
        
        self._final_path = self.a_star()
        for path in self._final_path:
            plot_movement(self._world_state, self._obstacles, self._goal_point, path)
            pausing_plot(pause_sec)            

    def a_star(self):
        open_path = {}
        closed_path = {}

        f = 0
        d = 0

        open_path[self._robot_init] = f
        successors = []

        while open_path is not bool(open_path):
            # get the minimum value of minimum key
            min_val = min(open_path.itervalues())
            min_keys = [key for key, value in open_path.iteritems()
                        if value == min_val]
            pos = min_keys[0]
            # for key in min_keys:
            open_path = {}
            #dist_start = open_path[pos]
            dist_start = min_val
            # generate the successors
            successors = [(pos[0], pos[1] + 1), (pos[0] + 1, pos[1]),
                          (pos[0], pos[1] - 1), (pos[0] - 1, pos[1])]
            for successor in successors:
                #### checking starts #####
                # check for world state
                if successor in self._world_state:
                    if successor not in self._obstacles:
                        if successor != self._goal_point:
                            # f(n) = g(n) + h(n)
                            # calculate h(n)
                            dist_to_goal = np.sqrt(np.square(
                                self._goal_point[0] - successor[0]) + np.square(self._goal_point[1] - successor[1]))
                            # calculate g(n)
                            dist = dist_start + \
                                np.sqrt(
                                    np.square(successor[0] - pos[0]) + np.square(successor[1] - pos[1]))
                            cost = dist_to_goal + dist
                            # check open list cost in successor
                            if successor not in closed_path.values():
                                if successor not in open_path or cost < open_path[successor]:
                                    open_path[successor] = cost
                                elif successor in closed_path and closed_path[successor] > cost:
                                    open_path[successor] = cost
                        else:
                            closed_path[cost] = pos
                            self._final_path.append(pos)
                            return self._final_path

            closed_path[cost] = pos
            self._final_path.append(pos)