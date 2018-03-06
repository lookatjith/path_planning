#!/usr/bin/env python
import numpy as np

from matplotlib import pyplot as plt

class AStar(object):
    def __init__(self, world_state, obstacles, goal_point, robot_init, pause_sec):
        self._world_state = world_state
        self._obstacles = obstacles
        self._goal_point = goal_point
        self._robot_init = robot_init
        self._final_path = []
        
        self._final_path = self.optimal_planner()
        for path in self._final_path:
            self.plot_movement(self._world_state, self._obstacles, self._goal_point, path)
            plt.pause(pause_sec)            

    def optimal_planner(self):
        """
        Method uses A star approach to find the optimal way to reach the
        goal point from the starting point

        Advantages
        ----------
        1) compared to the random approach, optimal planner finds the path
        in a short time
        2) Does not get stuck in between obstacles
        """

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
                            print("goal point reached")
                            closed_path[cost] = pos
                            self._final_path.append(pos)
                            return self._final_path

            closed_path[cost] = pos
            self._final_path.append(pos)

    def plot_movement(self, world_state, obstacles, goal_point, robot_init):
        """ Plot environment including robot and obstacles """
        plt.cla()
        x_min, x_max = min(self._world_state)[0], max(self._world_state)[0]
        y_min, y_max = min(self._world_state)[1], max(self._world_state)[1]

        axes = plt.gca()
        axes.set_xlim([x_min - 1, x_max + 1])
        axes.set_ylim([y_max + 1, y_min - 1])
        plt.grid()

        for x, y in self._obstacles:
            plt.plot(x, y, 'sr', ls='solid', markersize=40)

        plt.plot(robot_init[0], robot_init[1], 'b*', markersize=10)
        plt.plot(goal_point[0], goal_point[1], 'go', markersize=10)
        plt.show()
