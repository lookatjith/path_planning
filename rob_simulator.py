#!/usr/bin/env python
from random_approach.random_planner import RandomPlanner
from matplotlib import pyplot as plt
from matplotlib import *
import numpy as np


class RobSimulator(object):
    def __init__(self, approach):
        # world coordinates
        world_state = (0, 0), (0, 1), (0, 2), (0, 3), (0, 4), (0, 5), \
            (1, 0), (1, 1), (1, 2), (1, 3), (1, 4), (1, 5), \
            (2, 0), (2, 1), (2, 2), (2, 3), (2, 4), (2, 5), \
            (3, 0), (3, 1), (3, 2), (3, 3), (3, 4), (3, 5), \
            (4, 0), (4, 1), (4, 2), (4, 3), (4, 4), (4, 5), \
            (5, 0), (5, 1), (5, 2), (5, 3), (5, 4), (5, 5)
        # obstacles
        obstacles = [(2, 0), (2, 1), (3, 3), (4, 2), (2, 3), (4, 3), (4, 4)]
        # goal point
        goal_point = (1, 5)
        # robot init
        robot_init = (1, 0)
        # step size for reaching the goal
        max_step_number = 1000
        # plotsimulation
        plt.ion()
        # plot to pause in between
        pause_sec = 1
        # final path useful for optimal approach
        final_path = []

        if approach == "random":
            random_planner = RandomPlanner(
                max_step_number, world_state, obstacles, goal_point, robot_init, pause_sec)
            random_planner.random_planner(
                x=robot_init[0], y=robot_init[1], new_x=robot_init[0], new_y=robot_init[1])
        elif approach == "a_star":
            a_star = AStar(max_step_number, world_state, obstacles, goal_point, robot_init, pause_sec)
            a_star.a_star(final_path)
            final_path = optimal_planner()

    def optimal_planner():
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

        open_path[robot_init] = f
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
                if successor in world_state:
                    if successor not in obstacles:
                        if successor != goal_point:
                            # f(n) = g(n) + h(n)
                            # calculate h(n)
                            dist_to_goal = np.sqrt(np.square(
                                goal_point[0] - successor[0]) + np.square(goal_point[1] - successor[1]))
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
                            final_path.append(pos)
                            return final_path

            closed_path[cost] = pos
            final_path.append(pos)



    # run_random_approach()
    # run_optimal_approach()


if __name__ == '__main__':
    # get the approach name from the user
    import argparse
    parser = argparse.ArgumentParser("parsing the arguments")
    parser.add_argument("approach", help="An approach to pass by")
    args = parser.parse_args()

    rob_simulator = RobSimulator(args.approach)
    rob_simulator
