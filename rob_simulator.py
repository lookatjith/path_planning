#!/usr/bin/env python
from random_approach.random_planner import RandomPlanner
from optimal_approaches.a_star import AStar
from matplotlib import pyplot as plt

class RobSimulator(object):
    """
    A simple robot simulator which simulates an 2D environment with obstacles.

    Parameters
    ----------
    Robot environment: 2D points
        list of <tuples>
    obstacles: 2D points inside the robot environment
        list of <tuples>
    goal point: 2D point for the robot to reach
        <tuple>
    robot_init: initial state of the robot
        <tuple>
    approach: 2 different approaches are covered
        1. Random approach
            random movement
        2. Optimal approaches
            1. A-star algorithm
    """
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
        goal_point = (5, 1)
        # robot init
        robot_init = (1, 0)
        # step size for reaching the goal
        max_step_number = 1000
        # plotsimulation
        plt.ion()
        # plot to pause in between
        pause_sec = 0.1

        if approach == "random":
            random_planner = RandomPlanner(
                max_step_number, world_state, obstacles, goal_point, robot_init, pause_sec)
            random_planner.random_planner(
                x=robot_init[0], y=robot_init[1])
        elif approach == "a_star":
            a_star = AStar(world_state, obstacles, goal_point, robot_init, pause_sec)
            

if __name__ == '__main__':
    # get the approach name from the user
    import argparse
    parser = argparse.ArgumentParser("parsing the arguments")
    parser.add_argument("approach", help="An approach to simulate the robot")
    args = parser.parse_args()

    rob_simulator = RobSimulator(args.approach)
    rob_simulator
