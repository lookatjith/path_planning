#!/usr/bin/env python
from matplotlib import pyplot as plt

def plot_movement(world_state, obstacles, goal_point, robot_pos):
    """ Plot environment including robot and obstacles """
    plt.cla()
    x_min, x_max = min(world_state)[0], max(world_state)[0]
    y_min, y_max = min(world_state)[1], max(world_state)[1]

    axes = plt.gca()
    axes.set_xlim([x_min - 1, x_max + 1])
    axes.set_ylim([y_max + 1, y_min - 1])
    plt.grid()

    for x, y in obstacles:
        plt.plot(x, y, 'sr', ls='solid', markersize=40)

    plt.plot(robot_pos[0], robot_pos[1], 'b*', markersize=10)
    plt.plot(goal_point[0], goal_point[1], 'go', markersize=10)
    plt.show()

def pausing_plot(sec):
    plt.pause(sec)