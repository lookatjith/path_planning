#!/usr/bin/env python


class AStar(object):
    def __init__(self, world_state, obstacles, goal_point, robot_init, pause_sec):
        self._world_state = world_state
        self._obstacles = obstacles
        self._goal_point = goal_point
        self._robot_init = robot_init
        self._pause_sec = pause_sec






        for path in final_path:
            plot_movement(world_state, obstacles, goal_point, path)
            plt.pause(pause_sec)            

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
