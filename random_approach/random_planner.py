#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt

class RandomPlanner(object):
    def __init__(self, max_step_number, world_state, obstacles, goal_point, robot_init, pause_sec):
        self._max_step_number = max_step_number
        self._world_state = world_state
        self._obstacles = obstacles
        self._goal_point = goal_point
        self._robot_init = robot_init
        self._pause_sec = pause_sec

        self.plot_movement(self._world_state, self._obstacles, self._goal_point, self._robot_init)
            
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

    def random_planner(self, x, y, new_x, new_y):
        """
        A random planner implementation where the robot tries to achieve
        the goal point with in a max_step_number. The robot finds the new 
        pose by randomly incrementing/decrementing the current pose at only
        one direction at a time

        Complexity:
        ----------
        1) Robot can visit already covered pose multiple times
        2) Robot can get stuck at one point or between obstacles
        3) Robot can get diverged even after nearing the goal point
        3) Robot search is completely random. The robot may take long time
        to reach the goal point. If the maximum number of step is small, the
        robot may fail to achieve the goal point
        4) If the complexity of the robot environment increase, the robot
        may fail to reach the goal point.
        """
        new_x = x
        new_y = y

        def do_increment(val):
            new_val = val + np.random.randint(-1, 2)
            if new_val != val:
                return True, new_val
            else:
                return False, new_val

        for i in range(self._max_step_number):
            x = new_x
            y = new_y
            incremented, new_x = do_increment(x)
            if not incremented:
                incremented, new_y = do_increment(y)
            # new estimated pose lies with the world
            if (new_x, new_y) not in self._world_state:
                new_x = x
                new_y = y
            # obstacle avoidance
            elif (new_x, new_y) in self._obstacles:
                new_x = x
                new_y = y
            # goal point reached
            elif new_x == self._goal_point[0] and new_y == self._goal_point[1]:
                print("goal point reached in {} iterations".format(i))
                # return if goal point reached
                return
            new_pos = (new_x, new_y)
            self.plot_movement(self._world_state, self._obstacles, self._goal_point, new_pos)
            plt.pause(self._pause_sec)
        
        print("total iterations: {} reached. Robot not able to reach the goal point".format(
            self._max_step_number))
