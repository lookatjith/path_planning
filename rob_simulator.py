#!/usr/bin/env python
from matplotlib import pyplot as plt
from matplotlib import *
import numpy as np

class RobSimulator(object):
    def __init__(self, approach):
        # world coordinates
        self._world_state = (0, 0), (0, 1), (0, 2), (0, 3), (0, 4), (0, 5), \
        (1, 0), (1, 1), (1, 2), (1, 3), (1, 4), (1, 5), \
        (2, 0), (2, 1), (2, 2), (2, 3), (2, 4), (2, 5), \
        (3, 0), (3, 1), (3, 2), (3, 3), (3, 4), (3, 5), \
        (4, 0), (4, 1), (4, 2), (4, 3), (4, 4), (4, 5), \
        (5, 0), (5, 1), (5, 2), (5, 3), (5, 4), (5, 5)  
        # obstacles
        self._obstacles = [(2, 0), (2, 1), (3, 3), (4, 2), (2, 3), (4, 3), (4, 4)]
        # goal point
        self._goal_point = (5, 5)
        # robot init
        self._robot_init = (1, 0)
        # step size for reaching the goal
        self._max_step_number = 1000
        # plotsimulation
        plt.ion()
        # plot to pause in between        
        self._pause_sec = 1
        # final path useful for optimal approach
        self._final_path = []

        # approach to use
        print(approach)
        if approach == "random":
            print(approach)
            self.run_random_approach()
        

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



    def run_random_approach(self):
        self.plot_movement(self._world_state, self._obstacles, self._goal_point, self._robot_init)
        self.random_planner(x=self._robot_init[0], y=self._robot_init[1],
                       new_x=self._robot_init[0], new_y=self._robot_init[1])


    def run_optimal_approach(self):
        """ optimal_planner """
        final_path = optimal_planner()
            
        for path in final_path:
            plot_movement(world_state, obstacles, goal_point, path)
            plt.pause(pause_sec)

    #run_random_approach()
    #run_optimal_approach()

if __name__ == '__main__':
    # get the approach name from the user
    import argparse
    parser = argparse.ArgumentParser("parsing the arguments")
    parser.add_argument("approach", help="An approach to pass by")
    args = parser.parse_args()

    rob_simulator = RobSimulator(args.approach)
    rob_simulator
