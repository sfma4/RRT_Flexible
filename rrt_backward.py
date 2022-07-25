#!/usr/bin/env python


import sys
import time
# import rospy
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict
from configuration_space import FreeEuclideanSpace, FlexibleLocalPlanner, Plan
from tqdm import tqdm
import simulation_function as simu

class RRTGraph(object):

    def __init__(self, *nodes):
        self.nodes = [n for n in nodes]
        self.parent = defaultdict(lambda: None)
        self.path = defaultdict(lambda: None)

    def add_node(self, new_config, parent, path):
        new_config = tuple(new_config)
        parent = tuple(parent)
        # print('parent', parent)
        self.nodes.append(new_config)
        self.parent[new_config] = parent
        self.path[(parent, new_config)] = path

    def get_edge_paths(self):
        for pair in self.path:
            yield self.path[pair]

    def construct_path_to(self, c):
        # c = tuple(c)
        # return Plan.chain_paths(self.construct_path_to(self.parent[c]), self.path[(self.parent[c], c)]) if self.parent[c] else None
        cObj = tuple(c)
        cArray = [cObj]
        while self.parent[cObj]:
            cObj = tuple(self.parent[cObj])
            cArray.append(cObj)
        cArray.reverse()
        path = None
        for i in range(len(cArray) - 1):
            path = Plan.chain_paths(path, self.path[(cArray[i], cArray[i + 1])])
        return path


class RRTPlanner(object):

    def __init__(self, config_space, max_iter=10000, expand_dist=0.3):
        # config_space should be an object of type ConfigurationSpace
        # (or a subclass of ConfigurationSpace).
        self.config_space = config_space
        # Maximum number of iterations to run RRT for:
        self.max_iter = max_iter
        # Exit the algorithm once a node is sampled within this
        # distance of the goal:
        self.expand_dist = expand_dist



    def plan_to_pose(self, start, goal, dt=0.01, prefix_time_length=1):
        """
            Uses the RRT algorithm to plan from the start configuration
            to the goal configuration.
        """
        print("======= Planning with RRT =======")
        EE_goal = goal + np.array([0.2, -0.1, 0, 0])
        self.graph = RRTGraph(start)
        patha = self.config_space.local_plan(start, np.array([3,2,0,0]))
        delta1 = patha.get_prefix(prefix_time_length)
        new1 = delta1.end_position()
        # self.graph.add_node(np.array([3,2,0,0]), start, delta1)
        self.graph.add_node(new1, start, delta1)
        pathb = self.config_space.local_plan(new1,np.array([4,3,0,0]))
        delta2 = pathb.get_prefix(prefix_time_length)
        new2 = delta2.end_position()
        # self.graph.add_node(np.array([4,3,0,0]),np.array([3,2,0,0]), delta2)
        self.graph.add_node(new2, new1, delta2)
        pathc = self.config_space.local_plan(new2, np.array([4.0, 3.5, 0, 0]))
        delta3 = pathc.get_prefix(prefix_time_length)
        new3 = delta3.end_position()
        self.graph.add_node(np.array([4.0, 3.5, 0, 0]), new3, delta3)
        self.plan = None
        print("Iteration:", 0)
        for it in tqdm(range(self.max_iter)):
            # sys.stdout.write("\033[F")
            # print("Iteration:", it + 1)
            # if rospy.is_shutdown():
            #     print("Stopping path planner.")
            #     break
            rand_config = self.config_space.sample_config(goal)   #generate random point
            # using goal zoom/bias, this is from class Bicycle
            if self.config_space.check_collision(rand_config):  # check collision for respawn point
                continue #if true, 'continue' will go to the next loop of the for loop
            closest_config = self.config_space.nearest_config_to(self.graph.nodes, rand_config) #found the nearest point from the graph.nodes
            path = self.config_space.local_plan(closest_config, rand_config)  # use local planer to generate path
            if self.config_space.check_path_collision(path): #check the path collision
                continue
            delta_path = path.get_prefix(prefix_time_length) #get the new path under step length setting
            new_config = delta_path.end_position() #set the last point of that path above as the last point
            self.graph.add_node(new_config, closest_config, delta_path) #add point to the tree
            # if self.config_space.distance(new_config, goal) <= self.expand_dist: # check if it reach to goal
            #     path_to_goal = self.config_space.local_plan(new_config, goal)
            #     if self.config_space.check_path_collision(path_to_goal):
            #         continue
            #     # self.graph.add_node(goal, new_config, path_to_goal)
            #     # self.plan = self.graph.construct_path_to(goal)
            #     self.graph.add_node(goal, new_config, path_to_goal)
            #     print('all nodes', self.graph.nodes)
            #     self.plan = self.graph.construct_path_to(goal)
            #     return self.plan
            if self.config_space.distance(new_config, goal) <= self.expand_dist: # check if it reach to goal
                EE_state = check_EE_state(self.graph.nodes[-1],self.graph.nodes[-2],self.graph.nodes[-3],)
                if self.config_space.distance(EE_state, EE_goal) <= 0.02: # check if flexible end effector reach to its goal
                    print('distance', self.config_space.distance(EE_state, EE_goal))
                    path_to_goal = self.config_space.local_plan(new_config, goal)
                    if self.config_space.check_path_collision(path_to_goal):
                        continue
                    self.graph.add_node(goal, new_config, path_to_goal)
                    self.plan = self.graph.construct_path_to(goal)

                else:
                    print('distance after else', self.config_space.distance(EE_state, EE_goal))
                    print('distance to the real goal is larger than 0.05, which is', self.config_space.distance(EE_state, EE_goal))
                    new_goal = sample_fake_goal(self.graph.nodes[-1])
                    new_rand_config = new_goal  # for convenience this time
                    print('new_rand_config', new_rand_config)
                    if self.config_space.check_collision(new_rand_config):  # check collision for respawn point
                        continue  # if true, 'continue' will go to the next loop of the for loop
                    # closest_config_new = self.config_space.nearest_config_to(self.graph.nodes,
                    #                                                      new_rand_config)  # found the nearest point from the graph.nodes
                    path_new = self.config_space.local_plan(new_config,
                                                        new_rand_config)  # use local planer to generate path
                    if self.config_space.check_path_collision(path_new):  # check the path collision
                        continue
                    delta_path_new = path_new.get_prefix(prefix_time_length)  # get the new path under step length setting
                    new_config_new = delta_path_new.end_position()  # set the last point of that path above as the last point
                    # self.graph.nodes = self.graph.nodes[:-1]
                    self.graph.add_node(new_config_new, new_config, delta_path_new)
                    path_final = self.config_space.local_plan(new_config_new, goal)
                    if self.config_space.check_path_collision(path_final):
                        continue
                    self.graph.add_node(goal, new_config_new, path_final)
                    self.plan = self.graph.construct_path_to(goal)
                    simu.simulation_try(self.plan.positions[:, :2])
                    print("green plans", self.plan.positions)

                return self.plan
                    # new_goal = generate_new_goal(self.graph[-1],self.graph[-2],self.graph[-3], obstacle_area1)   # should also has obstacle

        print("Failed to find plan in allotted number of iterations.")
        return None

    # def obstacle_area(self,obstacle_point1,obstacle_point2,obstacle_point3,obstacle_point4):
    #     self.obstacle_point1 = obstacle_point1
    #     #   左下点
    #     self.obstacle_point2 = obstacle_point2
    #     #   左上点
    #     self.obstacle_point3 = obstacle_point3
    #     #   右下点
    #     self.obstacle_point4 = obstacle_point4
    #     #   右上点

    def plot_execution(self):
        """
        Creates a plot of the RRT graph on the environment. Assumes that the
        environment of the robot is in the x-y plane, and that the first two
        components in the state space are x and y position. Also assumes
        plan_to_pose has been called on this instance already, so that self.graph
        is populated. If planning was successful, then self.plan will be populated
        and it will be plotted as well.
        """
        ax = plt.subplot(1, 1, 1)
        ax.set_aspect(1)
        ax.set_xlim(self.config_space.low_lims[0], self.config_space.high_lims[0])
        ax.set_ylim(self.config_space.low_lims[1], self.config_space.high_lims[1])

        # for obs in self.config_space.obstacles:
        #     xc, yc, r = obs
        #     circle = plt.Circle((xc, yc), r, color='black')
        #     ax.add_artist(circle)
        plt.plot(
            [self.config_space.obstacles[0][0], self.config_space.obstacles[1][0],
             self.config_space.obstacles[3][0],
             self.config_space.obstacles[2][0], self.config_space.obstacles[0][0]],
            [self.config_space.obstacles[0][1], self.config_space.obstacles[1][1],
             self.config_space.obstacles[3][1],
             self.config_space.obstacles[2][1], self.config_space.obstacles[0][1]], color='r')  # 画处障碍物区域

        for path in self.graph.get_edge_paths():
            xs = path.positions[:, 0]
            ys = path.positions[:, 1]
            ax.plot(xs, ys, color='orange')
        # gripper_tip = [self.plan.positions[:, 0], self.config_space.gripper_deflection(self.plan.positions[:, 1])]
        if self.plan:
            plan_x = self.plan.positions[:, 0]
            plan_y = self.plan.positions[:, 1]
            print(len(plan_y), "length of the time steps")
            ax.plot(plan_x, plan_y, color='green')

            # ax.plot(plan_x, plan_y, gripper_tip, color='green')


        plt.show()


# def obstacle_area(obstacle_point1, obstacle_point2, obstacle_point3, obstacle_point4):
#     self.obstacle_point1 = obstacle_point1
#     #   左下点
#     self.obstacle_point2 = obstacle_point2
#     #   左上点
#     self.obstacle_point3 = obstacle_point3
#     #   右下点
#     self.obstacle_point4 = obstacle_point4
#     #   右上点
def check_EE_state(a, b, c):

    return a + np.array([0.2, 0.1, 0, 0])

def sample_fake_goal(state):
    centre_x = state[0]
    centre_y = state[1]
    l = 0.5   # hardcode l = 0.5, which is the length of the gripper
    r = l + 0.2 * np.sqrt(np.random.rand())  # make radius of the sampling range slightly larger than length
    delta = np.pi * np.random.uniform(-0.5, 0.5)  # only sample the upper circle
    x = r * np.cos(delta) + centre_x
    y = r * np.sin(delta) + centre_y
    fake_goal = [x, y, state[2], 0]  ## keep the orientation the same for the time being
    return fake_goal

def main():
    """Use this function if you'd like to test without ROS.

    If you're testing at home without ROS, you might want
    to get rid of the rospy.is_shutdown check in the main
    planner loop (and the corresponding rospy import).
    """
    start = np.array([2, 2, 0, 0])
    goal = np.array([5.2, 5.8, 3.14, 0])

    xy_low = [0, 0]
    xy_high = [10, 10]
    phi_max = 0.6
    u1_max = 2
    u2_max = 3
    obstacles = []
    obstacle_area1 = [[5, 0], [5, 5], [10, 0], [10, 5]]   ### rectangles 1 left-bottom 2 left-up 3 right bottom 4 right-up
    config = FlexibleLocalPlanner(xy_low + [-2*np.pi, -phi_max],
                                       xy_high + [2*np.pi, phi_max],
                                       [-u1_max, -u2_max],
                                       [u1_max, u2_max],
                                       obstacle_area1,
                                       0.15)

    planner = RRTPlanner(config, max_iter=500, expand_dist=0.2)
    plan = planner.plan_to_pose(start, goal, prefix_time_length=1.0)

    planner.plot_execution()
    print('path len:', len(plan.positions))
    print('final pose:', plan.positions[-1])
    # print('final path', Plan.open_loop_inputs())


if __name__ == '__main__':
    main()
