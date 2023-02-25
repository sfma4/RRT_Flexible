#!/usr/bin/env python


import sys
import time
# import rospy
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict
from configuration_space import FreeEuclideanSpace, FlexibleLocalPlanner, Plan
from tqdm import tqdm
import simulation_function_double_link as simu
# import simulation_function_3 as simu_forward
import csv
import pandas as pd

class RRTGraph(object):

    def __init__(self, *nodes):
        self.nodes = [n for n in nodes]
        self.parent = defaultdict(lambda: None)
        self.path = defaultdict(lambda: None)
        self.id = defaultdict(lambda: None)
        self.x_y = defaultdict(lambda: None)
        self.joints = defaultdict(lambda: None)
        self.result = defaultdict(lambda: None)
    def add_node(self, new_config, parent, path, result, x_y, theta):
        new_config = tuple(new_config)
        parent = tuple(parent)
        # print('parent', parent)
        self.nodes.append(new_config)
        self.parent[new_config] = parent
        self.path[(parent, new_config)] = path
        self.id[new_config] = len(self.nodes)
        self.result[new_config] = result
        self.x_y[new_config] = x_y
        self.joints[new_config] = theta

    def delete_node(self, config):
        config = tuple(config)
        self.nodes.remove(config)

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
            print('cobj in construction',cObj)
        cArray.reverse()
        path = None
        # for i in range(len(cArray) - 1):
        for i in range((len(cArray) - 1), 0, -1):
            print(i)
            # break
            # print("path2",self.path[(cArray[i-1], cArray[i])])
            path = Plan.chain_paths(path, self.path[(cArray[i-1], cArray[i])])
        # print('path at construct', path)
        cArray.reverse()
        print('cArray', cArray)
        p = np.array([[0, 0]])
        for i in range(len(cArray) - 1):
            # path = np.arange(cArray[i][0], cArray[i + 1][0] + 0.1, 0.0042)  ## why 0.1
            if cArray[i][0] < cArray[i + 1][0]:
                path = np.arange(cArray[i][0], cArray[i + 1][0], 0.0042)
            else:
                path = np.arange(cArray[i][0], cArray[i + 1][0], -0.0042)

            num = len(path)
            path = path.reshape(path.shape[0], 1)
            path1 = np.linspace(cArray[i][1], cArray[i + 1][1], num)
            path1 = path1.reshape(path.shape[0], 1)
            b = np.concatenate((path, path1), axis=1)
            p = np.concatenate((p, b))
        p = np.delete(p, 0, 0)
        return p


class RRTPlanner(object):

    def __init__(self, config_space, max_iter=100, expand_dist=0.3):
        # config_space should be an object of type ConfigurationSpace
        # (or a subclass of ConfigurationSpace).
        self.config_space = config_space
        # Maximum number of iterations to run RRT for:
        self.max_iter = max_iter
        # Exit the algorithm once a node is sampled within this
        # distance of the goal:
        self.expand_dist = expand_dist
        # self.new_goal = []
    # def sample_new_goal(goal):
    def path_exist(self, a, b):
        return True

    def iterate_new_goal(self, goal):
        ### this function is used for sample contacted states and pseudo goal which are not in contact
        probability_of_sampling_goal = 0.4   ### for 30% of chance, X(p) will be sampled outside of C reigion
        p = np.random.uniform(0, 1)
        scale = np.random.rand(1, 4)[0]
        # zoom_radius = min(Distance_goal)
        if p < probability_of_sampling_goal:  ## sample for pseudo goal
            radius = 1.0  #the radius of the sample range circle, assume that is step length
            circle_x = goal[0]
            circle_y = goal[1]
            alpha = 2 * np.pi * np.random.rand() # ramdom angle
            r = radius * np.sqrt(np.random.rand())  # random radius
            # x = r * np.cos(alpha) + circle_x
            # y = r * np.sin(alpha) + circle_y
            x = r * np.cos(alpha) + 5
            y = r * np.sin(alpha) + 6

            sample = [x, y, 0, 0]
            new_distance = np.sqrt((sample[0] - goal[0]) ** 2 + (sample[1] - goal[1]) ** 2)
            # print('random sample near goal', sample)
            # print('updated radius', radius)
            sample_type = 0 ### so this goal need to be check with check_collision_c
        else:   ## need to change the sampling area so only sample within C reigion
            sample = [1.5*np.random.uniform(0, 1)+5, 0.5*np.random.uniform(0, 1)+5.5, 0, 0]  ## just randomly sample within C reigion TODO
            sample_type = 1
            print('sample generated randomly',sample)
        return sample, sample_type

    def iterate_new_final_goal(self, goal):
        ### this function take the given goal from the main function,
        # the calculate the possible actuated states for goal

        r = 1.0 ## since robot length is 1.0
        alpha = np.pi * np.random.rand() - np.pi/2
        print('alpha of goal_ac', alpha)
        final_new_goal = goal
        final_new_goal[0] = goal[0] + r * np.sin(alpha)
        final_new_goal[1] = goal[1] + r * np.cos(alpha) - 0.5 * np.random.rand()
        print('x of goal_ac', final_new_goal[0])
        print('y of goal_ac', final_new_goal[1] )

        return final_new_goal
    def get_alpha(self, goal):    ### to do
        height = goal[1] - 5.0
        alpha = (1.2 - (-1.2)) * np.random.random((1,1)) - 1.2  ## generate a random number in [-1.2, 1.2]
        height = (height - 0.5*np.cos(alpha))[0][0]
        height = np.clip(height, -0.5, 0.5)
        print('height', height)
        # if height > 1.0:
        #     height = 0.5
        beta = np.arccos(height/0.5)
        a = np.array([goal[0], goal[1],alpha[0][0], beta])
        b = np.array([goal[0], goal[1],alpha[0][0], -beta])
        print('contact states sampled', a, b)
        return a, b

    def plan_to_pose(self, start, goal, dt=0.01, prefix_time_length=1):
        """
            Uses the RRT algorithm to plan from the start configuration
            to the goal configuration.
        """
        print("======= Planning with RRT =======")

        EE_goal = goal + np.array([0.2, -0.1, 0, 0])
        c_region = self.config_space.find_C_reigion(l=1.0)  ## assume length of gripper is 1.0
        print("c_region", c_region)
        # if self.config_space.check_collision_c(goal, c_region):   ### check if the goal is within C region
        global new_goal
        global new_goal_p
        global goal_orien
        tol = 0.1 ## global tolerance factor
        goal_orien = goal[2]
        Tsimu_start = time.time()
        # goal_a = np.array([6.6, 5.6, 0.8, 0.2])
        goal_a = np.array([6.6, 5.6, 0.8, 0.2])

        self.graph = RRTGraph(tuple(goal_a))   ### very important change, in normal rrt, it is self.graph = RRTGraph(start),
        ## so the start position is added to nodes. Here we add goal nodes
        self.graph.id[tuple(goal_a)] = 1
        self.plan = None
        t_start = time.time()
        for i in range(40):
            new_goal, sample_type = self.iterate_new_goal(goal)
            # new_goal_p = np.array([5.8, 6.2, 0.2, 0.1])
            ax = plt.subplot(1, 1, 1)

            # if sample_type == 0 and self.config_space.check_collision_c(new_goal, c_region):
            #     continue
            if sample_type == 0 :
                print("sample for pseudo goal")
                x = (7.0 - (3.0)) * np.random.random((1,1)) + 3.0
                y = (7.0 - (6.0)) * np.random.random((1,1))  + 6.0

                # new_goal_p = np.array([x[0][0], 6.2, 0.2, 0.1])  ## 0.2 and 0.1 is the stead state of joints
                new_goal_p = np.array([x[0][0], y[0][0], 0.0, 0.0]) ##now we try without the help of steady state

                closest_config_p = self.config_space.nearest_config_to(self.graph.nodes, new_goal_p)
                # end_pos_closest = [closest_config_p[0]+1.0*np.sin(closest_config_p[2]), closest_config_p[1]-1.0*np.cos(closest_config_p[2])]
                end_pos_closest = [closest_config_p[0]+0.5*np.sin(closest_config_p[2]) + 0.5*np.sin(closest_config_p[3]),
                                   closest_config_p[1]-0.5*np.cos(closest_config_p[3]) -0.5*np.cos(closest_config_p[3])]

                print('closes config to pseudo goal', closest_config_p)
                path_p = self.config_space.local_plan(new_goal_p, closest_config_p)
                end_pos_p, x_y, theta = simu.simulation_try(path_p.positions[:, :4])
                # if np.allclose(end_pos_p[:2], end_pos_closest, rtol=tol, atol=tol) \
                # and theta[0]*closest_config_p[2] > 0 and theta[1]*closest_config_p[3] > 0:
                # if theta[0]*closest_config_p[2] > 0 and theta[1]*closest_config_p[3] > 0:
                if np.allclose(end_pos_p[:2], end_pos_closest, rtol=tol, atol=tol) \
                        and theta[0]*closest_config_p[2] > 0 and theta[1]*closest_config_p[3] > 0:
                    print('end_pos', end_pos_p[:2], 'local goal', end_pos_closest[:2])
                    print('if simulation result is close enough to desired',
                          np.allclose(end_pos_p[:2], closest_config_p[:2], rtol=tol, atol=tol))
                    print('iteration number of new goal', i)
                    self.graph.add_node(new_goal_p, closest_config_p, path_p, end_pos_p, x_y, theta)
                    self.plan = self.graph.construct_path_to(new_goal_p)
                    time_d = time.time()
                    T_d = time_d - t_start

                    print('path to pseudo goal from actual goal found, iteration finish')
                    print('final plan', self.plan)
                    path_forward = np.concatenate((path_p.positions[:, :2], self.plan[:, :2]))

                    # path_forward = path_p.append(self.plan[:, :2])
                    print('final plan forward', list(path_forward))
                    print('======= Forward Simulation =======')
                    # end_pos_p_2, x_y_2, theta = simu.simulation_try(path_forward)
                    end_pos_p_2, x_y_2, theta = simu.simulation_try(self.plan)
                    if np.allclose(end_pos_p_2[:2], [7.4, 5.1], rtol=0.05, atol=0.05):

                        print('forward simulation', end_pos_p_2, x_y_2 )
                        print( 'planning time', T_d, 'iteration number', i)

                        print(self.graph.path[self.graph.nodes[0], self.graph.nodes[1]])
                        nodes = self.graph.nodes
                        parent = [('none')]
                        result = [('none')]  ## because for 1st point, there is no parent or result
                        for i in range(len(nodes)-1):
                            p = self.graph.parent[self.graph.nodes[i+1]]
                            parent.append(p)
                            r = self.graph.result[self.graph.nodes[i+1]]
                            result.append(r)

                        print('nodes', nodes)
                        print('parent', parent)
                        print('result', result)
                        # dict = {'nodes': nodes, 'parent': parent, 'result':result}
                        df = pd.concat([pd.DataFrame({'nodes': nodes}),pd.DataFrame({'parent': parent}),pd.DataFrame({'result': result})], axis=1)
                        timenow = time.strftime("%Y%m%d-%H%M%S")
                        df.to_csv(timenow + ' result.csv')
                        return self.plan
                    else:
                        print('======= Forward Simulation Result Failed =======')
                        self.graph.delete_node(new_goal_p)

            if sample_type == 1:
                sample_1, sample_2 = self.get_alpha(new_goal)
                print('pseudo goal contacted', sample_1, sample_2)
                # new_goal= np.array([6.471056748798914, 6.535613437266156, 0.0, 0.21362598277847444])
                # goal_a  = self.iterate_new_final_goal(np.array([7.5, 5.1, 1.3694384, 0]))  ## actuated goal
                print("final new goal", goal_a)
                print(sample_1, goal_a)
                closest_config_1 = self.config_space.nearest_config_to(self.graph.nodes, sample_1) #found the nearest point from the graph.nodes
                closest_config_2 = self.config_space.nearest_config_to(self.graph.nodes, sample_2)
                print('closes config to sample_1', closest_config_1, sample_1, sample_2)
                print('closes config to sample_2', closest_config_2)
                path_final_1 = self.config_space.local_plan(sample_1, closest_config_1)
                path_final_2 = self.config_space.local_plan(sample_2, closest_config_2)
                # path_final_1 = path_final_1.get_prefix(prefix_time_length)
                # path_final_2 = path_final_2.get_prefix(prefix_time_length)
                if closest_config_1[2]*sample_1[2] >= 0 and closest_config_1[3]*sample_1[3] >= 0:
                    # print('path_final_1.positions[:, :3]', path_final_1.positions[:, :3])
                    end_pos_1, x_y, theta = simu.simulation_try(path_final_1.positions[:, :4])
                    end_pos_closest_1 = [closest_config_1[0] + 1.0 * np.sin(closest_config_1[2]),
                                       closest_config_1[1] - 1.0 * np.cos(closest_config_1[2])]

                    if np.allclose(end_pos_1[:2], end_pos_closest_1, rtol=tol, atol=tol):
                        print('end_pos', end_pos_1[:2], 'local goal', end_pos_closest_1[:2])
                        print('if simulation result is close enough to desired',
                              np.allclose(end_pos_1[:2], end_pos_closest_1[:2], rtol=tol, atol=tol))
                        print('iteration number of new goal', i)
                        # ax.plot([end_pos_1[0], x_y[0]], [end_pos_1[1], x_y[1]], color='blue', linewidth='3', alpha=0.5)
                        # print('adding nodes',sample_1, end_pos_1, closest_config_1, path_final_1)

                        self.graph.add_node(sample_1, closest_config_1, path_final_1, end_pos_1, x_y, theta)
                if closest_config_2[2] * sample_2[2] > 0 and closest_config_2[3]*sample_2[3] >= 0:
                    # print('path_final_2.positions[:, :3]', path_final_2.positions[:, :3])

                    end_pos_2, x_y, theta = simu.simulation_try(path_final_2.positions[:, :4])
                    end_pos_closest_2 = [closest_config_2[0] + 1.0 * np.sin(closest_config_2[2]),
                                     closest_config_2[1] - 1.0 * np.cos(closest_config_2[2])]
                    if np.allclose(end_pos_2[:2], closest_config_2[:2], rtol=0.1, atol=0.1):
                        print('end_pos', end_pos_1[:2], 'local goal', closest_config_2[:2])
                        print('if simulation result is close enough to desired',
                              np.allclose(end_pos_2[:2], end_pos_closest_2[:2], rtol=0.1, atol=0.1))
                        print('iteration number of new goal', i)
                        # ax.plot([end_pos_2[0], x_y[0]], [end_pos_2[1], x_y[1]], color='blue', linewidth='3', alpha=0.5)
                        print('adding nodes',sample_2, end_pos_2, goal_a)
                        self.graph.add_node(sample_2 , closest_config_2, path_final_2, end_pos_2, x_y, theta)
                        ### TODO TODO TODO TODO



        print("Failed to find plan in allotted number of iterations.")
        return None

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
        plt.plot([7.5,7.5,8.5,8.5,7.5],[5,6,6,5,5], color='r')
        # plt.plot([6.5,6.5,7.5,7.5,6.5],[6,7,7,6,6], color='r')  ## this is the plot of the upper smaller cube obstacle
        for path in self.graph.get_edge_paths():
            xs = path.positions[:, 0]
            ys = path.positions[:, 1]
            ax.plot(xs, ys, color='orange')

        new_x = new_goal_p[0]
        new_y = new_goal_p[1]
        ee_x = new_x + np.sin(new_goal[2]) * 1.0
        ee_y = new_y - np.cos(new_goal[2]) * 1.0
        # ax.plot(ee_x, ee_y, color='blue', marker='o', markersize=3)

        # goal_orien = goal[2]
        # goal_pos_x = self.plan.positions[-1][0] + np.sin(goal_orien) * 1.0
        # goal_pos_y = self.plan.positions[-1][1] - np.cos(goal_orien) * 1.0
        ax.plot(new_x, new_y, color='yellow', marker='o',markersize=5)
        ax.plot(7.5, 5.1, color='blue', marker='^', markersize=5)
        # gripper_tip = [self.plan.positions[:, 0], self.config_space.gripper_deflection(self.plan.positions[:, 1])]
        # if self.plan.positions:
            # plan_x = self.plan.positions[:, 0]
            # plan_y = self.plan.positions[:, 1]
        try:
            plan_x = self.plan[:, 0]
            plan_y = self.plan[:, 1]
        # angle = self.plan.positions[:,2]
        # for i in range(len(plan_x)):
        #     x_i = plan_x[i]+np.sin(angle[i])*0.2
        # x_angle = plan_x + np.cos(angle)*0.2
        # y_angle = plan_y + np.sin(angle) * 0.2
            print(len(plan_y), "length of the time steps")
            ax.plot(plan_x, plan_y, color='green')
        except:
            print('no plan')
        else:
            print('plan plotted')
        # ax.plot(x_angle, y_angle, color='blue',marker='o',markersize=1)

        # ax.plot(plan_x, plan_y, gripper_tip, color='green')


        plt.show()


def check_EE_state(a, b, c):

    return a + np.array([0.2, 0.1, 0, 0])

def find_angular_acc(x_acc, y_acc, theta_init):  ### basically a non-inverted pendulum attached to a cart moving in 2D space

    ### initial theta(underactuated join value) is 0 as assumed
    F_x, F_y = 50 ### assume max reaction force is 50 N
    g = 9.81
    theta = theta_init
    l = 0.5 ## l is the length of pendulum
    theta_acc = (3/(2*l))*(np.cos(theta)*x_acc + np.sin(theta)*g + np.sin(theta)*y_acc)
    return theta_acc


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

    T0_start = time.time()
    start = np.array([2, 2, 0, 0])
    goal = np.array([7.5, 5.1, 1.3694384, 0])

    xy_low = [0, 0]
    xy_high = [10, 10]
    phi_max = 0.6
    u1_max = 2
    u2_max = 3
    obstacles = []
    obstacle_area1 = [[5, 0], [5, 5], [10, 0], [10, 5]]   ### rectangles 1 left-bottom 2 left-up 3 right bottom 4 right-up
    config = FlexibleLocalPlanner(xy_low + [-1*np.pi, -phi_max],
                                       xy_high + [1*np.pi, phi_max],
                                       [-u1_max, -u2_max],
                                       [u1_max, u2_max],
                                       obstacle_area1,
                                       0.15)

    planner = RRTPlanner(config, max_iter=500, expand_dist=0.2)
    plan = planner.plan_to_pose(start, goal, prefix_time_length=1.2)
    # plot_new = self.new_goal
    T2_end = time.time()
    T2= T2_end - T0_start
    print("==t1===", T2)
    planner.plot_execution()

# def test_runing():
#     for i in range(100):
#         main()
# if __name__ == '__main__':
#     test_runing()

if __name__ == '__main__':
    main()