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
        # print('path at construct', path)
        return path


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

    def iterate_new_goal(self, goal, c):
        for i in range(200):
            # scale = np.random.rand(1, 4)[0]
            circle_x = goal[0]
            circle_y = goal[1]
            alpha = 3 * np.pi/8 * np.random.rand() + np.pi/4
            # alpha = np.pi/4
            r = np.sqrt(np.random.rand()) + 1   ### where is the c-region boundry and gripper radius
            x = r * np.cos(alpha) + circle_x + np.random.rand()
            y = r * np.sin(alpha) + circle_y
            # x_new = [x, y, np.random.uniform(self.config_space.low_lims[2], self.config_space.high_lims[2]), np.random.uniform(self.config_space.low_lims[3], self.config_space.high_lims[3])]

            x_new = [x, y, 0.0, np.random.uniform(self.config_space.low_lims[3], self.config_space.high_lims[3])]

            # x_new = [6.020781662675009, 6.637158245458579, 1.0, -0.08081289659681912]   ### fake code to get a steady state underactuated state (angle)
            if self.config_space.check_collision_c(x_new, c):
                continue## path between new_goal and goal is not None
            if self.path_exist(goal, x_new):
                return x_new
        print("failed to find new_goal outside of C reigion")
        return None

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
    def get_alpha(self, c):    ### to do
        alpha = 0.1
        return alpha

    def plan_to_pose(self, start, goal, dt=0.01, prefix_time_length=2.0):
        """
            Uses the RRT algorithm to plan from the start configuration
            to the goal configuration.
        """
        print("======= Planning with RRT =======")

        EE_goal = goal + np.array([0.2, -0.1, 0, 0])
        self.graph = RRTGraph(start)
        self.plan = None
        c_region = self.config_space.find_C_reigion(l=1.0)  ## assume length of gripper is 1.0
        print("c_region", c_region)
        if self.config_space.check_collision_c(goal, c_region):   ### check if the goal is within C region
            # global new_goal
            print("goal is within the collision region")
            global goal_orien
            goal_orien = goal[2]
            # for i in range(100):
                # new_goal = self.iterate_new_goal(goal, c_region)
            goal_a  = self.iterate_new_final_goal(np.array([6.5, 5.2, 1.3694384, 0]))  ## actuated goal
            print("final new goal", goal_a)
                # path_final = self.config_space.local_plan(new_goal, goal_a)
                # print('path final ', path_final.positions[:, :3])
                # end_pos = simu.simulation_try(path_final.positions[:, :3])
                # if np.allclose(end_pos[:2], goal[:2], rtol=0.05, atol=0.05):
                #     print('end_pos', end_pos[:2], 'goal', goal[:2])
                #     print('if simulation result is close enough to desired', np.allclose(end_pos[:2], goal[:2], rtol=0.05, atol=0.05))
                #     print('iteration number of new goal', i)
                #     break
            # print("new goal", new_goal)

            print("Iteration:", 0)  ### yes the goal is within C region
            for it in tqdm(range(self.max_iter)):

                rand_config = self.config_space.sample_config(goal_a)   #since it is the base line, there is no fake goal, so we just sample towards the goal
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

                if self.config_space.distance(new_config, goal_a) <= self.expand_dist: # check if it reach to goal

                    path_to_goal = self.config_space.local_plan(new_config, goal_a)
                    if self.config_space.check_path_collision(path_to_goal):
                        print("line 156 collision check")
                        continue
                    print('graph-1', len(self.graph.nodes), self.graph.nodes)

                    # self.graph.add_node(goal_a, new_config, path_final)
                    path_before = self.graph.construct_path_to(new_config)

                    # path_before = self.config_space.local_plan(start, new_config)
                    path_pos = path_before.positions   ### path information of the tree excluding the last one
                    # print('path_pos shape', path_pos.shape)
                    # ax = plt.subplot(1, 1, 1)
                    # ax.set_aspect(1)
                    # ax.set_xlim(self.config_space.low_lims[0], self.config_space.high_lims[0])
                    # ax.set_ylim(self.config_space.low_lims[1], self.config_space.high_lims[1])
                    # plt.plot(path_pos[:,0], path_pos[:,1])
                    # plt.show()
                    # velo = []


                    # dt = 0.1
                    # for i in range(path_pos.shape[0]):
                    #     if i < 1:
                    #         velo_x, velo_y = 0, 0
                    #         velo.append([velo_x,velo_y])
                    #     else:
                    #         velo_x = (path_pos[i][0] - path_pos[i-1][0])/dt
                    #         velo_y = (path_pos[i][1] - path_pos[i-1][1]) / dt
                    #         velo.append([velo_x, velo_y])
                    # print('velo', len(velo), velo)
                    # acc = []
                    # velo = np.array(velo)
                    # print('velo', velo.shape)
                    # for i in range(velo.shape[0]):
                    #     if i < 1:
                    #         acc_x, acc_y = 0, 0
                    #         acc.append([acc_x,acc_y])
                    #     else:
                    #         acc_x = (velo[i][0]-velo[i-1][0])/dt ### dt = 0.1 from configuration space
                    #         acc_y = (velo[i][1] - velo[i - 1][1]) / dt
                    #         acc.append([acc_x, acc_y])
                    # print('acc', acc)
                    # acc = np.array(acc)
                    # alpha_acc = find_angular_acc(acc[0,:], acc[1,:], 0)
                    # print("alpha_acc", alpha_acc)
                    # alpha = find_angular_displacement(alpha_acc, dt)


                    alpha = random_angular_displacement()
                    print("underactuated join state before enter c reigion", alpha)
                    self.graph.add_node(goal_a, new_config, path_to_goal)
                    path_final = self.config_space.local_plan(new_config, goal_a)    #### REPLACE this local_plan with a new local_plan
                    if self.config_space.check_path_collision(path_final):
                        continue
                    end_pos = simu.simulation_try(path_final.positions[:, :3], alpha)
                    print('end_pos', end_pos)
                    ### refactor this section code, the current logic is that, if the desired underactuated state is not acheive,
                    ## then it will return a plan (even not successful)
                    if np.allclose(end_pos[:2], goal[:2], rtol=0.05, atol=0.05):
                        print('end_pos', end_pos[:2], 'goal', goal[:2])
                        print('if simulation result is close enough to desired, successful!',
                              np.allclose(end_pos[:2], goal[:2], rtol=0.05, atol=0.05))
                        self.graph.add_node(goal_a, new_config, path_final)
                        self.plan = self.graph.construct_path_to(goal_a)
                    else:
                        self.graph.add_node(goal_a, new_config, path_final)
                        self.plan = self.graph.construct_path_to(goal_a)
                        print('graph', self.graph.nodes)
                        print('did not acheive to the desired underactuated ', self.plan.positions)
                        break

                    return self.plan

        else:
            for it in tqdm(range(self.max_iter)):
                rand_config = self.config_space.sample_config(goal)  # generate random point
                # using goal zoom/bias, this is from class Bicycle
                if self.config_space.check_collision(rand_config):  # check collision for respawn point
                    continue  # if true, 'continue' will go to the next loop of the for loop
                closest_config = self.config_space.nearest_config_to(self.graph.nodes,
                                                                     rand_config)  # found the nearest point from the graph.nodes
                path = self.config_space.local_plan(closest_config, rand_config)  # use local planer to generate path
                if self.config_space.check_path_collision(path):  # check the path collision
                    continue
                delta_path = path.get_prefix(prefix_time_length)  # get the new path under step length setting
                new_config = delta_path.end_position()  # set the last point of that path above as the last point
                self.graph.add_node(new_config, closest_config, delta_path)  # add point to the tree

                if self.config_space.distance(new_config, goal) <= self.expand_dist:  # check if it reach to goal
                    path_to_goal = self.config_space.local_plan(new_config, goal)
                    if self.config_space.check_path_collision(path_to_goal):
                        continue
                    print('graph-1', len(self.graph.nodes), self.graph.nodes)
                    self.graph.add_node(goal, new_config, path_to_goal)
                    self.plan = self.graph.construct_path_to(goal)
                    print('graph', self.graph.nodes)
                    print('plan of the iteration', self.plan.positions)
                    return self.plan
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

        # new_x = new_goal[0]
        # new_y = new_goal[1]
        # ee_x = new_x + np.sin(new_goal[2]) * 1.0
        # ee_y = new_y - np.cos(new_goal[2]) * 1.0
        # ax.plot(ee_x, ee_y, color='blue', marker='o', markersize=3)

        # goal_orien = goal[2]
        if self.plan:
            goal_pos_x = self.plan.positions[-1][0] + np.sin(goal_orien) * 1.0
            goal_pos_y = self.plan.positions[-1][1] - np.cos(goal_orien) * 1.0

        # ax.plot(new_x, new_y, color='yellow', marker='o',markersize=5)
        # ax.plot(goal_pos_x, goal_pos_y, color='blue', marker='^', markersize=3)
        ax.plot(6.5, 5.1, color='blue', marker='^', markersize=3)

        # gripper_tip = [self.plan.positions[:, 0], self.config_space.gripper_deflection(self.plan.positions[:, 1])]
        if self.plan:
            plan_x = self.plan.positions[:, 0]
            plan_y = self.plan.positions[:, 1]
            angle = self.plan.positions[:,2]
            # for i in range(len(plan_x)):
            #     x_i = plan_x[i]+np.sin(angle[i])*0.2
            # x_angle = plan_x + np.cos(angle)*0.2
            # y_angle = plan_y + np.sin(angle) * 0.2
            print(len(plan_y), "length of the time steps")
            ax.plot(plan_x, plan_y, color='green')
            # ax.plot(x_angle, y_angle, color='blue',marker='o',markersize=1)

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


def find_angular_acc(x_acc, y_acc, theta_init):  ### basically a non-inverted pendulum attached to a cart moving in 2D space

    ### initial theta(underactuated join value) is 0 as assumed
    # F_x, F_y = 50, 50  ### assume max reaction force is 50 N
    g = 9.81
    theta = theta_init
    l = 0.5 ## l is the length of pendulum
    theta_acc_list = []
    for i in range(len(x_acc)):

        theta_acc = (3/(2*l))*(np.cos(theta)*x_acc[i] + np.sin(theta)*g + np.sin(theta)*y_acc[i])
        theta_acc_list.append(theta_acc)
    return theta_acc_list
def find_angular_displacement(theta_acc, dt):
    velo = 0
    displacement = 0
    for i in range(len(theta_acc)):
        velo = velo + theta_acc[i] * dt ### dt = 0.1
        displacement = displacement + velo * dt + 0.5*theta_acc[i] * (dt*dt)
        # displacement = np.clip(displacement, -1.0, 1.0)    ### not sure if we want to add this
    return displacement

def random_angular_displacement():
    displacement = np.random.uniform(-0.1, 1.0)
    return displacement

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
    T1_start = time.time()
    start = np.array([2, 2, 0, 0])
    goal = np.array([6.5, 5.1, 1.3694384, 0])

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
    plan = planner.plan_to_pose(start, goal, prefix_time_length=1.0)
    # plot_new = self.new_goal
    T1_end = time.time()
    T1= T1_end - T1_start
    print("==t1===", T1)
    planner.plot_execution()
    print('path len:', len(plan.positions))
    print('final pose:', plan.positions[-1])
    # print('final path', Plan.open_loop_inputs())

# def test_runing():
#     for i in range(100):
#         main()
# if __name__ == '__main__':
#     test_runing()

if __name__ == '__main__':
    main()