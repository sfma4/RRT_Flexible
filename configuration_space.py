#!/usr/bin/env python


from copy import copy
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from contextlib import contextmanager

sample_list = []
class Plan(object):
    """Data structure to represent a motion plan. Stores plans in the form of
    three arrays of the same length: times, positions, and open_loop_inputs.

    The following invariants are assumed:
        - at time times[i] the plan prescribes that we be in position
          positions[i] and perform input open_loop_inputs[i].
        - times starts at zero. Each plan is meant to represent the motion
          from one point to another over a time interval starting at 
          time zero. If you wish to append together multiple paths
          c1 -> c2 -> c3 -> ... -> cn, you should use the chain_paths
          method.
    """

    def __init__(self, times, target_positions, open_loop_inputs, dt=0.01):
        self.dt = dt
        self.times = times
        self.positions = target_positions
        self.open_loop_inputs = open_loop_inputs

    def __iter__(self):
        for t, p, c in zip(self.times, self.positions, self.open_loop_inputs):
            yield t, p, c

    def __len__(self):
        return len(self.times)

    def get(self, t):
        """Returns the desired position and open loop input at time t.
        """
        index = int(np.sum(self.times <= t))
        index = index - 1 if index else 0
        return self.positions[index], self.open_loop_inputs[index]

    def end_position(self):
        return self.positions[-1]

    def start_position(self):
        return self.positions[0]

    def get_prefix(self, until_time):
        """Returns a new plan that is a prefix of this plan up until the
        time until_time.
        """
        times = self.times[self.times <= until_time]
        positions = self.positions[self.times <= until_time]
        open_loop_inputs = self.open_loop_inputs[self.times <= until_time]
        return Plan(times, positions, open_loop_inputs)

    @classmethod
    def chain_paths(self, *paths):
        """Chain together any number of plans into a single plan.
        """

        def chain_two_paths(path1, path2):
            """Chains together two plans to create a single plan. Requires
            that path1 ends at the same configuration that path2 begins at.
            Also requires that both paths have the same discretization time
            step dt.
            """
            if not path1 and not path2:
                return None
            elif not path1:
                return path2
            elif not path2:
                return path1
            assert path1.dt == path2.dt, "Cannot append paths with different time deltas."
            # print('end position', path1.end_position(),
            #                    path2.start_position())
            # if np.allclose(path1.end_position(),
            #                    path2.start_position(), rtol=5e-02) is False:
            #     print('check path1 and path2 to debug', path1.end_position(), path2.start_position())
            # assert np.allclose(path1.end_position(),
            #                    path2.start_position(), rtol=1e-01), "Cannot append paths with inconsistent start and end positions."
            times = np.concatenate((path1.times, path1.times[-1] + path2.times[1:]), axis=0)
            positions = np.concatenate((path1.positions, path2.positions[1:]), axis=0)
            open_loop_inputs = np.concatenate((path1.open_loop_inputs, path2.open_loop_inputs[1:]), axis=0)
            dt = path1.dt
            # print('positions in chain_two_path', positions)
            return Plan(times, positions, open_loop_inputs, dt=dt)

        chained_path = None
        for path in paths:
            chained_path = chain_two_paths(chained_path, path)
        return chained_path


@contextmanager
def expanded_obstacles(obstacle_list, delta):
    """Context manager that edits obstacle list to increase the radius of
    all obstacles by delta.
    
    Assumes obstacles are circles in the x-y plane and are given as lists
    of [x, y, r] specifying the center and radius of the obstacle. So
    obstacle_list is a list of [x, y, r] lists.

    Note we want the obstacles to be lists instead of tuples since tuples
    are immutable and we would be unable to change the radii.

    Usage:
        with expanded_obstacles(obstacle_list, 0.1):
            # do things with expanded obstacle_list. While inside this with 
            # block, the radius of each element of obstacle_list has been
            # expanded by 0.1 meters.
        # once we're out of the with block, obstacle_list will be
        # back to normal
    """
    for obs in obstacle_list:
        obs[2] += delta
    yield obstacle_list
    for obs in obstacle_list:
        obs[2] -= delta


class ConfigurationSpace(object):
    """ An abstract class for a Configuration Space. 
    
        DO NOT FILL IN THIS CLASS

        Instead, fill in the BicycleConfigurationSpace at the bottom of the
        file which inherits from this class.
    """

    def __init__(self, dim, low_lims, high_lims, obstacles, dt=0.01):
        """
        Parameters
        ----------
        dim: dimension of the state space: number of state variables.
        low_lims: the lower bounds of the state variables. Should be an
                iterable of length dim.
        high_lims: the higher bounds of the state variables. Should be an
                iterable of length dim.
        obstacles: A list of obstacles. This could be in any representation
            we choose, based on the application. In this project, for the bicycle
            model, we assume each obstacle is a circle in x, y space, and then
            obstacles is a list of [x, y, r] lists specifying the center and 
            radius of each obstacle.
        dt: The discretization timestep our local planner should use when constructing
            plans.
        """
        self.dim = dim
        self.low_lims = np.array(low_lims)
        self.high_lims = np.array(high_lims)
        self.obstacles = obstacles
        self.dt = dt

    def distance(self, c1, c2):
        """
            Implements the chosen metric for this configuration space.
            This method should be implemented whenever this ConfigurationSpace
            is subclassed.

            Returns the distance between configurations c1 and c2 according to
            the chosen metric.
        """
        pass

    def sample_config(self, *args):
        """
            Samples a new configuration from this C-Space according to the
            chosen probability measure.
            This method should be implemented whenever this ConfigurationSpace
            is subclassed.

            Returns a new configuration sampled at random from the configuration
            space.
        """
        pass

    def check_collision(self, c):
        """
            Checks to see if the specified configuration c is in collision with
            any obstacles.
            This method should be implemented whenever this ConfigurationSpace
            is subclassed.
        """
        pass

    def check_path_collision(self, path):
        """
            Checks to see if a specified path through the configuration space is 
            in collision with any obstacles.
            This method should be implemented whenever this ConfigurationSpace
            is subclassed.
        """
        pass

    def local_plan(self, c1, c2):
        """
            Constructs a plan from configuration c1 to c2.

            This is the local planning step in RRT. This should be where you extend
            the trajectory of the robot a little bit starting from c1. This may not
            constitute finding a complete plan from c1 to c2. Remember that we only
            care about moving in some direction while respecting the kinemtics of
            the robot. You may perform this step by picking a number of motion
            primitives, and then returning the primitive that brings you closest
            to c2.
        """
        pass

    def nearest_config_to(self, config_list, config):
        """
            Finds the configuration from config_list that is closest to config.
        """
        return np.asarray(min(config_list, key=lambda c: self.distance(c, config)))


class FreeEuclideanSpace(ConfigurationSpace):
    """
        Example implementation of a configuration space. This class implements
        a configuration space representing free n dimensional euclidean space.
    """

    def __init__(self, dim, low_lims, high_lims, sec_per_meter=4):
        super(FreeEuclideanSpace, self).__init__(dim, low_lims, high_lims, [])
        self.sec_per_meter = sec_per_meter

    def distance(self, c1, c2):
        """
        c1 and c2 should by numpy.ndarrays of size (dim, 1) or (1, dim) or (dim,).
        """
        return np.linalg.norm(c1 - c2)

    def sample_config(self, *args):
        probability_of_sampling_goal = 0.2
        p = np.random.uniform(0, 1)
        scale = np.random.rand(1, 4)[0]
        if p < probability_of_sampling_goal:
            print('sample goal')
            sample = goal
        else:
            sample = np.multiply(self.high_lims - self.low_lims, scale) + self.low_lims
        # return np.random.uniform(self.low_lims, self.high_lims).reshape((self.dim,))
        return sample

    def check_collision(self, c):
        return False

    def check_path_collision(self, path):
        return False

    def local_plan(self, c1, c2):
        v = c2 - c1
        dist = np.linalg.norm(c1 - c2)
        total_time = dist * self.sec_per_meter
        vel = v / total_time
        p = lambda t: (1 - (t / total_time)) * c1 + (t / total_time) * c2
        times = np.arange(0, total_time, self.dt)
        positions = p(times[:, None])
        velocities = np.tile(vel, (positions.shape[0], 1))
        plan = Plan(times, positions, velocities, dt=self.dt)
        return plan

class FlexibleLocalPlanner(ConfigurationSpace):
    """
        The configuration space for a Bicycle modeled robot
        Obstacles should be tuples (a, b, c, d), representing rectangles
        We assume that the robot is circular and has radius equal to robot_radius
        The state of the robot is defined as (x, y, alpha). alpha is the angle of the gripper
    """

    def __init__(self, low_lims, high_lims, input_low_lims, input_high_lims, obstacles, robot_radius):
        dim = 4
        super(FlexibleLocalPlanner, self).__init__(dim, low_lims, high_lims, obstacles)
        self.robot_radius = robot_radius
        self.robot_length = 0.3  ## consider this as the gripper length
        self.input_low_lims = input_low_lims   ## input can be the torque input??
        self.input_high_lims = input_high_lims
        self.low_lims_phi = low_lims
        self.high_lims_phi = high_lims
        self.zoom_radius = None

    def distance(self, c1, c2):
        """
        c1 and c2 should be numpy.ndarrays of size (4,)
        """
        x1 = c1[0]
        y1 = c1[1]
        x2 = c2[0]
        y2 = c2[1]
        sin_theta1 = np.sin(c1[2])
        cos_theta1 = np.cos(c1[2])
        sin_theta2 = np.sin(c2[2])
        cos_theta2 = np.cos(c2[2])

        return (x1 - x2) ** 2 + (y1 - y2) ** 2 + 0.6 * ((sin_theta1 - sin_theta2) ** 2 + (cos_theta1 - cos_theta2) ** 2)

    def sample_config(self, goal):
        """
        Pick a random configuration from within our state boundaries.

        You can pass in any number of additional optional arguments if you
        would like to implement custom sampling heuristics. By default, the
        RRT implementation passes in the goal as an additional argument,
        which can be used to implement a goal-biasing heuristic.
        """
        # probability_of_sampling_goal = 1.0
        # p = np.random.uniform(0, 1)
        # scale = np.random.rand(1, 4)[0]
        #
        # sample = np.multiply(self.high_lims - self.low_lims, scale) + self.low_lims
        # if p < probability_of_sampling_goal:
        #     print('sampling goal')
        #     sample = goal
        # else:
        #     sample = np.multiply(self.high_lims - self.low_lims, scale) + self.low_lims
        #
        # return sample


        probability_of_sampling_goal = 0.1
        p = np.random.uniform(0, 1)
        scale = np.random.rand(1, 4)[0]
        if self.zoom_radius is None:
            self.zoom_radius = np.sqrt((1 - goal[0]) ** 2 + (1 - goal[1]) ** 2)  #### harcoding the starting position!
        # zoom_radius = min(Distance_goal)
        if p < probability_of_sampling_goal:
            radius = self.zoom_radius #the radius of the sample range circle
            circle_x = goal[0]
            circle_y = goal[1]
            alpha = 2 * np.pi * np.random.rand() # ramdom angle
            r = radius * np.sqrt(np.random.rand())  # random radius
            x = r * np.cos(alpha) + circle_x
            y = r * np.sin(alpha) + circle_y
            sample = [x, y, np.random.uniform(self.low_lims[2], self.high_lims[2]), np.random.uniform(self.low_lims[3], self.high_lims[3])]
            new_distance = np.sqrt((sample[0] - goal[0]) ** 2 + (sample[1] - goal[1]) ** 2)
            if new_distance < self.zoom_radius:
                self.zoom_radius = new_distance
            # print('random sample near goal', sample)
            # print('updated radius', radius)
        else:
            sample = np.multiply(self.high_lims - self.low_lims, scale) + self.low_lims
        return sample



    def check_collision(self, c):
        """
        Returns true if a configuration c is in collision
        c should be a numpy.ndarray of size (4,)
        """
        x = c[0]
        y = c[1]
        for obj in self.obstacles:
            # change the collsion method to previous method (x,y, length, width) so that multiple obstacles can be used
            point_1 = self.obstacles[0]  #left bottom
            point_2 = self.obstacles[1]  #left top
            point_3 = self.obstacles[2]  #right bottom
            point_4 = self.obstacles[3]  #right top
            if (x >= point_1[0]) \
                and (x <= point_3[0]) \
                and (y >= point_1[1]) \
                and (y <= point_2[1]):
                return True
        return False

    def check_collision_c(self, rand, c):
        x = rand[0]
        y = rand[1]
        point_1 = c[0]  # left bottom
        point_2 = c[1]  # left top
        point_3 = c[2]  # right bottom
        point_4 = c[3]  # right top
        if (x >= point_1[0]) \
                and (x <= point_3[0]) \
                and (y >= point_1[1]) \
                and (y <= point_2[1]):
            return True

    def check_path_collision(self, path):
        """
        Returns true if the input path is in collision. The path
        is given as a Plan object. See configuration_space.py
        for details on the Plan interface.

        You should also ensure that the path does not exceed any state bounds,
        and the open loop inputs don't exceed input bounds.
        """
        # for time, position, inputs in path:
        c1 = path.positions[0]
        c2 = path.positions[-1]
        last_position = None
        for time, position, inputs in path:

            if time == 0:
                last_position = position
            # if self.distance(last_position, position) < 0.2:
            #     return True
            try:
                inbetween = np.linspace(last_position, position, 10)
            except:
                inbetween = np.array([last_position, position])
            for position in inbetween:
                collide = self.check_collision(position)

                inval_state = bool(np.any(np.array(position) < np.array(self.low_lims))
                                   or np.any(np.array(position) > np.array(self.high_lims)))

                if collide or inval_state:
                    # return True
                    if self.flexibility_check(c1):
                        print("flexibility pass")
                        return False
                    else:
                        # print('path collision')
                        return True

            # inval_input = bool(np.any(np.array(inputs) < np.array(self.input_low_lims))
            #                    or np.any(np.array(inputs) > np.array(self.input_high_lims)))
            #
            # if inval_input:
            #     return True

            last_position = position

        return False

    def flexibility_check(self, c1): #compare the result of the force output and max deformation force
        if self.dynamics_model(c1)[0] > 0.6:
            return False
        else:
            print("flexibility pass")
            return True

    def gripper_deflection(self, y):
        l = 0.4  # length of the gripper
        mass_object = 3  # assume the mass of the grasping object is 3kg
        p = mass_object * 9.81
        phi =  p * l**2 / 10 # assume 2EI = 10, calculating the angle of deflection
        delta = p * l**3 / 15 # assume 2EI = 10, calculating vertical deflection distance
        tip_pos = y - delta # y is the position of robot on y direction
        return tip_pos
    ## this calculate the natural gripper angle by gravity and by previous step and current obstacles

    # def gripper_tip_pos(self, c1):
    # ## this calculate the end position of the gripper

    def find_C_reigion(self,  l):
        C_region = [[self.obstacles[0][0] - l, 0],
                    [self.obstacles[1][0] - l, self.obstacles[1][1] + l],
                    [self.obstacles[2][0] + l, self.obstacles[2][1] - l],
                    [self.obstacles[3][0] + l, self.obstacles[3][0] - l
                     ]]
        return C_region

    def dynamics_model(self, c1): #generate result of the force output
        l = 0.4 # length of the gripper
        mass_object = 3  # assume the mass of the grasping object is 3kg
        p = abs( 180 * (c1[0] + l - 5))  +3 * 9.81# friction resulted by the deformation of gripper contacting with the obstacles and weight
        angle_deflection = p * l ** 2 / 10  # assume 2EI = 10
        k = 10  # torsion spring constant
        load = k * angle_deflection # exerted force on robot from the deformation as assumed
        print(angle_deflection, 'theta')
        return angle_deflection,load

    def local_plan(self, c1, c2, dt=0.1, consider_theta=False, is_end=False):
        v = c2 - c1
        dist = np.linalg.norm(c1 - c2)
        total_time = dist * 1  # assume 10 seconds per meter
        vel = v / total_time
        p = lambda  t: (1 - (t / total_time)) * c1 + (t / total_time) * c2
        print('total time', total_time)
        times = np.arange(0, abs(total_time), self.dt)  ### I added abs() to debug
        positions = p(times[:, None])
        velocities = np.tile(vel, (positions.shape[0], 1))
        plan = Plan(times, positions, velocities, dt=self.dt)
        return plan


