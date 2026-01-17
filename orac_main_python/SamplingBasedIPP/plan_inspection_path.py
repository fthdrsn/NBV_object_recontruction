"""
Path planning for inspection path using sampling-based IPP method. This is a baseline
method  which is not related to our algorithm. The algoritm samples candidate views between current robot position and NBV,
and visits them sequentially.

"""

import math
from Utils.utils import *
import numpy as np
import random
from matplotlib import pyplot as plt
from dqrobotics import *
from SamplingBasedIPP.rrt_star import RRTStar
from math import pi


class IPP:
    def __init__(self, start, goal, max_iter=5000, expand_dis=0.8) -> None:

        self.rrt_plan = RRTStar(start=[start[0], start[1], start[2]], goal=[
            goal[0], goal[1], goal[2]], expand_dis=expand_dis, max_iter=max_iter)

        self.all_samples = []
        self.path_positions = []
        self.path_views = []
        self.sampling_angle_range = [-pi/6, pi/6]
        self.num_of_sample = 20
        self.sampling_radius_range = [0.05, 0.4]
        self.search_space_center = [0, 0, 0]

    def get_ipp_result(self):
        self.rrt_res = self.rrt_plan.plan()
        is_plan_found = len(self.rrt_res) != 0

        results_dic = {
            "rrt_positions": self.rrt_res,            # Shortest path positions
            "rrt_node_list": self.rrt_plan.node_list,  # All nodes in RRT
        }
        return is_plan_found, results_dic

    def sample_in_sphere(self, node_position):

        sample_list = []
        centre_position = self.search_space_center

        # Sample a point in a shpere centred around rrt node,
        # Check point to see if it is outside object cylinder and inside maximum and minimum height limits
        for _ in range(self.num_of_sample):
            is_point_valid = False
            while not is_point_valid:
                radius_sample = random.uniform(
                    self.sampling_radius_range[0], self.sampling_radius_range[1])
                p_x, p_y, p_z = self.random_point_in_sphere(
                    node_position, radius_sample)
                is_point_valid = self.check_point([p_x, p_y, p_z])
            # Randomly sample a orientation around x an y
            random_angle_y = random.uniform(
                # random angle around y
                self.sampling_angle_range[0], self.sampling_angle_range[1])
            random_angle_x = random.uniform(
                # random angle around x
                self.sampling_angle_range[0], self.sampling_angle_range[1])
            # Apply rotations first around y, then around x

            view_position = 1+E_*0.5 * \
                (p_x*i_+p_y*j_+p_z*k_)
            vw = vec3(translation(view_position))
            dir_line = np.array(
                [centre_position[0]-vw[0], centre_position[1]-vw[1], 0])

            quat = direction_to_orientation(dir_line)

            pose_cam = normalize((quat+E_*0.5*translation(view_position)*quat) *
                                 (math.cos(random_angle_y/2)+j_*math.sin(random_angle_y/2)) *
                                 (math.cos(random_angle_x/2)+i_*math.sin(random_angle_x/2)))
            sample_list.append(pose_cam)

        return sample_list

    def random_point_in_sphere(self, center, radius):
        # Generate random spherical coordinates
        theta = random.uniform(0, 2 * math.pi)  # Azimuthal angle (longitude)
        phi = random.uniform(0, math.pi)         # Polar angle (latitude)

        # Convert spherical coordinates to Cartesian coordinates
        x = center[0] + radius * math.sin(phi) * math.cos(theta)
        y = center[1] + radius * math.sin(phi) * math.sin(theta)
        z = center[2] + radius * math.cos(phi)

        return x, y, z

    def check_point(self, pnt):

        d = pnt[0]**2+pnt[1]**2+pnt[2]**2
        d = np.sqrt(d)
        if d < self.rrt_plan.min_rad or d > self.rrt_plan.max_rad:
            return False
        if pnt[2] < self.rrt_plan.min_height or pnt[2] > self.rrt_plan.max_height:
            return False

        return True  # safe
