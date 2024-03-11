"""
Move the robot to the NBVs, using visibility as a hard constraint. All environment constraints and joint limits constraints are also included.

"""

import math
from pyrep import PyRep
from Robots.utils import *
from dqrobotics.robot_control import DQ_PseudoinverseController, ControlObjective, DQ_ClassicQPController
from dqrobotics.utils import DQ_LinearAlgebra
from dqrobotics.solvers import DQ_QuadprogSolver
from dqrobotics.utils import DQ_Geometry
from dqrobotics.interfaces.vrep import DQ_VrepInterface

import numpy as np
from QP import DQ_QuadprogSolver_Custom
import pyrep
from pyrep.objects.shape import Shape
from pyrep.backend import sim as sim_pyrep
from pyrep.const import PrimitiveShape
from os.path import dirname, join, abspath
from pyrep.objects import VisionSensor
import random
import re
from matplotlib import pyplot as plt
from dqrobotics import *
import json
import rospy
import yaml
from rrt_star import RRTStar
import time


class IPP:
    def __init__(self, start, goal, max_iter=5000, expand_dis=0.8) -> None:

        self.rrt_plan = RRTStar(robot_map=np.zeros((100, 100)), start=[start[0], start[1], start[2]], goal=[
            goal[0], goal[1], goal[2]], expand_dis=expand_dis, max_iter=max_iter)

        self.all_samples = []
        self.path_positions = []
        self.path_views = []
        self.sampling_angle_range = [-pi/6, pi/6]
        self.num_of_sample = 20
        self.sampling_radius_range = [0.05, 0.4]

    def get_ipp_result(self):
        is_plan_found, path_views, view_list_srv, rrt_positions = self.get_path_views()
        all_samples = []
        if is_plan_found:
            all_samples = self.rrt_plan.samples_all
        results_dic = {"path_views": path_views,
                       "view_list_srv": view_list_srv,
                       "rrt_positions": rrt_positions,
                       "rrt_node_list": self.rrt_plan.node_list,
                       "rrt_generated_samples": all_samples
                       }
        return is_plan_found, results_dic

    def get_path_views(self):
        self.rrt_res = self.rrt_plan.plan()

        is_plan_found = False
        if len(self.rrt_res) != 0:
            is_plan_found = True
        view_list_srv = []
        views = []

        if is_plan_found:
            views = self.get_view_list(self.rrt_res)

            # Make the views one vector for the ros server requiest
            for dq_pose in views:
                trans = vec3(translation(dq_pose))
                orient = vec4(rotation(dq_pose))
                tmp_list = [trans[0], trans[1], trans[2],
                            orient[0], orient[1], orient[2], orient[3]]
                view_list_srv += tmp_list

        return is_plan_found, views, view_list_srv, self.rrt_res

   # Calculate the 5 orientation for each position in position list
   # The output size will be 5x input size

    def get_view_list(self, position_list):
        cam_poses_coppelia = []
        centre_position = [0, 0, 0]
        for pt in position_list:
            view_position = 1+E_*0.5*(pt[0]*i_+pt[1]*j_+pt[2]*k_)
            vw = vec4(translation(view_position))[1:]
            dir_line = np.array(
                [centre_position[0]-vw[0], centre_position[1]-vw[1], 0])

            quat = direction_to_pose(dir_line)
            # Centre, left and right poses
            comm_angles = [0, -pi/6, pi/6]
            for ang in comm_angles:
                pose_cam = (quat+E_*0.5*translation(view_position)*quat) * \
                    (math.cos(ang/2)+j_*math.sin(ang/2))
                pose_cam = normalize(pose_cam)
                cam_poses_coppelia.append(pose_cam)
            # Up, down
            comm_angles = [-pi/6, pi/6]
            for ang in comm_angles:
                pose_cam = (quat+E_*0.5*translation(view_position)*quat) * \
                    (math.cos(ang/2)+i_*math.sin(ang/2))
                pose_cam = normalize(pose_cam)
                cam_poses_coppelia.append(pose_cam)
        return cam_poses_coppelia

    def sample_in_cylinder(self, node_position):

        sample_list = []
        centre_position = [0, 0, 0]

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
                self.sampling_angle_range[0], self.sampling_angle_range[1])  # random angle around y
            random_angle_x = random.uniform(
                self.sampling_angle_range[0], self.sampling_angle_range[1])  # random angle around x
            # Apply rotations first around y, then around x

            view_position = 1+E_*0.5 * \
                (p_x*i_+p_y*j_+p_z*k_)
            vw = vec3(translation(view_position))
            dir_line = np.array(
                [centre_position[0]-vw[0], centre_position[1]-vw[1], 0])

            quat = direction_to_pose(dir_line)

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
