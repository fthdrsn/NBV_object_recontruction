
from math import pi
import numpy as np
from pyrep import PyRep
import pyrep.objects as PyRepObj
import enum
from os.path import join, dirname, abspath
from QP import DQ_QuadprogSolver_Custom
from dqrobotics import *
from Robots.RobotsInteraction import YouBotModel
from Robots.Communication import BaseCommunication
from Robots.utils import *
import yaml
from pyrep.objects.shape import Shape
from pyrep.const import PrimitiveShape
from pyrep.objects.vision_sensor import VisionSensor


class ManageVisuals:
    def __init__(self, pr):
        self.pr = pr
        self.comm_agent = BaseCommunication()
        # Get the relevant object handles from CoppeliaSim
        self.camera_frame = PyRepObj.Object.get_object("camera_frame")
        self.depth_camera = VisionSensor("kinect_depth")
        self.nbv_frame = PyRepObj.Object.get_object("nbv_frame")
        self.sampling_best_view_frame = PyRepObj.Object.get_object(
            "sampling_best_view_frame")
        self.focus_point_visual = PyRepObj.Object.get_object("focus_point")

        self.obs_handle_list = []
        self.rrt_paths_handle_list = []
        self.rrt_shortest_path_handle_list = []
        self.rrt_candidate_view_handle_list = []

    def set_depth_camera_parameters(self, res, persp_angle, clip_near=0.01, clip_far=5.0):
        """ Set the depth camera perspective parameters: resolution and angles """
        self.depth_camera.set_resolution((int(res[0]), int(res[1])))
        self.depth_camera.set_perspective_angle(persp_angle)
        self.depth_camera.set_near_clipping_plane(clip_near)
        self.depth_camera.set_far_clipping_plane(clip_far)

    def get_depth_camera_parameters(self):
        """ Get the depth camera perspective parameters: resolution and angles """
        res = np.array(self.depth_camera.get_resolution())
        ratio = res[0] / res[1]
        pa_x = pa_y = math.radians(self.depth_camera.get_perspective_angle())
        if ratio > 1:
            pa_y = 2 * np.arctan(np.tan(pa_y / 2) / ratio)
        elif ratio < 1:
            pa_x = 2 * np.arctan(np.tan(pa_x / 2) * ratio)
        persp_angles = np.array([math.degrees(pa_x), math.degrees(pa_y)])
        near_clip = self.depth_camera.get_near_clipping_plane()
        far_clip = self.depth_camera.get_far_clipping_plane()

        return res, persp_angles, near_clip, far_clip

    def set_nbv_frame_pose(self, pose_dq):
        self.comm_agent.set_object_pose(self.nbv_frame, pose_dq)

    def set_focus_point_position(self, focus_position_np):
        self.focus_point_visual.set_position(focus_position_np)

    def set_sampling_best_view_frame_pose(self, pose_dq):
        self.comm_agent.set_object_pose(self.sampling_best_view_frame, pose_dq)

    def show_obstacles(self, obs_pose_list_dq, obs_rad_list):
        """ Show the obstacles in the CoppeliaSim simulation given their poses and radii """
        self.clear_obstacles()  # Clear existing obstacles
        for idx_, (obs_pose_, obs_rad_) in enumerate(zip(obs_pose_list_dq, obs_rad_list)):
            # Create obstacle visuals in the simulation
            self.obs_visual = Shape.create(type=PrimitiveShape.CYLINDER,
                                           size=[2*obs_rad_,
                                                 2*obs_rad_, 0.22],
                                           color=[0.0, 0.0, 1.0],
                                           static=True, respondable=False, renderable=False)
            self.obs_visual.set_name(f"base_obs_{idx_}")  # Set obstacle name
            self.obs_visual.set_color((0, 0, 1))  # Set obstacle color to blue
            self.comm_agent.set_object_pose(
                self.obs_visual, obs_pose_)  # Set obstacle pose
            self.obs_handle_list.append(
                self.obs_visual)  # Store obstacle handle

    def show_rrt_paths(self, rrt_result_dic):
        """ Show the RRT* planned path in the CoppeliaSim simulation """
        self.clear_rrt_path()  # Clear existing RRT* path visuals
        path_sample_count = 70
        for rrt_node in rrt_result_dic["rrt_node_list"][:path_sample_count]:
            prnt_list = []
            prnt_list.append([rrt_node.x, rrt_node.y, rrt_node.z])
            prnt = rrt_node.parent
            if prnt != None:
                prnt_list.append([prnt.x, prnt.y, prnt.z])
            while prnt != None:
                prnt = prnt.parent
                if prnt != None:
                    prnt_list.append([prnt.x, prnt.y, prnt.z])
            prnt_np = np.array(prnt_list)
            for i in range(0, prnt_np.shape[0]-1):
                target_a1 = Shape.create(type=PrimitiveShape.SPHERE,
                                         size=[0.05, 0.05, 0.05],
                                         color=[0.0, 0.0, 1.0],
                                         static=True, respondable=False, renderable=False)
                target_a2 = Shape.create(type=PrimitiveShape.SPHERE,
                                         size=[0.05, 0.05, 0.05],
                                         color=[0.0, 0.0, 1.0],
                                         static=True, respondable=False, renderable=False)
                dir_cyl = (prnt_np[i+1]-prnt_np[i]) / \
                    np.linalg.norm(prnt_np[i+1]-prnt_np[i])
                len_cyl = np.linalg.norm(prnt_np[i+1]-prnt_np[i])
                target_cyl = Shape.create(type=PrimitiveShape.CYLINDER,
                                          size=[0.01, 0.01, len_cyl],
                                          color=[1.0, 0.0, 0.0],
                                          static=True, respondable=False, renderable=False)
                position_cyl = prnt_np[i]+len_cyl*dir_cyl*0.5
                position_cyl_dq = position_cyl[0]*i_ + \
                    position_cyl[1]*j_+position_cyl[2]*k_
                quat = direction_to_orientation(dir_cyl)
                pose_cyl_dq = (quat+E_*0.5*position_cyl_dq*quat)
                pose_cyl_dq = normalize(pose_cyl_dq)
                target_a1.set_position(prnt_np[i])
                target_a2.set_position(prnt_np[i+1])

                self.comm_agent.set_object_pose(
                    target_cyl, pose_cyl_dq)

                self.rrt_paths_handle_list.append(target_a1)
                self.rrt_paths_handle_list.append(target_a2)
                self.rrt_paths_handle_list.append(target_cyl)

    def show_shortest_rrt_path(self, rrt_path_points):
        # Visualize tree nodes
        self.clear_shortest_rrt_path()  # Clear existing shortest path visuals
        for path_node_position in rrt_path_points:
            target = Shape.create(type=PrimitiveShape.SPHERE,
                                  size=[0.1, 0.1, 0.1],
                                  color=[0.0, 1.0, 0.0],
                                  static=True, respondable=False, renderable=False)
            target.set_position(path_node_position)
            self.rrt_shortest_path_handle_list.append(target)

    def show_rrt_candidate_views(self, candidate_views_dq):
        # Visualize candidate views
        self.clear_rrt_candidate_views()  # Clear existing candidate view visuals
        for view_dq in candidate_views_dq:
            cam_visual = Shape.create(type=PrimitiveShape.CONE,
                                      size=[0.05, 0.05, 0.1],
                                      color=[1.0, 1.0, 0.0],
                                      static=True, respondable=False, renderable=False)
            self.comm_agent.set_object_pose(cam_visual, view_dq)
            self.rrt_candidate_view_handle_list.append(cam_visual)

    def get_workspace_plane_poses(self):
        """ Get the workspace limits defined by
            four planes in the scene """
        # Get the plane poses from CoppeliaSim as dual quaternions
        plane_left_pose = self.comm_agent.get_object_pose(
            PyRepObj.Object.get_object("plane_left"))  # Right plane (Min y)
        plane_right_pose = self.comm_agent.get_object_pose(
            PyRepObj.Object.get_object("plane_right"))  # Left plane (Max y)
        plane_top_pose = self.comm_agent.get_object_pose(
            PyRepObj.Object.get_object("plane_top"))  # Upper plane (Max x)
        plane_bottom_pose = self.comm_agent.get_object_pose(
            PyRepObj.Object.get_object("plane_bottom"))  # Lower plane (Min x)

        return plane_left_pose, plane_right_pose, plane_top_pose, plane_bottom_pose

    def create_shape_visual(self, mesh_path, pose_dq, scale):
        """ Create a shape visual in the CoppeliaSim simulation from a given mesh path, pose and scale """

        rec_obj_ref = Shape.import_mesh(
            mesh_path+"/model.obj", scaling_factor=scale, ignore_up_vector=True)
        rec_obj_ref.is_renderable = True
        self.comm_agent.set_object_pose(rec_obj_ref, pose_dq)
        return rec_obj_ref

    def clear_obstacles(self):
        """ Remove all obstacle visuals from the CoppeliaSim simulation """
        for obs_handle in self.obs_handle_list:
            obs_handle.remove()
        self.obs_handle_list = []  # Clear the list of obstacle handles

    def clear_rrt_path(self):
        """ Remove all RRT* path visuals from the CoppeliaSim simulation """
        for path_handle in self.rrt_paths_handle_list:
            path_handle.remove()
        self.rrt_paths_handle_list = []  # Clear the list of path handles

    def clear_shortest_rrt_path(self):
        """ Remove all shortest RRT* path visuals from the CoppeliaSim simulation """
        for path_handle in self.rrt_shortest_path_handle_list:
            path_handle.remove()
        # Clear the list of shortest path handles
        self.rrt_shortest_path_handle_list = []

    def clear_rrt_candidate_views(self):
        """ Remove all RRT* candidate view visuals from the CoppeliaSim simulation """
        for view_handle in self.rrt_candidate_view_handle_list:
            view_handle.remove()
        # Clear the list of candidate view handles
        self.rrt_candidate_view_handle_list = []
