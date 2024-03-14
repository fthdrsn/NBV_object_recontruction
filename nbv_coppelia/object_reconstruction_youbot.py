
import math
from pyrep import PyRep
from Robots.utils import *
# from dqrobotics.robot_control import DQ_PseudoinverseController, ControlObjective, DQ_ClassicQPController
# from dqrobotics.utils import DQ_LinearAlgebra
# from dqrobotics.solvers import DQ_QuadprogSolver
from dqrobotics.utils import DQ_Geometry
# from dqrobotics.interfaces.vrep import DQ_VrepInterface

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
from dqrobotics import *
from focus_point_calculator.srv import focus_point_srv, coverage_srv, view_evaluate_srv, view_evaluate_srvRequest
from std_srvs.srv import Empty
import torch
import json
import rospy
import yaml
import time
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import os
import roslaunch
from plan_inspection_path import IPP


class NBV:
    def __init__(self, params) -> None:
        self.object_number = 0
        self.max_object_count = 114
        self.ig_method_number = 2
        self.pr = PyRep()
        self.params = params
        self.start_focus_now = True
        # Is velocity control activated
        # Use different simulation scenes for position and velocity
        # control since to use position control dynamics should be disabled.
        self.is_velocity_control = self.params["ProcessVariables"]["enableVelocityControl"]
        if self.is_velocity_control:
            SCENE_FILE = join(dirname(abspath(__file__)),
                              self.params["SimulatorSettings"]["sceneNameVelocity"])
        else:
            SCENE_FILE = join(dirname(abspath(__file__)),
                              self.params["SimulatorSettings"]["sceneNamePosition"])

        self.pr.launch(SCENE_FILE, headless=False)

        # Different simulation steps might be suitable for position and velocity control
        if self.is_velocity_control:
            self.simulation_time_step = self.params["SimulatorSettings"]["simTimeStepVelocity"]
        else:
            self.simulation_time_step = self.params["SimulatorSettings"]["simTimeStepPosition"]

        self.pr.set_simulation_timestep(self.simulation_time_step)
        self.pr.start()

        # # Create robots` models
        self.robot_model = YouBotModel(
            is_velocity_control=self.is_velocity_control)
        self.comm_agent = BaseCommunication()
        #### Controllers#####
        self.qp_solver = DQ_QuadprogSolver_Custom()

        if self.is_velocity_control:
            self.controller_gain = self.params["VelocityControllerSettings"]["controllerGain"]
            self.controller_damping = self.params["VelocityControllerSettings"]["controllerDamping"]
            self.controller_err_tolerance = self.params["VelocityControllerSettings"]["errorTolerance"]
            self.controller_stab_threshold = self.params["VelocityControllerSettings"]["stabilityThreshold"]
            self.controller_stab_max = self.params["VelocityControllerSettings"]["maxStabilityCount"]
            self.max_time_limit = self.params["VelocityControllerSettings"]["maxTimeLimit"]
        else:
            self.controller_gain = self.params["PositionControllerSettings"]["controllerGain"]
            self.controller_damping = self.params["PositionControllerSettings"]["controllerDamping"]
            self.controller_err_tolerance = self.params["PositionControllerSettings"]["errorTolerance"]
            self.controller_stab_threshold = self.params["PositionControllerSettings"]["stabilityThreshold"]
            self.controller_stab_max = self.params["PositionControllerSettings"]["maxStabilityCount"]
            self.integration_time_step = self.params["PositionControllerSettings"]["integrationTimeStep"]
            self.max_time_limit = self.params["PositionControllerSettings"]["maxTimeLimit"]

        # Robot pose will be the target if it is true, direction and position otherwise
        self.is_pose_control = self.params["ProcessVariables"]["enablePoseControl"]
        # The sampling-based informative path planning for the object reconstruction will be used if it is enabled
        self.enable_rrt_based_ipp = self.params["NBV"]["useRrtIPP"]

        self.camera_ref = PyRepObj.Object.get_object("CameraRef")
        self.nbv_obj = PyRepObj.Object.get_object("nbv")
        self.dq_ref = PyRepObj.Object.get_object("dqRef")
        self.ipp_taraget = PyRepObj.Object.get_object("nbv_ipp")
        self.focus_prev_ref = PyRepObj.Object.get_object("focus_prev_ref")
        self.focus_target = Shape.create(type=PrimitiveShape.SPHERE,
                                         size=[0.2, 0.2, 0.2],
                                         color=[0.0, 0.0, 1.0],
                                         static=True, respondable=False, renderable=False)

        self.num_of_base_obstacles = self.params["BaseObstacleConstraint"]["baseObsCount"]
        self.base_obstacle_poses = self.get_base_obstacles()
        self.enable_collision_constraints = self.params["ProcessVariables"]["enableCollisionConst"]
        self.robot_to_far = False

        # Get required paths
        self.launch_file_path = self.params["NBV"]["launchFilePath"]
        self.search_space_path = self.params["NBV"]["searchSpacePath"]

        # Base and Arm Velocity Limits
        self.vel_lim_base = self.params["JointContraints"]["velLimBase"]
        self.vel_lim_arm = self.params["JointContraints"]["velLimArm"]
        self.base_vel_lim_max = [self.vel_lim_base,
                                 self.vel_lim_base, self.vel_lim_base]
        self.base_vel_lim_min = [-self.vel_lim_base, -
                                 self.vel_lim_base, -self.vel_lim_base]

        self.arm_vel_lim_max = [self.vel_lim_arm, self.vel_lim_arm,
                                self.vel_lim_arm, self.vel_lim_arm, self.vel_lim_arm]
        self.arm_vel_lim_min = [-self.vel_lim_arm, -
                                self.vel_lim_arm, -self.vel_lim_arm, -self.vel_lim_arm, -self.vel_lim_arm]
        self.total_vel_lim_max = np.array(
            self.base_vel_lim_max + self.arm_vel_lim_max)
        self.total_vel_lim_min = np.array(
            self.base_vel_lim_min + self.arm_vel_lim_min)

        # Arm Joint Position Limits
        self.joint_pos_max = np.array(
            [ang*pi/180 for ang in self.params["JointContraints"]["armJointLimMax"]])
        self.joint_pos_min = np.array(
            [ang*pi/180 for ang in self.params["JointContraints"]["armJointLimMin"]])
        self.enable_joint_limits = self.params["ProcessVariables"]["enableJointLimitConst"]

        # Visibility constraint

        im_width = self.params["CameraParameters"]["imWidth"]
        im_height = self.params["CameraParameters"]["imHeight"]
        cam_fov = self.params["CameraParameters"]["camFov"]

        self.left_pose, self.right_pose, self.up_pose, self.down_pose = get_camera_fov_planes(
            im_width, im_height, cam_fov)
        self.enable_visibility_constraint = self.params["ProcessVariables"]["enableVisibilityConst"]
        self.enable_focus_point = self.params["NBV"]["useFocusPoint"]

        self.safe_dist_visibility = self.params["VisibilityConstraint"]["safeDistVisibility"]
        self.soft_alpha = self.params["VisibilityConstraint"]["softAlpha"]
        self.soft_beta = self.params["VisibilityConstraint"]["softBeta"]

        # Services
        self.coverage_func = rospy.ServiceProxy("get_coverage", coverage_srv)
        self.focus_pnt_func = rospy.ServiceProxy(
            "get_focus_point", focus_point_srv)

        self.view_ig_func = rospy.ServiceProxy(
            "get_view_igs", view_evaluate_srv)

        self.num_of_nbv_call = 0
        self.max_nbv_calls = self.params["NBV"]["maxNBVCalls"]

        self.shape_net_path = self.params["NBV"]["objectsPath"]
        self.gt_path = self.params["NBV"]["groundTruthPath"]
        self.object_path = self.gt_path+"/used_objects.txt"
        self.nbv_result_path = self.params["NBV"]["resultSavePath"] + \
            "/reconstruction"
        self.save_pcl = self.params["NBV"]["savePcl"]

        self.max_nbv_calls = self.params["NBV"]["maxNBVCalls"]

        self.selected_objects = []
        self.object_list = []

        with open(self.object_path, "r") as f:
            lines = f.readlines()
            for idx, line in enumerate(lines):
                if idx % 2 == 0:
                    obj = line.strip()
                    self.object_list.append(obj)
                    self.selected_objects.append(self.shape_net_path+"/"+obj)

        self.view_space = np.loadtxt(
            self.search_space_path)
        self.run_planner()

    def make_ready_for_new_iteration(self):

        self.pr.stop()
        self.pr.start()
        self.focus_target = Shape.create(type=PrimitiveShape.SPHERE,
                                         size=[0.2, 0.2, 0.2],
                                         color=[0.0, 0.0, 1.0],
                                         static=True, respondable=False, renderable=False)
        self.num_of_nbv_call = 0
        # Run the roslaunch file which starts octomap server and point cloud generation
        self.run_id = rospy.get_param('/run_id')
        # self.ig_method = "area_factor"
        self.roslaunch_parent = roslaunch.parent.ROSLaunchParent(
            self.run_id, [self.launch_file_path])
        self.roslaunch_parent.start()
        time.sleep(1)

        inputfile = self.gt_path+"/" + \
            self.object_list[self.object_number]+"/gt.npy"

        self.result_save_path = self.nbv_result_path + \
            "/"+self.object_list[self.object_number]

        if not os.path.isdir(self.result_save_path):
            os.makedirs(self.result_save_path)

        self.gt_pcl = np.load(inputfile)
        self.gt_pcl = self.gt_pcl[self.gt_pcl[:, 2] > 0.01]
        self.total_point = self.gt_pcl.shape[0]
        self.current_point = 0
        self.current_point_torch = 0
        self.previous_position_focus = None
        self.coverage_list = []
        self.entropy_list = []
        self.unknown_list = []
        self.occupied_list = []
        self.ig_max_list = []
        self.free_list = []
        self.cov_volume_list = []
        self.distance_list = []
        self.nbv_list = []
        self.nbv_time_list = []
        self.nbv_current_list = []
        self.travel_time_list = []
        self.robot_motion_time_list = []
        self.termination_cause_list = []
        self.plane_distances = []
        self.rrt_shape_list = []
        self.ipp_shape_list = []

        # GPU Operations
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.gt_pcl_torch = torch.from_numpy(self.gt_pcl).to(self.device)

        # Set the reconstruction object pose
        poseInputfile = self.gt_path+"/" + \
            self.object_list[self.object_number]+"/gt_pose.npy"

        self.rec_ob_pose = np.load(poseInputfile)
        mesh_path = self.selected_objects[self.object_number]+"/model.obj"
        self.rec_obj_ref = Shape.import_mesh(
            mesh_path, scaling_factor=5.8, ignore_up_vector=True)
        self.rec_obj_ref.is_renderable = True
        self.comm_agent.set_object_pose(self.rec_obj_ref, DQ(self.rec_ob_pose))

        self.pr.step()

        for _ in range(20):
            self.pr.step()
        time.sleep(5)

        if self.enable_visibility_constraint:
            if self.enable_focus_point:
                self.used_strategy = "focus_point"
            else:
                self.used_strategy = "fixed_vis"
        elif self.enable_rrt_based_ipp:
            self.used_strategy = "sampling_based"
        else:
            self.used_strategy = "no_path"

        # For termintation criteria, we can terminate if the ig values do not change more than %5 of the first maximum ig
        self.previous_ig_result = np.zeros(self.view_space.shape[0])
        self.first_ig_val = 0
        self.ig_threshold = 0
        self.remaining_views = self.view_space

        self.nbv_start_time = time.time()

    def run_planner(self):
        method_counter = 0  # To switch between methods automatically
        ##
        while True:
            self.change_method(method_counter % 3)
            self.make_ready_for_new_iteration()
            while not self.num_of_nbv_call == self.max_nbv_calls:

                # Calculate IG of views
                prev_time = time.time()
                tmp_list = self.remaining_views.flatten()
                req_focus = np.array(tmp_list)
                # Create a service request
                request = view_evaluate_srvRequest()
                request.view_list = req_focus
                request.ig_method = "RSV"
                response = self.view_ig_func(request)
                self.nbv_time = time.time()-prev_time
                print("Elapsed Time for NBV:", time.time()-prev_time)
                view_ig_result = np.array(response.view_igs)

                # Find the best view
                max_ig = np.max(view_ig_result)
                self.ig_max_list.append(max_ig)
                best_view_idx = np.argmax(view_ig_result)
                best_view = self.remaining_views[best_view_idx]
                # Delete the best view for the next iteration
                self.remaining_views = np.delete(
                    self.remaining_views, (best_view_idx), axis=0)

                # Go to the best view
                self.move_to_nbv(best_view)

            method_counter += 1
            # Use the next object after finishing all three methods
            if method_counter >= 1 and method_counter % 3 == 0:
                self.object_number += 1

    def change_method(self, idx):
        if idx == 0:  # Informative focus point method
            self.enable_visibility_constraint = True
            self.enable_focus_point = True
            self.enable_rrt_based_ipp = False
            self.max_time_limit = 120
        if idx == 1:  # IPP method
            self.enable_visibility_constraint = False
            self.enable_focus_point = False
            self.enable_rrt_based_ipp = True
            self.max_time_limit = 20
        if idx == 2:  # No Path Method
            self.enable_visibility_constraint = False
            self.enable_focus_point = False
            self.enable_rrt_based_ipp = False
            self.max_time_limit = 120
        if idx == 3:  # Focus the centre
            self.enable_visibility_constraint = True
            self.enable_focus_point = False
            self.enable_rrt_based_ipp = False
            self.max_time_limit = 120

    def get_base_obstacles(self):
        collision_cylinder = PyRepObj.Object.get_object(
            "collision_cylinder")
        cylinder_pose = self.comm_agent.get_object_pose(collision_cylinder)
        base_obstacles = [cylinder_pose]
        # Get all the base obstacle positions
        for i in range(self.num_of_base_obstacles):
            cylinder = PyRepObj.Object.get_object(
                f"base_obs_{i+1}")
            pose = self.comm_agent.get_object_pose(cylinder)
            base_obstacles.append(pose)
        return base_obstacles

    def calculate_surface_coverage_torch(self, partial_pcl_in):
        z_min = 0.01
        # a = self.gt_pcl[:, 2] > z_min

        partial_pcl_in = partial_pcl_in[partial_pcl_in[:, 2] > z_min]
        max_pcl_points = 100
        iter_count = math.floor(partial_pcl_in.shape[0]/max_pcl_points)
        remaining = partial_pcl_in.shape[0] % max_pcl_points

        # Modify matrixes
        for i in range(iter_count):
            gt_pcl_torch = self.gt_pcl_torch.unsqueeze(1)

            partial_pcl_torch = torch.from_numpy(partial_pcl_in[int(
                i*max_pcl_points):int((i+1)*max_pcl_points)]).to(self.device)

            partial_pcl_torch = partial_pcl_torch.unsqueeze(0)
            pcl_diff_torch = gt_pcl_torch-partial_pcl_torch
            pcl_square_torch = pcl_diff_torch*pcl_diff_torch
            pcl_dist_torch = torch.sqrt(torch.sum(pcl_square_torch, dim=-1))
            pcl_accepted_torch = torch.sum(pcl_dist_torch <= 0.008, dim=1)
            num_of_accepted = torch.sum(pcl_accepted_torch).cpu().item()
            self.current_point_torch += num_of_accepted
            pcl_remaining_torch = pcl_accepted_torch == 0
            self.gt_pcl_torch = self.gt_pcl_torch[pcl_remaining_torch]

        if remaining != 0:

            gt_pcl_torch = self.gt_pcl_torch.unsqueeze(1)

            partial_pcl_torch = torch.from_numpy(partial_pcl_in[int(
                iter_count*max_pcl_points):]).to(self.device)

            partial_pcl_torch = partial_pcl_torch.unsqueeze(0)
            pcl_diff_torch = gt_pcl_torch-partial_pcl_torch
            pcl_square_torch = pcl_diff_torch*pcl_diff_torch
            pcl_dist_torch = torch.sqrt(torch.sum(pcl_square_torch, dim=-1))

            pcl_accepted_torch = torch.sum(pcl_dist_torch <= 0.008, dim=1)
            num_of_accepted = torch.sum(pcl_accepted_torch).cpu().item()
            self.current_point_torch += num_of_accepted
            pcl_remaining_torch = pcl_accepted_torch == 0
            self.gt_pcl_torch = self.gt_pcl_torch[pcl_remaining_torch]

        return self.current_point_torch/self.total_point

    def move_to_nbv(self, nbv_pose):

        time_start_nbv = time.time()
        nbv_time = time.time()-self.nbv_start_time
        self.nbv_time_list.append(nbv_time)

        nbv_position = nbv_pose[0]*i_ + \
            nbv_pose[1]*j_+nbv_pose[2]*k_
        nbv_ori = normalize(nbv_pose[3]+nbv_pose[4]*i_ +
                            nbv_pose[5]*j_+nbv_pose[6]*k_)
        # To match the camera pose in sim with NBV rotate around z
        target_nbv = (nbv_ori+0.5*E_*nbv_position*nbv_ori)

        self.nbv_list.append([nbv_pose[0], nbv_pose[1], nbv_pose[2],
                              nbv_pose[3], nbv_pose[4], nbv_pose[5], nbv_pose[6]
                              ])
        # Move NBV marker in sim to NBV
        self.comm_agent.set_object_pose(self.nbv_obj, target_nbv)
        self.pr.step()
        if self.enable_rrt_based_ipp:
            # Get the path to NBV
            is_plan_found = False
            while not is_plan_found:
                nbv_position_np = vec3(translation(target_nbv))
                current_position = self.camera_ref.get_position()

                ipp = IPP(start=[current_position[0], current_position[1], current_position[2]],
                          goal=[nbv_position_np[0], nbv_position_np[1], nbv_position_np[2]], max_iter=5000, expand_dis=0.5)
                is_plan_found, rrt_result_dic = ipp.get_ipp_result()
                if not is_plan_found:
                    print("RRT Failed")

            rrt_path = []
            dist_threshold = 1
            for rrt_pstn in np.array(rrt_result_dic["rrt_positions"]):
                dist_to_start = np.linalg.norm(rrt_pstn-current_position)
                dist_to_target = np.linalg.norm(rrt_pstn-nbv_position_np)
                if not (dist_to_start < dist_threshold or dist_to_target < dist_threshold):
                    rrt_path.append(rrt_pstn)
            rrt_path = np.array(rrt_path)[::-1]
            # Remove shapes belonging to the previous iteration
            if len(self.rrt_shape_list) != 0:
                for trgt in self.rrt_shape_list:
                    trgt.remove()
                    self.pr.step()
                self.rrt_shape_list = []

            # Show all samples generated for RRT
            if self.params["Visual"]["showAllSamples"]:
                for sample_position in rrt_result_dic["rrt_generated_samples"][:100]:
                    target = Shape.create(type=PrimitiveShape.SPHERE,
                                          size=[0.1, 0.1, 0.1],
                                          color=[1.0, 1.0, 0.0],
                                          static=True, respondable=False, renderable=False)
                    target.set_position(sample_position)
                    self.rrt_shape_list.append(target)
                    self.pr.step()

                for _ in range(100):
                    self.pr.step()
                    time.sleep(0.01)

            if self.params["Visual"]["showAllNodes"]:
                for rrt_node in rrt_result_dic["rrt_node_list"][:50]:
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
                        a = prnt_np[i+1]-prnt_np[i]
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
                        quat = direction_to_pose(dir_cyl)
                        pose_cyl_dq = (quat+E_*0.5*position_cyl_dq*quat)
                        pose_cyl_dq = normalize(pose_cyl_dq)
                        target_a1.set_position(prnt_np[i])
                        target_a2.set_position(prnt_np[i+1])

                        self.comm_agent.set_object_pose(
                            target_cyl, pose_cyl_dq)
                        self.pr.step()
                        self.rrt_shape_list.append(target_a1)
                        self.rrt_shape_list.append(target_a2)
                        self.rrt_shape_list.append(target_cyl)

                for path_node_position in rrt_path:
                    target = Shape.create(type=PrimitiveShape.SPHERE,
                                          size=[0.1, 0.1, 0.1],
                                          color=[0.0, 1.0, 0.0],
                                          static=True, respondable=False, renderable=False)
                    target.set_position(path_node_position)
                    self.rrt_shape_list.append(target)
                    self.pr.step()
            # Remove shapes belonging to the previous iteration
            if len(self.rrt_shape_list) != 0:
                for trgt in self.rrt_shape_list:
                    trgt.remove()
                    self.pr.step()
                self.rrt_shape_list = []
            # Show the best RRT path
            if self.params["Visual"]["showRrtPath"]:
                for path_node_position in rrt_path:
                    target = Shape.create(type=PrimitiveShape.SPHERE,
                                          size=[0.1, 0.1, 0.1],
                                          color=[0.0, 1.0, 0.0],
                                          static=True, respondable=False, renderable=False)
                    target.set_position(path_node_position)
                    self.rrt_shape_list.append(target)
                    self.pr.step()

        # If it is the first NBV, save data for the initial state of the robot
        if self.num_of_nbv_call == 0:
            # Get the current entropy (robot took measurements from the start pose)
            response = self.coverage_func()
            print("Current Entropy", response.ent)
            self.entropy_list.append(response.ent)
            # Calculate surface coverage
            ros_point_cloud = rospy.wait_for_message(
                "/octomap_point_cloud_centers", PointCloud2)
            gen = pc2.read_points(ros_point_cloud, skip_nans=True)
            int_data = list(gen)
            xyz = []
            for x in int_data:
                xyz.append([*x])
            xyz_np = np.array(xyz)
            t1 = time.time()
            cov = self.calculate_surface_coverage_torch(xyz_np)
            self.coverage_list.append(cov)
            # Save pcl data for the current partial model
            if self.save_pcl:
                self.pcl_save_path = self.result_save_path + \
                    f"/{self.used_strategy}_pcl"
                if not os.path.isdir(self.pcl_save_path):
                    os.makedirs(self.pcl_save_path)
                self.pcl_save_path_name = self.pcl_save_path + \
                    f"/data_{self.num_of_nbv_call}"
                np.save(self.pcl_save_path_name, xyz_np)

        # Calculate distance between current camera (robot eef) position and NBV
        current_position = self.camera_ref.get_position()
        next_position = np.array(
            [nbv_pose[0], nbv_pose[1], nbv_pose[2]])
        nbv_dist = np.linalg.norm(current_position-next_position)
        self.distance_list.append(nbv_dist)

        # To calculate focus point if it is activated
        trans = self.camera_ref.get_position()
        orient = self.camera_ref.get_quaternion()
        dir_line = np.array([0-trans[0], 0-trans[1], 0])
        quat = direction_to_pose(dir_line)
        pose_cam = quat+E_*0.5 * (trans[0]*i_+trans[1]*j_+trans[2]*k_)*quat
        trans = vec4(translation(pose_cam))[1:]
        orient = vec4(rotation(pose_cam))
        orient = [orient[1], orient[2], orient[3], orient[0]]

        if self.enable_focus_point:
            req_focus = np.array([trans[0], trans[1], trans[2],
                                  orient[3], orient[0], orient[1], orient[2]])
            response = self.focus_pnt_func(req_focus)
            fcs_pnt = np.array(response.focus_pnt)

            self.focus_target.set_position(fcs_pnt)
            vis_target_position = fcs_pnt[0]*i_+fcs_pnt[1]*j_+fcs_pnt[2]*k_
            print("FOCUS TARGET")
            print(vis_target_position)
            self.previous_position_focus = trans
        else:
            vis_target_position = 0.45*k_
            self.focus_target.set_position([0, 0, 0.45])
            self.pr.step()

        # initial_focus = self.comm_agent.get_object_pose(
        #     self.camera_ref)*(1+E_*0.5*2.5*k_)
        # vis_target_position = translation(initial_focus)
        self.previous_position_focus = self.camera_ref.get_position()
        self.focus_prev_ref.set_position(self.previous_position_focus)
        if self.enable_rrt_based_ipp:
            num_of_path_views = rrt_path.shape[0]
        else:
            num_of_path_views = 0

        for idx in range(num_of_path_views+1):

            if idx < num_of_path_views:
                rrt_path_position = rrt_path[idx]
                # Sample 10 views around the node
                sphere_sample_list = ipp.sample_in_cylinder(
                    rrt_path_position)

                # Remove shapes
                if len(self.ipp_shape_list) != 0:
                    for trgt in self.ipp_shape_list:
                        trgt.remove()
                        self.pr.step()
                    self.ipp_shape_list = []

                # Show the samples generated around rrt node
                if self.params["Visual"]["showSphereSamples"]:
                    for sample_pose in sphere_sample_list:
                        target = Shape.create(type=PrimitiveShape.SPHERE,
                                              size=[0.05, 0.05, 0.05],
                                              color=[0.0, 0.0, 1.0],
                                              static=True, respondable=False, renderable=False)
                        self.comm_agent.set_object_pose(target, sample_pose)
                        self.ipp_shape_list.append(target)
                        self.pr.step()

                # Make the views one vector for the ros server request
                view_list_srv = []
                for dq_pose in sphere_sample_list:
                    trans = vec3(translation(dq_pose))
                    orient = vec4(rotation(dq_pose))
                    tmp_list = [trans[0], trans[1], trans[2],
                                orient[0], orient[1], orient[2], orient[3]]
                    view_list_srv += tmp_list

                # Calculate IG of views
                prev_time = time.time()
                req_focus = np.array(view_list_srv)

                request = view_evaluate_srvRequest()
                request.view_list = req_focus
                request.ig_method = "ENT"
                response = self.view_ig_func(request)
                view_ig_result = np.array(response.view_igs)

                # Find the best view
                best_view_idx = np.argmax(view_ig_result)
                best_view = sphere_sample_list[best_view_idx]

                # Remove shapes
                if len(self.ipp_shape_list) != 0:
                    for trgt in self.ipp_shape_list:
                        trgt.remove()
                        self.pr.step()
                    self.ipp_shape_list = []
                # Set the robot target pose
                robot_target_pose = best_view
                # Show the best sample (current target for the robot)
                self.comm_agent.set_object_pose(
                    self.ipp_taraget, robot_target_pose)
                self.pr.step()
            else:
                robot_target_pose = target_nbv

            err = 1
            counter = 0
            previous_err = 1
            stability_count = 0
            self.movement_start = time.time()
            termination_cause = "CONVERGE"
            youbot_q = self.robot_model.get_q_from_sim()
            while err > self.controller_err_tolerance:
                trans = self.camera_ref.get_position()
                if np.linalg.norm(trans[:2]) > 5:
                    self.enable_visibility_constraint = False
                    self.robot_to_far = True
                if self.robot_to_far and np.linalg.norm(trans[:2]) < 3.5:
                    self.robot_to_far = False
                    self.enable_visibility_constraint = True

                dist_to_goal = np.linalg.norm(
                    trans-vec3(translation(robot_target_pose)))
                dist_to_previous = np.linalg.norm(
                    trans-self.previous_position_focus)
                counter += 1
                # Update the focus point if the distance between current eef position and the position
                # where previous focus point determined is higher than a certain value.
                if self.enable_focus_point and dist_to_previous > self.params["NBV"]["focusUpdateDistance"]:
                    counter = 0
                    trans = self.camera_ref.get_position()
                    orient = self.camera_ref.get_quaternion()
                    # To focus centre
                    dir_line = np.array([0-trans[0], 0-trans[1], 0])
                    quat = direction_to_pose(dir_line)
                    pose_cam = quat+E_*0.5 * \
                        (trans[0]*i_+trans[1]*j_+trans[2]*k_)*quat
                    trans = vec4(translation(pose_cam))[1:]
                    orient = vec4(rotation(pose_cam))
                    orient = [orient[1], orient[2], orient[3], orient[0]]

                    req_focus = np.array([trans[0], trans[1], trans[2],
                                          orient[3], orient[0], orient[1], orient[2]])
                    st1 = time.time()
                    response = self.focus_pnt_func(req_focus)
                    print("Focus Point Time:", time.time()-st1)
                    fcs_pnt = np.array(response.focus_pnt)
                    vis_target_position = fcs_pnt[0] * \
                        i_+fcs_pnt[1]*j_+fcs_pnt[2]*k_
                    print("FOCUS TARGET")
                    print(vis_target_position)

                    self.focus_target.set_position(fcs_pnt)
                    self.previous_position_focus = trans
                    self.focus_prev_ref.set_position(
                        self.previous_position_focus)
                    self.pr.step()

                youbot_q = self.robot_model.get_q_from_sim()
                pose_J = self.robot_model.Kinematics.pose_jacobian(youbot_q)
                pose_ = self.robot_model.Kinematics.fkm(youbot_q)
                line_J = self.robot_model.Kinematics.line_jacobian(
                    pose_J, pose_, k_)

                line_J = line_J[1:4]
                tra_J = self.robot_model.Kinematics.translation_jacobian(
                    pose_J, pose_)
                tra_J = tra_J

                pose_err = vec8(pose_) - vec8(robot_target_pose)
                tra_err = vec4(translation(pose_) -
                               translation(robot_target_pose))
                # eef_line = vec8(Ad(pose_, k_))
                eef_line = vec3(get_direction(pose_, k_))
                # target_line = vec8(Ad(robot_target_pose, k_))
                target_line = vec3(get_direction(robot_target_pose, k_))
                direction_err = eef_line - target_line

                if self.is_pose_control:
                    whole_J = pose_J
                    whole_err = pose_err
                else:
                    whole_J = np.vstack([line_J, tra_J])
                    whole_err = np.vstack([direction_err[:, np.newaxis],
                                           tra_err[:, np.newaxis]])

                hard_const_J = []
                hard_const_b = []

                if self.enable_visibility_constraint:
                    W_visibility = self.soft_alpha * \
                        np.linalg.norm(tra_err)**self.soft_beta
                    # Expand _jacobian matrix to include slack variable
                    slack_count = 4
                    whole_J_slack = np.block([
                        [whole_J, np.zeros((whole_J.shape[0], slack_count))],
                        [np.zeros((slack_count, whole_J.shape[1])),
                            W_visibility * np.eye(slack_count)]
                    ])
                    whole_err_slack = np.vstack(
                        [whole_err, np.zeros((slack_count, 1))])

                    # Calculate the pose of the planes w.r.t world frame, and their plane jacobians
                    left_plane_pose, left_plane_jacob = calculate_plane_jacobian(
                        self.robot_model.Kinematics, youbot_q, self.left_pose)
                    right_plane_pose, right_plane_jacob = calculate_plane_jacobian(
                        self.robot_model.Kinematics, youbot_q, self.right_pose)
                    up_plane_pose, up_plane_jacob = calculate_plane_jacobian(
                        self.robot_model.Kinematics, youbot_q, self.up_pose)
                    down_plane_pose, down_plane_jacob = calculate_plane_jacobian(
                        self.robot_model.Kinematics, youbot_q, self.down_pose)

                    b_constraint_im = []
                    left_plane = pose_to_plane(left_plane_pose, k_)
                    left_dist_jacob = self.robot_model.Kinematics.plane_to_point_distance_jacobian(
                        left_plane_jacob, vis_target_position)
                    dist2left = DQ_Geometry.point_to_plane_distance(
                        vis_target_position, left_plane) - self.safe_dist_visibility
                    self.plane_distances.append(dist2left)

                    j_constraint_im = np.array(left_dist_jacob)
                    b_constraint_im.append(dist2left)

                    right_plane = pose_to_plane(right_plane_pose, k_)
                    right_dist_jacob = self.robot_model.Kinematics.plane_to_point_distance_jacobian(
                        right_plane_jacob, vis_target_position)
                    dist2right = DQ_Geometry.point_to_plane_distance(
                        vis_target_position, right_plane) - self.safe_dist_visibility
                    self.plane_distances.append(dist2right)
                    j_constraint_im = np.concatenate(
                        (j_constraint_im, right_dist_jacob), axis=0)
                    b_constraint_im.append(dist2right)

                    up_plane = pose_to_plane(up_plane_pose, k_)
                    up_dist_jacob = self.robot_model.Kinematics.plane_to_point_distance_jacobian(
                        up_plane_jacob, vis_target_position)
                    dist2up = DQ_Geometry.point_to_plane_distance(
                        vis_target_position, up_plane) - self.safe_dist_visibility
                    self.plane_distances.append(dist2up)
                    j_constraint_im = np.concatenate(
                        (j_constraint_im, up_dist_jacob), axis=0)
                    b_constraint_im.append(dist2up)

                    down_plane = pose_to_plane(down_plane_pose, k_)
                    down_dist_jacob = self.robot_model.Kinematics.plane_to_point_distance_jacobian(
                        down_plane_jacob, vis_target_position)
                    dist2down = DQ_Geometry.point_to_plane_distance(
                        vis_target_position, down_plane) - self.safe_dist_visibility
                    j_constraint_im = np.concatenate(
                        (j_constraint_im, down_dist_jacob), axis=0)
                    self.plane_distances.append(dist2down)
                    b_constraint_im.append(dist2down)
                    b_constraint = np.array(b_constraint_im)

                    J_soft_1 = np.concatenate((j_constraint_im, np.zeros(
                        (j_constraint_im.shape[0], slack_count))), axis=1)
                    J_soft_2 = np.concatenate((np.zeros_like(j_constraint_im), np.eye(
                        j_constraint_im.shape[0], slack_count)), axis=1)
                    J_soft = -J_soft_1-J_soft_2
                    b_soft = b_constraint
                    J_slack = -J_soft_2
                    b_slack = np.array(slack_count*[0])

                    hard_const_J.append(J_soft)
                    hard_const_J.append(J_slack)
                    if self.enable_visibility_constraint:
                        hard_const_b.append(b_soft)
                        hard_const_b.append(b_slack)

                if self.enable_joint_limits:
                    # Import joint constraints
                    # Limit the velocity
                    J_limit_vel = np.concatenate(
                        (np.eye(8, 8), -np.eye(8, 8)), axis=0)
                    b_limit_vel = np.concatenate(
                        (self.total_vel_lim_max, -self.total_vel_lim_min), axis=0)

                    # Limit the positions
                    B_mat = np.concatenate(
                        (np.zeros((5, 3)), np.eye(5)), axis=1)
                    J_limit_angle = np.concatenate((B_mat, -B_mat), axis=0)
                    b_limit_angle = np.concatenate(
                        (-(youbot_q[3:]-self.joint_pos_max), (youbot_q[3:]-self.joint_pos_min)), axis=0)
                    J_limit_whole = np.concatenate(
                        (J_limit_vel, J_limit_angle), axis=0)

                    if self.enable_visibility_constraint:
                        J_limit_whole = np.concatenate(
                            (J_limit_whole, np.zeros((J_limit_whole.shape[0], slack_count))), axis=1)

                    b_limit_whole = np.concatenate(
                        (b_limit_vel, b_limit_angle), axis=0)
                    hard_const_J.append(J_limit_whole)
                    hard_const_b.append(b_limit_whole)
                # Add the constraint for the workspace always

                if self.enable_collision_constraints:
                   # Compute base collision constraints
                    pose_list = self.base_obstacle_poses
                    radius_list = [2.75]+self.num_of_base_obstacles*[0.1]
                    J_base, b_base = compute_base_constraints(pose_list=pose_list, radius_list=radius_list,
                                                              robot_kin=self.robot_model.Kinematics, youbot_q=youbot_q)
                    if self.enable_visibility_constraint:
                        J_base = np.concatenate(
                            (J_base, np.zeros((J_base.shape[0], slack_count))), axis=1)

                    hard_const_J.append(-J_base)
                    hard_const_b.append(b_base)

                    # Compute eef collision constraint
                    cylinder_dq = pose_to_line(pose_list[0], k_)
                    position_eef = translation(pose_)
                    J_eef = self.robot_model.Kinematics.point_to_line_distance_jacobian(
                        tra_J, position_eef, cylinder_dq)

                    if self.enable_visibility_constraint:
                        J_eef = np.concatenate(
                            (J_eef, np.zeros((J_eef.shape[0], slack_count))), axis=1)

                    b_eef = DQ_Geometry.point_to_line_squared_distance(
                        position_eef, cylinder_dq) - 2.75**2
                    hard_const_J.append(-J_eef)
                    hard_const_b.append(np.array([b_eef]))

                else:
                    # Compute base collision constraints
                    pose_list = [self.base_obstacle_poses[0]]
                    radius_list = [2.75]
                    J_base, b_base = compute_base_constraints(pose_list=pose_list, radius_list=radius_list,
                                                              robot_kin=self.robot_model.Kinematics, youbot_q=youbot_q)
                    if self.enable_visibility_constraint:
                        J_base = np.concatenate(
                            (J_base, np.zeros((J_base.shape[0], slack_count))), axis=1)

                    hard_const_J.append(-J_base)
                    hard_const_b.append(b_base)

                    # Compute eef collision constraint
                    cylinder_dq = pose_to_line(pose_list[0], k_)
                    position_eef = translation(pose_)
                    J_eef = self.robot_model.Kinematics.point_to_line_distance_jacobian(
                        tra_J, position_eef, cylinder_dq)

                    if self.enable_visibility_constraint:
                        J_eef = np.concatenate(
                            (J_eef, np.zeros((J_eef.shape[0], slack_count))), axis=1)

                    b_eef = DQ_Geometry.point_to_line_squared_distance(
                        position_eef, cylinder_dq) - 2.75**2
                    hard_const_J.append(-J_eef)
                    hard_const_b.append(np.array([b_eef]))

                if len(hard_const_J) == 0:
                    u = self.qp_solver.compute_control_signal(
                        whole_J, whole_err, Aineq=None, bineq=None, gain=self.controller_gain,
                        damping=self.controller_damping)
                else:
                    hard_const_J = np.concatenate(hard_const_J, axis=0)
                    hard_const_b = np.concatenate(hard_const_b, axis=0)
                    if self.enable_visibility_constraint:
                        u = self.qp_solver.compute_control_signal(
                            whole_J_slack, whole_err_slack, Aineq=hard_const_J, bineq=hard_const_b, gain=self.controller_gain,
                            damping=self.controller_damping)
                    else:
                        u = self.qp_solver.compute_control_signal(
                            whole_J, whole_err, Aineq=hard_const_J, bineq=hard_const_b, gain=self.controller_gain,
                            damping=self.controller_damping)

                if self.is_velocity_control:
                    base_phi = youbot_q[2]
                    rot_mat = np.array([[math.cos(base_phi), math.sin(base_phi), 0],
                                        [-math.sin(base_phi),
                                        math.cos(base_phi), 0],
                                        [0, 0, 1]])
                    u = u[:8]
                    base_vel_1 = rot_mat@u[:3]

                    u[:3] = base_vel_1
                    self.robot_model.send_velocities(u)
                else:
                    youbot_q = youbot_q + self.integration_time_step*u[:8]
                    self.robot_model.send_q_to_sim(list(youbot_q))

                self.pr.step()

                err = np.linalg.norm(whole_err)
                tra_err_norm = np.linalg.norm(tra_err)
                dir_err_norm = np.linalg.norm(direction_err)

                youbot_q = self.robot_model.get_q_from_sim()
                dq_position = vec4(translation(
                    self.robot_model.Kinematics.fkm(youbot_q)))[1:]
                real_position = self.camera_ref.get_position()

                # Stability Check
                err_diff = np.linalg.norm(whole_err-previous_err)
                if err_diff < self.controller_stab_threshold:
                    stability_count += 1
                else:
                    stability_count = 0

                if stability_count > self.controller_stab_max:
                    termination_cause = "STABILITY"
                    break
                if (time.time()-self.movement_start) >= self.max_time_limit:
                    termination_cause = "TIME"
                    break
                previous_err = whole_err

        robot_motion_time = time.time()-time_start_nbv
        self.robot_motion_time_list.append(robot_motion_time)
        self.termination_cause_list.append(termination_cause)
        self.travel_time = time.time()-self.movement_start
        self.travel_time_list.append(self.travel_time)
        response = self.coverage_func()

        print("Current Entropy", response.ent)
        self.entropy_list.append(response.ent)
        self.unknown_list.append(response.unknown)
        self.occupied_list.append(response.occupied)
        self.free_list.append(response.free)
        self.cov_volume_list.append(response.cvr)
        ros_point_cloud = rospy.wait_for_message(
            "/octomap_point_cloud_centers", PointCloud2)
        gen = pc2.read_points(ros_point_cloud, skip_nans=True)
        int_data = list(gen)
        xyz = []
        for x in int_data:
            xyz.append([*x])
        xyz_np = np.array(xyz)
        t1 = time.time()
        cov = self.calculate_surface_coverage_torch(xyz_np)
        print("Torch Time:", time.time()-t1)
        print("Current Coverage", cov)
        self.coverage_list.append(cov)
        # Current camera pose
        trans = self.camera_ref.get_position()
        orient = self.camera_ref.get_quaternion()
        current_pose = [trans[0], trans[1], trans[2],
                        orient[3], orient[0], orient[1], orient[2]]
        self.nbv_current_list.append(current_pose)
        if self.save_pcl:
            self.pcl_save_path = self.result_save_path + \
                f"/{self.used_strategy}_pcl"

            if not os.path.isdir(self.pcl_save_path):
                os.makedirs(self.pcl_save_path)
            self.pcl_save_path_name = self.pcl_save_path + \
                f"/data_{self.num_of_nbv_call+1}"
            np.save(self.pcl_save_path_name, xyz_np)

        self.num_of_nbv_call += 1
        print(self.num_of_nbv_call)
        if self.num_of_nbv_call == self.max_nbv_calls:

            data = {
                "coverage_list": self.coverage_list,
                "entropy_list": self.entropy_list,
                "robot_motion_time_total": np.sum(np.array(self.robot_motion_time_list)),
                "robot_motion_time": self.robot_motion_time_list,
                "distance_list": self.distance_list,
                "nbv_list": self.nbv_list,
                "total_nbv": self.num_of_nbv_call,
                "nbv_time_list": self.nbv_time_list,
                "nbv_curent_list": self.nbv_current_list,
                "termination_cause_list": self.termination_cause_list,
                "unknown_list": self.unknown_list,
                "free_list": self.free_list,
                "occupied_list": self.occupied_list,
                "free_list": self.free_list,
                "cov_volume_list": self.cov_volume_list,
                "best_ig_val_list": self.ig_max_list

            }

            file = self.result_save_path + f"/{self.used_strategy}"
            with open(file, 'w') as f:
                json.dump(data, f)
            self.roslaunch_parent.shutdown()
            self.rec_obj_ref.remove()
        self.nbv_start_time = time.time()
        time.sleep(1)
        return True


if __name__ == "__main__":
    print("NBV Evaluation with Robot Started")
    rospy.init_node('nbv_coppelia_node', anonymous=True)
    with open("nbv_coppelia/Config/config.yaml", 'r') as configFile:
        configs = yaml.safe_load(configFile)
    nbv_obj = NBV(configs)
    rate = rospy.Rate(configs["NBV"]["nodeFreq"])
    rospy.spin()
