from Utils.manage_visuals import ManageVisuals
import rospy
import numpy as np
from pyrep import PyRep
import math
import enum
from os.path import join, dirname, abspath
from dqrobotics import *
from RobotModel.youbot_model import YouBotModel
from RobotModel.dq_pyrep import BaseCommunication
from Utils.utils import *
import yaml
from Utils.manage_data import ManageData
from orac_reconstruction_services.srv import focus_point_srv, focus_point_srvRequest, coverage_srv, \
    view_evaluate_srv, view_evaluate_srvRequest, save_octomap_srv, save_octomap_srvRequest
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from Utils.random_env_generator import RandomEnv
import roslaunch
from RobotController.controller import RobotController
from SamplingBasedIPP.plan_inspection_path import IPP
import sys
import atexit
from copy import deepcopy
import time
import os

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class TerminationReason(enum.Enum):
    SUCCESS = 0
    STABILITY_ERROR = 1


class MainClass():
    def __init__(self, params):
        self.pr = PyRep()
        self.params = params
        self.launch_simulation()

        # Model to interact with the YouBot robot. It has kinematic model and methods to get/set pose and joint positions/velocities
        self.robot_dim = 0.7
        self.robot_model = YouBotModel(self.robot_dim)
        # Communication agent to convert data between dqrobotics and CoppeliaSim
        self.comm_agent = BaseCommunication()

        # it manages the visuals in CoppeliaSim
        self.visual_manager = ManageVisuals(self.pr)
        self.data_manager = ManageData(self.params)
        self.robot_controller = RobotController(self.params)
        # Load candidate views from file as numpy array
        self.candidate_views = self.data_manager.get_candidate_views()

        # Initialize ROS services
        self.init_ros_services()
        # Initialize random environment generator
        self.search_space_radius = 3
        self.search_space_center = np.array([0, 0])
        self.offset_workspace_planes = 1
        self.init_random_env_generator()
        self.set_camera_params()
        self.run_planner()

    def set_camera_params(self):
        """ Set the depth camera parameters in CoppeliaSim based on the ROS parameters. """
        res = [self.params["CameraParameters"]["imWidth"],
               self.params["CameraParameters"]["imHeight"]]
        persp_angle = self.params["CameraParameters"]["camFov"]
        near_clip = self.params["CameraParameters"]["nearClip"]
        far_clip = self.params["CameraParameters"]["farClip"]
        self.visual_manager.set_depth_camera_parameters(
            res, persp_angle, near_clip, far_clip)

    def find_closest_obstacle(self, robot_position: np.ndarray) -> tuple:
        """ Find the closest obstacle to the robot position.
        Args:
            robot_position (np.ndarray): The position of the robot as a numpy array [x, y].
        Returns:
            tuple: A tuple containing the index of the closest obstacle and its distance to the robot.
        """
        dist = np.array(self.obs_position_list)-robot_position
        dists = np.linalg.norm(dist, axis=1) - \
            np.array(self.obs_rad_list)-self.robot_dim/2
        closest_idx = np.argmin(dists)

        return (closest_idx, dists[closest_idx])

    def initiate_new_object(self, object_index=0):
        self.robot_initial_base_q = self.robot_model.get_q_from_sim()[:2]
        if self.params["NBV"]["includeObstacles"]:
            self.obs_info_dic = self.rand_env_gen.generate_random_env(
                robot_position=self.robot_initial_base_q)
        else:
            self.obs_info_dic = {
                "obs_pose_list_dq": [],
                "obs_pose_list": [],
                "obs_rad_list": [],
                "obs_wall_state_list": []
            }
        self.obs_pose_list_dq = self.obs_info_dic["obs_pose_list_dq"]
        self.obs_position_list = self.obs_info_dic["obs_pose_list"]
        self.obs_rad_list = self.obs_info_dic["obs_rad_list"]
        self.obs_wall_state_list = self.obs_info_dic["obs_wall_state_list"]
        self.visual_manager.show_obstacles(
            self.obs_pose_list_dq, self.obs_rad_list)  # Show the obstacles in CoppeliaSim

        # Load the object data
        self.gt_pose, self.gt_pcl = self.data_manager.get_gt_data(
            object_index)  # Groundtruth Point cloud and pose
        self.mesh_path = self.data_manager.get_mesh_path(object_index)
        # Create the object visual in CoppeliaSim
        self.object_mesh_handle = self.visual_manager.create_shape_visual(
            self.mesh_path, DQ(self.gt_pose), scale=5.8)
        self.pr.step()
        # time.sleep(1)  # Wait for a second to ensure everything is stable

    def initiate_new_iteration(self, object_index):
        # Refresh the simulation to reset the robot pose to its initial position
        self.pr.stop()
        self.pr.start()
        # First bring up the object shape to the scene and load related gt data
        self.initiate_new_object(object_index)
        # For each method, we do following operations:
        self.launch_ros_session()  # Launch a new ROS session to reset octomap and related stuff

        # Wait for a second to ensure everything is stable
        # Reset the remaining views
        self.remaining_views = deepcopy(self.candidate_views)
        # Reset the ground truth remaining point cloud
        self.gt_pcl_remaining = deepcopy(self.gt_pcl)
        self.current_partial_pcl = np.empty((0, 3))

        # Reset the per method variables
        self.coverage_for_all_nbvs = []
        self.entropy_for_all_nbvs = []
        self.time_for_all_nbvs = []
        self.total_path_length_for_all_nbvs = []
        self.control_generation_time_for_all_nbvs = []
        self.robot_motion_time_for_all_nbvs = []
        self.chamfer_distance_for_all_nbvs = []
        self.max_ig_for_all_nbvs = []
        self.robot_path_length_for_all_nbvs = []
        self.termination_reason_for_all_nbvs = []
        self.iteration_counter_for_all_nbvs = []
        # Related only focus point method
        self.total_focus_time_for_all_nbvs = []
        self.total_focus_calculations_for_all_nbvs = []

        # Related only sampling method
        self.total_view_evaluation_time_for_all_nbvs = []
        self.total_num_of_view_evaluations_for_all_nbvs = []
        for _ in range(40):
            self.pr.step()  # Step the simulation to ensure visuals are updated
        time.sleep(5)  # Wait for a second to ensure everything is stable

    def save_current_octomap(self, object_index, method, nbv_index):
        """ Save the current octomap to a file using the ROS service.
        Args:
            filename (str): The name of the file to save the octomap to.
        """
        """Save the partial model data for the object, method and nbv_index to results folder."""
        result_save_path = self.data_manager.result_save_path
        object_name_list = self.data_manager.get_object_name_list()

        file_name = join(result_save_path,
                         object_name_list[object_index],  method, "octomap_data")

        os.makedirs(file_name, exist_ok=True)
        file_name = join(file_name, f"nbv_{nbv_index}.bt")
        try:
            request = save_octomap_srvRequest()
            request.file_path = file_name
            response = self.octomap_save_func(request)
            if response.success:
                print(f"Octomap saved successfully to {file_name}")
            else:
                print(f"Failed to save octomap to {file_name}")
        except Exception as e:
            print(f"Error in saving octomap: {e}")

    def run_planner(self):
        """Run the NBV planner for all objects and methods."""
        for object_index in range(0, self.data_manager.length_object_list(), 1):

            for method in ["FOCUS", "SAMPLING", "NOPATH"]:
                self.initiate_new_iteration(object_index)

                # Before starting NBV iterations, calculate initial coverage
                #  resulted from initial measurement without visiting any NBV
                total_coverage, chamfer_distance = self.calculate_coverage()
                # Other results calculated from octomap (entropy, number of unknown, occupied, free voxels, covered volume)
                octomap_results = self.calculate_entropy()
                # Calculate initial robot pose
                initial_q = self.robot_model.get_q_from_sim()
                initial_pose_dq = self.robot_model.Kinematics.fkm(
                    initial_q)
                initial_position = vec3(translation(initial_pose_dq))
                initial_orientation = vec4(rotation(initial_pose_dq))
                initial_pose_np = np.concatenate(
                    (initial_position, initial_orientation), axis=0)
                initial_result_dic = {
                    "total_coverage": total_coverage,
                    "octomap_results": octomap_results,
                    "initial_robot_pose": initial_pose_np.tolist(),
                }
                # Add to the method result list
                self.coverage_for_all_nbvs.append(total_coverage)
                self.chamfer_distance_for_all_nbvs.append(chamfer_distance)
                self.entropy_for_all_nbvs.append(octomap_results["entropy"])

                # Save initial result as nbv_0.json
                self.data_manager.save_nbv_result(
                    object_index, 0, initial_result_dic, method)
                if self.params["NBV"]["savePcl"]:
                    self.data_manager.save_partial_model_data(
                        object_index, 0, self.current_partial_pcl, method)

                if self.params["NBV"]["saveOctomap"]:
                    self.save_current_octomap(object_index, method, 0)

                for nbv_cntr in range(self.params["NBV"]["maxNBVCalls"]):
                    cam_res, cam_fovs, near_clip, far_clip = self.visual_manager.get_depth_camera_parameters()
                    print(
                        f"Camera Resolution: {cam_res}, FOVs: {cam_fovs}, clip planes: {near_clip}, {far_clip}    ")

                    # Estimate the next best view using RSV method

                    max_ig, nbv_np, best_view_idx, time_for_nbv_calculation = self.evaluate_candidate_views(
                        candidate_views=self.remaining_views, ig_method="RSV", run_parallel=self.params["NBV"]["parallelComputationEnabled"])

                    # Delete the best view for the next iteration to avoid evaluating it again
                    self.remaining_views = np.delete(
                        self.remaining_views, (best_view_idx), axis=0)

                    # Convert NBV to dq pose
                    nbv_position = nbv_np[0]*i_ + \
                        nbv_np[1]*j_+nbv_np[2]*k_
                    nbv_ori = normalize(nbv_np[3]+nbv_np[4]*i_ +
                                        nbv_np[5]*j_+nbv_np[6]*k_)
                    nbv_pose_dq = (nbv_ori+0.5*E_*nbv_position*nbv_ori)
                    self.visual_manager.set_nbv_frame_pose(nbv_pose_dq)
                    # Before moving, determine circulation direction based on closest obstacle
                    nbv_initial_q = self.robot_model.get_q_from_sim()
                    # Determine circulation direction based on initial position and NBV positions
                    circ_dir = 1 if cross_product_2d(
                        nbv_np[:2], nbv_initial_q[:2]) > 0 else -1

                    motion_start_time = time.time()
                    if method == "FOCUS":
                        result_dic_for_nbv = self.move_to_NBV_focus_point(
                            nbv_pose_dq, nbv_initial_q, circ_dir)
                    elif method == "NOPATH":
                        result_dic_for_nbv = self.move_to_NBV_no_path(
                            nbv_pose_dq, circ_dir)
                    elif method == "SAMPLING":
                        result_dic_for_nbv = self.move_to_NBV_sampling(
                            nbv_pose_dq, circ_dir)
                    motion_end_time = time.time()
                    time_for_robot_motion = motion_end_time - motion_start_time
                    # Wait for a second to ensure octomap is updated after reaching the NBV
                    time.sleep(2)
                    # After reaching the NBV, calculate the updated coverage
                    total_coverage, chamfer_distance = self.calculate_coverage(
                        total_coverage)
                    # Other results calculated from octomap (entropy, number of unknown, occupied, free voxels, covered volume)
                    octomap_results = self.calculate_entropy()

                    # Store the time taken for motion to NBV
                    result_dic_for_nbv["selected_nbv"] = nbv_np.tolist()
                    result_dic_for_nbv["total_coverage"] = total_coverage
                    result_dic_for_nbv["octomap_results"] = octomap_results
                    result_dic_for_nbv["time_for_nbv_calculation"] = time_for_nbv_calculation
                    result_dic_for_nbv["time_for_robot_motion"] = time_for_robot_motion
                    result_dic_for_nbv["max_ig"] = max_ig
                    # Add to the method result list
                    self.coverage_for_all_nbvs.append(total_coverage)
                    self.chamfer_distance_for_all_nbvs.append(chamfer_distance)
                    self.entropy_for_all_nbvs.append(
                        octomap_results["entropy"])
                    self.time_for_all_nbvs.append(time_for_nbv_calculation)
                    self.control_generation_time_for_all_nbvs.append(
                        result_dic_for_nbv["total_controller_time"])
                    self.robot_motion_time_for_all_nbvs.append(
                        time_for_robot_motion)
                    self.robot_path_length_for_all_nbvs.append(
                        result_dic_for_nbv["robot_path_length"])
                    self.max_ig_for_all_nbvs.append(max_ig)
                    self.termination_reason_for_all_nbvs.append(
                        result_dic_for_nbv["termination_reason"])
                    self.iteration_counter_for_all_nbvs.append(
                        result_dic_for_nbv["iteration_counter"])

                    if method == "FOCUS":
                        self.total_focus_time_for_all_nbvs.append(
                            np.array(result_dic_for_nbv["focus_evaluation_times"]).sum())
                        self.total_focus_calculations_for_all_nbvs.append(
                            result_dic_for_nbv["num_of_focus_calculations"])
                    if method == "SAMPLING":
                        self.total_view_evaluation_time_for_all_nbvs.append(
                            np.array(result_dic_for_nbv["view_evaluation_times"]).sum())
                        self.total_num_of_view_evaluations_for_all_nbvs.append(
                            result_dic_for_nbv["num_of_view_evaluations"])

                    self.data_manager.save_nbv_result(
                        object_index, (nbv_cntr+1), result_dic_for_nbv, method)
                    if self.params["NBV"]["savePcl"]:
                        self.data_manager.save_partial_model_data(
                            object_index, (nbv_cntr+1), self.current_partial_pcl, method)
                    if self.params["NBV"]["saveOctomap"]:
                        self.save_current_octomap(
                            object_index, method, (nbv_cntr+1))

                auc_final_for_method = calculate_area_under_curve(np.arange(len(self.coverage_for_all_nbvs)),
                                                                  self.coverage_for_all_nbvs,
                                                                  x_max=self.params["NBV"]["maxNBVCalls"]+1,
                                                                  y_max=1.0)
                summary_result_dic = {
                    "SUMMARY":
                    {
                        "auc_final_for_method": auc_final_for_method,
                        "chamfer_distance_final_for_method": self.chamfer_distance_for_all_nbvs[-1],
                        "coverage_final_for_method": self.coverage_for_all_nbvs[-1],
                        "entropy_final_for_method": self.entropy_for_all_nbvs[-1],
                        "total_motion_time_for_method": np.sum(np.array(self.robot_motion_time_for_all_nbvs)),
                        "total_control_time_for_method": np.sum(np.array(self.control_generation_time_for_all_nbvs)),
                        "total_nbv_calculation_time_for_method": np.sum(np.array(self.time_for_all_nbvs)),
                        "total_path_length_for_method": np.sum(np.array(self.robot_path_length_for_all_nbvs)),
                        "termination_reasons_for_all_nbvs": self.termination_reason_for_all_nbvs,
                        "iteration_counters_for_all_nbvs": int(np.sum(np.array(self.iteration_counter_for_all_nbvs)))
                    },
                    "EACH_NBV":
                    {
                        "coverage_for_all_nbvs": self.coverage_for_all_nbvs,
                        "chamfer_distance_for_all_nbvs": self.chamfer_distance_for_all_nbvs,
                        "entropy_for_all_nbvs": self.entropy_for_all_nbvs,
                        "time_for_all_nbvs": self.time_for_all_nbvs,
                        "control_generation_time_for_all_nbvs": self.control_generation_time_for_all_nbvs,
                        "robot_motion_time_for_all_nbvs": self.robot_motion_time_for_all_nbvs,
                        "robot_path_length_for_all_nbvs": self.robot_path_length_for_all_nbvs,
                        "max_ig_for_all_nbvs": self.max_ig_for_all_nbvs
                    }
                }
                if method == "FOCUS":
                    summary_result_dic["SUMMARY"]["total_focus_time_for_method"] = np.sum(np.array(
                        self.total_focus_time_for_all_nbvs))
                    summary_result_dic["EACH_NBV"]["total_focus_time_for_all_nbvs"] = self.total_focus_time_for_all_nbvs

                    summary_result_dic["SUMMARY"]["total_focus_calculations_for_method"] = int(np.sum(np.array(
                        self.total_focus_calculations_for_all_nbvs)))
                    summary_result_dic["EACH_NBV"]["total_focus_calculations_for_all_nbvs"] = self.total_focus_calculations_for_all_nbvs

                if method == "SAMPLING":
                    summary_result_dic["SUMMARY"]["total_view_evaluation_time_for_method"] = np.sum(np.array(
                        self.total_view_evaluation_time_for_all_nbvs))
                    summary_result_dic["EACH_NBV"]["total_view_evaluation_time_for_all_nbvs"] = self.total_view_evaluation_time_for_all_nbvs

                    summary_result_dic["SUMMARY"]["total_view_evaluations_for_method"] = int(np.sum(np.array(
                        self.total_num_of_view_evaluations_for_all_nbvs)))
                    summary_result_dic["EACH_NBV"]["total_view_evaluations_for_all_nbvs"] = self.total_num_of_view_evaluations_for_all_nbvs

                self.data_manager.save_nbv_result(
                    object_index, "summary", summary_result_dic, method)
                # Remove the previous object mesh from coppeliasim for next object
                self.object_mesh_handle.remove
                self.pr.step()
                self.roslaunch_parent.shutdown()
                # Wait for a second to roslaunch to shutdown properly
                time.sleep(1)

    def move_to_NBV_no_path(self, nbv_pose_dq, circ_dir=1):

        vis_target_position = np.array([0, 0, 0])
        stability_count = 0
        prev_task_err_norm = float('inf')
        task_err_norm = float('inf')

        iteration_counter = 0
        robot_path_list = []
        is_stability_error = False
        total_controller_time = 0.0
        while task_err_norm > self.params["ControllerSettings"]["errorTolerance"]:
            iteration_counter += 1
            robot_q = self.robot_model.get_q_from_sim()

            controller_start_time = time.time()
            self.robot_controller.constraint_switches["enableVisibilityConst"] = False
            u, _, _ = self.robot_controller.compute_one_step_u(nbv_pose_dq, robot_q, self.obs_pose_list_dq, self.obs_rad_list,
                                                               focus_point=vis_target_position, circulation_dir=circ_dir, method="NOPATH")
            controller_end_time = time.time()
            controller_duration = controller_end_time - controller_start_time
            total_controller_time += controller_duration
            # Convert base velocities to robot frame
            base_phi = robot_q[2]
            rot_mat = np.array([[math.cos(base_phi), math.sin(base_phi), 0],
                                [-math.sin(base_phi),
                                math.cos(base_phi), 0],
                                [0, 0, 1]])
            u = u[:8]
            base_vel_1 = rot_mat@u[:3]

            u[:3] = base_vel_1

            # Send robot velocities
            self.robot_model.send_velocities(u)
            self.pr.step()

            q_to_check = self.robot_model.get_q_from_sim()
            robot_path_list.append(q_to_check[:2])
            # Check if the robot did not make significant progress which help detect local minima
            transition_err_norm, direction_err_norm, task_err_norm = self.calculate_error_norms(
                robot_q, nbv_pose_dq)

            if self.params["ControllerSettings"]["includeStabilityCheck"]:
                task_err_norm_change = abs(prev_task_err_norm - task_err_norm)
                if task_err_norm_change < self.params["ControllerSettings"]["stabilityThreshold"]:
                    stability_count += 1
                else:
                    stability_count = 0

                if stability_count >= self.params["ControllerSettings"]["maxStabilityCount"]:
                    print(
                        "Controller stabilized but error norm is still high, stopping to prevent oscillations.")
                    is_stability_error = True
                    break

            prev_task_err_norm = task_err_norm
        self.robot_model.send_velocities(np.zeros(8))
        self.pr.step()
        termination_reason = ""
        if not is_stability_error:
            termination_reason = TerminationReason.SUCCESS
        else:
            termination_reason = TerminationReason.STABILITY_ERROR
        diffs = np.diff(np.array(robot_path_list), axis=0)
        path_length = np.linalg.norm(
            diffs, axis=1).sum() if diffs.size else 0.0
        result_dic = {"iteration_counter": int(iteration_counter),
                      "termination_reason": termination_reason,
                      "stability_count": stability_count,
                      "task_err_norm": task_err_norm,
                      "average_controller_time": total_controller_time / max(1, iteration_counter),
                      "total_controller_time": total_controller_time,
                      "robot_path_length": path_length}

        print(f"Result Dic: {result_dic}")
        return result_dic

    def move_to_NBV_focus_point(self, nbv_pose_dq, nbv_initial_q, circ_dir=1):
        """ Move the robot to the NBV using focus point based controller. 

        Args:
            nbv_pose_dq (DQ): The desired NBV pose as a dual quaternion.
            nbv_initial_q (np.ndarray): The initial joint configuration of the robot.
            circ_dir (int, optional): The circulation direction around obstacles. Defaults to 1."""

        prev_eef_position = vec3(translation(
            self.robot_model.Kinematics.fkm(nbv_initial_q)))
        vis_target_position = np.array([0, 0, 0])
        stability_count = 0
        prev_task_err_norm = float('inf')
        task_err_norm = float('inf')
        # Calculate focus point in the first iteration and then based on movement
        is_first_iteration = True
        iteration_counter = 0
        robot_path_list = []
        focus_time_list = []
        num_of_focus_calculations = 0

        is_stability_error = False
        total_controller_time = 0.0
        # Main control loop to reach the NBV
        while task_err_norm > self.params["ControllerSettings"]["errorTolerance"]:
            iteration_counter += 1
            robot_q = self.robot_model.get_q_from_sim()
            current_eef_pose_dq = self.robot_model.Kinematics.fkm(robot_q)
            current_eef_position = vec3(translation(current_eef_pose_dq))

            # keep track of the distance moved by the robot end-effector to determine if we should recalculate the focus point
            dist_eef_moved = np.linalg.norm(
                current_eef_position - prev_eef_position)

            # Calculate a new focus point if the robot has moved significantly or it is the first iteration
            if dist_eef_moved > self.params["NBV"]["focusUpdateDistance"] or is_first_iteration:
                prev_eef_position = current_eef_position
                # Request a new focus point from the ROS service
                dir_2_center = np.array([self.search_space_center[0]-current_eef_position[0],
                                        self.search_space_center[1]-current_eef_position[1], 0])
                orientation_toward_center = vec4(direction_to_orientation(
                    dir_2_center))
                focus_point_request_np = np.concatenate(
                    (current_eef_position, orientation_toward_center), axis=0)
                # request focus point from the focus point calculation service
                try:
                    focus_req = focus_point_srvRequest()
                    focus_req.pose = focus_point_request_np
                    focus_req.run_parallel = self.params["NBV"]["parallelComputationEnabled"]
                    response = self.focus_pnt_func(focus_req)
                    vis_target_position = np.array(response.focus_pnt)
                    focus_point_calculation_time = response.elapsed_time/1000.0  # Convert to seconds
                except Exception as e:
                    print(f"Error in focus point service call: {e}")
                    vis_target_position = np.array([0, 0, 0])

                # Store the time taken for focus point calculation
                focus_time_list.append(focus_point_calculation_time)
                num_of_focus_calculations += 1

                prev_eef_position = current_eef_position
                self.visual_manager.set_focus_point_position(
                    vis_target_position)

            is_first_iteration = False
            # Run the controller with visibility constraint enabled and focus point provided
            # Controller generates one step velocities for the robot to move toward the NBV while keeping the focus point in view
            controller_start_time = time.time()
            self.robot_controller.constraint_switches["enableVisibilityConst"] = True
            u, _, _ = self.robot_controller.compute_one_step_u(nbv_pose_dq, robot_q, self.obs_pose_list_dq, self.obs_rad_list,
                                                               focus_point=vis_target_position, circulation_dir=circ_dir, method="FOCUS")
            controller_end_time = time.time()
            # keep track of total controller time
            controller_duration = controller_end_time - controller_start_time
            total_controller_time += controller_duration

            # Generated base velocities (x_dot, y_dot, theta_dot) by the controller is w.r.t word frame,
            # we transform them to the robot base frame
            base_phi = robot_q[2]
            rot_mat = np.array([[math.cos(base_phi), math.sin(base_phi), 0],
                                [-math.sin(base_phi),
                                math.cos(base_phi), 0],
                                [0, 0, 1]])
            u = u[:8]
            base_vel_1 = rot_mat@u[:3]

            u[:3] = base_vel_1

            # Send robot velocities
            self.robot_model.send_velocities(u)
            self.pr.step()

            q_to_check = self.robot_model.get_q_from_sim()
            robot_path_list.append(q_to_check[:2])

            # Check if the robot did not make significant progress which help detect local minima
            transition_err_norm, direction_err_norm, task_err_norm = self.calculate_error_norms(
                robot_q, nbv_pose_dq)

            # If the error norm is not decreasing significantly over time, we assume the controller has stabilized before reaching the target
            if self.params["ControllerSettings"]["includeStabilityCheck"]:
                task_err_norm_change = abs(prev_task_err_norm - task_err_norm)
                if task_err_norm_change < self.params["ControllerSettings"]["stabilityThreshold"]:
                    stability_count += 1
                else:
                    stability_count = 0

                if stability_count >= self.params["ControllerSettings"]["maxStabilityCount"]:
                    print(
                        "Controller stabilized but error norm is still high, stopping to prevent oscillations.")
                    is_stability_error = True
                    break
            # keep track of previous error norm for checking stability
            prev_task_err_norm = task_err_norm
        # The robot has reached the NBV or terminated due to stability error
        # Stop the robot.
        self.robot_model.send_velocities(np.zeros(8))
        self.pr.step()
        termination_reason = ""
        if not is_stability_error:
            termination_reason = TerminationReason.SUCCESS
        else:
            termination_reason = TerminationReason.STABILITY_ERROR
        # calculate the robot path length
        diffs = np.diff(np.array(robot_path_list), axis=0)
        path_length = np.linalg.norm(
            diffs, axis=1).sum() if diffs.size else 0.0
        # Store the results in a dictionary
        result_dic = {"iteration_counter": int(iteration_counter),  # the number of iterations the controller ran
                      "termination_reason": termination_reason,  # the reason for termination
                      "stability_count": stability_count,
                      "focus_evaluation_times": focus_time_list,  # list of the e
                      "num_of_focus_calculations": num_of_focus_calculations,
                      "task_err_norm": task_err_norm,
                      "average_controller_time": total_controller_time / max(1, iteration_counter),
                      "total_controller_time": total_controller_time,
                      "robot_path_length": path_length}

        return result_dic

    def move_to_NBV_sampling(self, nbv_pose_dq, circ_dir=1, show_rrt_paths=True):

        # Create marker dictionary to visualize RRT* paths
        self.sampling_marker_dict = {"rrt_nodes": [],
                                     "rrt_global_path_nodes": [],
                                     "rrt_edges": [],
                                     "sample_views": []}

        # Keep trying to find a proper path until it finds one
        robot_q = self.robot_model.get_q_from_sim()
        current_eef_pose_dq = self.robot_model.Kinematics.fkm(robot_q)
        current_position = vec3(translation(current_eef_pose_dq))
        nbv_position_np = vec3(translation(nbv_pose_dq))

        is_plan_found = False
        rrt_fail_count = 0
        rrt_start_time = time.time()
        while not is_plan_found:

            ipp = IPP(start=[current_position[0], current_position[1], current_position[2]],
                      goal=[nbv_position_np[0], nbv_position_np[1], nbv_position_np[2]], max_iter=5000, expand_dis=0.5)

            is_plan_found, rrt_result_dic = ipp.get_ipp_result()
            if not is_plan_found:
                print("RRT Failed")
                rrt_fail_count += 1
            if rrt_fail_count >= 10:
                is_plan_found = False
                break
        rrt_end_time = time.time()
        time_for_rrt_planning = rrt_end_time - rrt_start_time
        if not is_plan_found:
            return False

        # Extract RRT edges efficiently
        rrt_edges = ipp.rrt_plan.extract_rrt_edges()
        self.sampling_marker_dict["rrt_nodes"] = rrt_result_dic["rrt_node_list"]
        self.sampling_marker_dict["rrt_global_path_nodes"] = rrt_result_dic["rrt_positions"]
        self.sampling_marker_dict["rrt_edges"] = rrt_edges

        # Rule out the path nodes that are too close to target and initial positions
        dist_to_start = np.linalg.norm(
            rrt_result_dic["rrt_positions"]-current_position, axis=1)
        dist_to_target = np.linalg.norm(
            rrt_result_dic["rrt_positions"]-nbv_position_np, axis=1)
        rrt_path_np = np.array(rrt_result_dic["rrt_positions"])[
            (dist_to_start > 1) & (dist_to_target > 1)]

        # Reverse the path node list to make it from start to finish
        rrt_path_np = np.array(rrt_path_np)[::-1]
        num_view_evaluations = 0

        eval_time_for_all_views = []
        controller_times_for_all_views = []
        max_ig_for_all_views = []
        termination_reasons_for_all_views = []
        err_norms_for_all_views = []
        robot_motion_time_for_all_views = []
        iteration_count_for_all_views = []
        stability_count_for_all_views = []
        path_length_for_all_views = []

        for node_idx in range(len(rrt_path_np)+1):
            if node_idx == len(rrt_path_np):
                # At the last node, go directly to the NBV
                sampling_view_pose_dq = nbv_pose_dq
                max_ig = 0.0
                time_for_view_evaluation = 0.0
            else:
                path_node = rrt_path_np[node_idx]
                # Sample 10 views around the node
                sphere_sample_list = ipp.sample_in_sphere(
                    path_node)

                self.sampling_marker_dict["sample_views"] = sphere_sample_list
                if show_rrt_paths:
                    self.publish_markers(self.sampling_marker_dict)

                # Make the views one vector for the ros server request
                view_list_srv = []
                for dq_pose in sphere_sample_list:
                    trans = vec3(translation(dq_pose))
                    orient = vec4(rotation(dq_pose))
                    view_list_srv.append(trans.tolist() + orient.tolist())

                # Evaluate the sampled views using ENT method
                max_ig, best_view_sampling, best_view_idx_sampling, time_for_view_evaluation = self.evaluate_candidate_views(
                    np.array(view_list_srv), ig_method="ENT", run_parallel=self.params["NBV"]["parallelComputationEnabled"])

                num_view_evaluations += 1
                # Move to the best view around the path node
                # Convert NBV to dq pose
                sampling_view_position = best_view_sampling[0]*i_ + \
                    best_view_sampling[1]*j_+best_view_sampling[2]*k_
                sampling_view_ori = normalize(best_view_sampling[3]+best_view_sampling[4]*i_ +
                                              best_view_sampling[5]*j_+best_view_sampling[6]*k_)
                sampling_view_pose_dq = (
                    sampling_view_ori+0.5*E_*sampling_view_position*sampling_view_ori)
                if show_rrt_paths:
                    self.visual_manager.set_sampling_best_view_frame_pose(
                        sampling_view_pose_dq)

            stability_count = 0
            prev_task_err_norm = float('inf')
            task_err_norm = float('inf')

            iteration_counter = 0
            robot_path_list = []
            is_stability_error = False
            total_controller_time = 0.0
            view_motion_start_time = time.time()
            self.robot_controller.constraint_switches["enableVisibilityConst"] = False
            while task_err_norm > self.params["ControllerSettings"]["errorTolerance"]:
                iteration_counter += 1
                robot_q = self.robot_model.get_q_from_sim()
                controller_start_time = time.time()
                u, _, _ = self.robot_controller.compute_one_step_u(sampling_view_pose_dq, robot_q, self.obs_pose_list_dq, self.obs_rad_list,
                                                                   focus_point=None, circulation_dir=circ_dir, method="SAMPLING")

                controller_end_time = time.time()
                controller_duration = controller_end_time - controller_start_time
                #  Accumulate total controller time
                total_controller_time += controller_duration
                # Convert base velocities to robot frame
                base_phi = robot_q[2]
                rot_mat = np.array([[math.cos(base_phi), math.sin(base_phi), 0],
                                    [-math.sin(base_phi),
                                    math.cos(base_phi), 0],
                                    [0, 0, 1]])
                u = u[:8]
                base_vel_1 = rot_mat@u[:3]

                u[:3] = base_vel_1

                # Send robot velocities
                self.robot_model.send_velocities(u)
                self.pr.step()
                q_to_check = self.robot_model.get_q_from_sim()
                robot_path_list.append(q_to_check[:2])
                transition_err_norm, direction_err_norm, task_err_norm = self.calculate_error_norms(
                    robot_q, sampling_view_pose_dq)

                if self.params["ControllerSettings"]["includeStabilityCheck"]:
                    task_err_norm_change = abs(
                        prev_task_err_norm - task_err_norm)
                    if task_err_norm_change < self.params["ControllerSettings"]["stabilityThreshold"]:
                        stability_count += 1
                    else:
                        stability_count = 0

                    if stability_count >= self.params["ControllerSettings"]["maxStabilityCount"]:
                        print(
                            "Controller stabilized but error norm is still high, stopping to prevent oscillations.")
                        is_stability_error = True
                        break

                prev_task_err_norm = task_err_norm
            view_motion_end_time = time.time()
            robot_motion_time_for_view = view_motion_end_time - view_motion_start_time

            self.robot_model.send_velocities(np.zeros(8))
            self.pr.step()
            termination_reason = ""
            if not is_stability_error:
                termination_reason = TerminationReason.SUCCESS
            else:
                termination_reason = TerminationReason.STABILITY_ERROR
            # Store data for each view motion until reaching the NBV
            eval_time_for_all_views.append(time_for_view_evaluation)
            max_ig_for_all_views.append(max_ig)
            robot_motion_time_for_all_views.append(robot_motion_time_for_view)
            controller_times_for_all_views.append(total_controller_time)
            termination_reasons_for_all_views.append(termination_reason)
            err_norms_for_all_views.append(task_err_norm)
            iteration_count_for_all_views.append(iteration_counter)
            stability_count_for_all_views.append(stability_count)

            diffs = np.diff(np.array(robot_path_list), axis=0)
            path_length = np.linalg.norm(
                diffs, axis=1).sum() if diffs.size else 0.0
            path_length_for_all_views.append(path_length)

        result_dic = {"iteration_counter": int(np.array(iteration_count_for_all_views).sum()),
                      "termination_reason": termination_reasons_for_all_views[-1],
                      "stability_count": stability_count_for_all_views[-1],
                      "view_evaluation_times": eval_time_for_all_views,
                      "num_of_view_evaluations": num_view_evaluations,
                      "task_err_norm": err_norms_for_all_views[-1],
                      "average_controller_time": np.array(controller_times_for_all_views).sum() / max(1, np.array(iteration_count_for_all_views).sum()),
                      "total_controller_time": np.array(controller_times_for_all_views).sum(),
                      "robot_path_length": np.array(path_length_for_all_views).sum(),
                      "rrt_planning_time": time_for_rrt_planning,


                      "details_for_views": {
                          "max_ig_list": max_ig_for_all_views,
                          "view_motion_time_list": robot_motion_time_for_all_views,
                          "termination_reasons": termination_reasons_for_all_views,
                          "err_norms": err_norms_for_all_views,
                          "iteration_counts": iteration_count_for_all_views,
                          "stability_counts": stability_count_for_all_views,
                          "path_lengths": path_length_for_all_views,
                          "total_controller_times": controller_times_for_all_views,
                          "average_controller_times": [total_time/max(1, iter) for total_time, iter in zip(controller_times_for_all_views, iteration_count_for_all_views)]
        }}

        return result_dic

    def calculate_error_norms(self, robot_q, target_pose_dq):
        """ Get the current error norm between the robot end effector and the target pose """

        # Check if the robot did not make significant progress which help detect local minima
        updated_pose_dq = self.robot_model.Kinematics.fkm(
            robot_q)
        # Translation error between the robot end effector and the target pose
        tra_err = vec3(translation(updated_pose_dq)) - \
            vec3(translation(target_pose_dq))
        eef_line = vec3(get_direction(updated_pose_dq, k_))
        target_line = vec3(get_direction(target_pose_dq, k_))
        direction_err = eef_line - target_line
        task_err = np.vstack([direction_err[:, np.newaxis],
                              tra_err[:, np.newaxis]])
        transition_err_norm = np.linalg.norm(tra_err)
        direction_err_norm = np.linalg.norm(direction_err)
        task_err_norm = np.linalg.norm(task_err)
        return transition_err_norm, direction_err_norm, task_err_norm

    def evaluate_candidate_views(self, candidate_views=None, ig_method="RSV", run_parallel=True):
        """ Calculate the next best view (NBV) based on information gain (IG) evaluation and remove it from the remaining views list """
        try:

            nbv_req = candidate_views.flatten()
            request = view_evaluate_srvRequest()
            request.view_list = nbv_req
            request.ig_method = ig_method  # "RSV" or "ENT"
            request.run_parallel = run_parallel
            response = self.view_ig_func(request)
            view_ig_result = np.array(response.view_igs)
            views_eval_time = response.elapsed_time/1000.0  # Convert to seconds

            max_ig = np.max(view_ig_result)
            best_view_idx = np.argmax(view_ig_result)
            best_view = candidate_views[best_view_idx]

        except Exception as e:
            print(f"Error in calculate_next_best_view: {e}")
            best_view = None
            best_view_idx = -1
            max_ig = -1
        return max_ig, best_view, best_view_idx, views_eval_time

    def calculate_coverage(self, total_coverage=0.0):
        """ Get the current surface coverage of the object based on the generated point cloud from octomap """
        # Calculate surface coverage
        ros_point_cloud = rospy.wait_for_message(
            "/octomap_server_fine/octomap_point_cloud_centers", PointCloud2)
        gen = pc2.read_points(ros_point_cloud, skip_nans=True)
        int_data = list(gen)
        xyz = []
        for x in int_data:
            xyz.append([*x])
        xyz_np = np.array(xyz)
        # Calculate the number of points covered in the remaining ground truth point
        # cloud with covered point indexes
        covered_idxes, num_of_covered = pcl_coverage(
            self.gt_pcl_remaining, xyz_np)
        # Add to total coverage
        current_coverage = num_of_covered / self.gt_pcl.shape[0]
        total_coverage += current_coverage
        # Update the current partial point cloud with newly covered points
        self.current_partial_pcl = np.concatenate(
            (self.current_partial_pcl, self.gt_pcl_remaining[covered_idxes]), axis=0)
        _, _, chamfer_distance = pcl_chamfer_distance(
            self.current_partial_pcl, self.gt_pcl)
        # Remove covered points from remaining ground truth point cloud
        self.gt_pcl_remaining = self.gt_pcl_remaining[~covered_idxes]
        return total_coverage, chamfer_distance

    def calculate_entropy(self):
        response = self.coverage_func()
        result_dic = {"entropy": response.ent,
                      "unknown": response.unknown,
                      "occupied": response.occupied,
                      "free": response.free,
                      "cov_volume": response.cvr
                      }
        return result_dic

    def launch_simulation(self):
        """ Launch the CoppeliaSim simulation with the specified scene file and time step """

        SCENE_FILE = join(dirname(abspath(__file__)),
                          self.params["SimulatorSettings"]["sceneNameVelocity"])

        self.pr.launch(SCENE_FILE, headless=False)
        self.simulation_time_step = self.params["SimulatorSettings"]["simTimeStepVelocity"]

        self.pr.set_simulation_timestep(self.simulation_time_step)
        self.pr.start()
        print("Simulation launched with scene:", SCENE_FILE)

    def init_ros_services(self):
        """ Initialize the ROS services used for obtaining coverage, focus points,
            and view information gains """

        # Rospy services to get coverage, focus point, and view IGs
        # Returns information related to the current partial model of the object
        self.coverage_func = rospy.ServiceProxy("get_coverage", coverage_srv)
        # Return a focus point that helps improve the model reconstruction
        self.focus_pnt_func = rospy.ServiceProxy(
            "get_focus_point", focus_point_srv)
        # Evaluate the information gain of a given viewpoints which is used for NBV selection
        self.view_ig_func = rospy.ServiceProxy(
            "get_view_igs", view_evaluate_srv)

        self.octomap_save_func = rospy.ServiceProxy(
            "save_octomap", save_octomap_srv)

        self.sampling_path_pub = rospy.Publisher(
            "/sampling_path", MarkerArray, queue_size=1)

        # Get the launch file path to launch a new session from this script
        self.launch_file_path = self.params["NBV"]["launchFilePath"]

    def launch_ros_session(self):
        """ Launch a new ROS session with octomap server and point cloud generation """
        # Run the roslaunch file which starts octomap server and point cloud generation
        self.run_id = rospy.get_param('/run_id')
        self.roslaunch_parent = roslaunch.parent.ROSLaunchParent(
            self.run_id, [self.launch_file_path])
        self.roslaunch_parent.start()

    def init_random_env_generator(self):
        """ Initialize the parameters for random obstacle
            generation in the workspace """

        # Get the workspace plane poses from CoppeliaSim
        self.plane_left_pose, self.plane_right_pose, self.plane_top_pose, self.plane_bottom_pose = self.visual_manager.get_workspace_plane_poses()

        # Get the plane positions from their poses as numpy arrays
        self.plane_left_psn = vec3(translation(self.plane_left_pose))
        self.plane_right_psn = vec3(translation(self.plane_right_pose))
        self.plane_top_psn = vec3(translation(
            self.plane_top_pose))  # Max x (Plane 2)
        self.plane_bottom_psn = vec3(translation(
            self.plane_bottom_pose))  # Min x (Plane 3)

        self.limits_obs_region = {"x_min": self.plane_bottom_psn[0]+self.offset_workspace_planes,
                                  "x_max": self.plane_top_psn[0]-self.offset_workspace_planes,
                                  "y_min": self.plane_left_psn[1]-self.offset_workspace_planes,
                                  "y_max": self.plane_right_psn[1]+self.offset_workspace_planes
                                  }

        self.random_env_params = {
            "obs_x_lims": [self.limits_obs_region["x_min"], self.limits_obs_region["x_max"]],
            "obs_y_lims": [self.limits_obs_region["y_min"], self.limits_obs_region["y_max"]],
            "robot_rad": self.robot_dim/2,
            "min_dist_between_obs": self.robot_dim+0.02,
            "num_obstacle": self.params["RandomEnvironment"]["numObstacles"],
            "obstacle_rads_range": self.params["RandomEnvironment"]["obstacleRads"],
            "search_space_center": self.search_space_center,
            "search_space_radius": self.search_space_radius,
            "robot_safe_rad": self.params["RandomEnvironment"]["robotSafeRadius"]
        }
        self.rand_env_gen = RandomEnv(self.random_env_params)

    def publish_markers(self, markers_dict):
        marker_array = MarkerArray()
        # Include nodes as spheres
        for i, node in enumerate(markers_dict["rrt_nodes"]):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "rrt_nodes"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = node.x
            marker.pose.position.y = node.y
            marker.pose.position.z = node.z
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1  # Sphere diameter
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)

        # Include global path nodes as green spheres
        for i, node in enumerate(markers_dict["rrt_global_path_nodes"]):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "rrt_global_path"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = node[0]
            marker.pose.position.y = node[1]
            marker.pose.position.z = node[2]
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.15  # Sphere diameter
            marker.scale.y = 0.15
            marker.scale.z = 0.15
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)

        # Include edges as line strips
        for i, edge in enumerate(markers_dict["rrt_edges"]):
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "rrt_edges"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.01  # Line width
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0

            start_point = Point()
            start_point.x, start_point.y, start_point.z = edge[0]
            end_point = Point()
            end_point.x, end_point.y, end_point.z = edge[1]

            marker.points.append(start_point)
            marker.points.append(end_point)

            marker_array.markers.append(marker)

        # Include sampling best views as yellow spheres
        for i, node in enumerate(markers_dict["sample_views"]):
            node = vec3(translation(node))
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "sample_views"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = node[0]
            marker.pose.position.y = node[1]
            marker.pose.position.z = node[2]
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.12  # Sphere diameter
            marker.scale.y = 0.12
            marker.scale.z = 0.12
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)

        self.sampling_path_pub.publish(marker_array)


if __name__ == "__main__":

    rospy.init_node("object_reconstruction_simulation_node", anonymous=True)
    with open("Config/config.yaml", 'r') as configFile:
        configs = yaml.safe_load(configFile)

    nbv_obj = None

    def cleanup():
        print("Shutting down...")
        nbv_obj.pr.stop()
        nbv_obj.pr.shutdown()
        nbv_obj.roslaunch_parent.shutdown()
        sys.exit(0)

    # Register shutdown handlers
    rospy.on_shutdown(cleanup)
    atexit.register(cleanup)

    try:
        nbv_obj = MainClass(configs)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received, shutting down...")
        cleanup()
