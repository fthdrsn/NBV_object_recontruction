
import math
from pyrep import PyRep
from Robots.utils import *
from dqrobotics.utils import DQ_Geometry
import numpy as np
from QP import DQ_QuadprogSolver_Custom
from pyrep.objects.shape import Shape
from pyrep.const import PrimitiveShape
from os.path import dirname, join, abspath
from dqrobotics import *
from focus_point_calculator.srv import focus_point_srv, coverage_srv, view_evaluate_srv, view_evaluate_srvRequest
import json
import rospy
import yaml
import time
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import os
import roslaunch
from plan_inspection_path import IPP
from random_env_generator import RandomEnv
from sklearn.cluster import DBSCAN
from scipy.spatial import ConvexHull
import open3d as o3d


class NBV:

    def __init__(self, params) -> None:
        self.object_number = 10
        self.max_object_count = 114
        self.ig_method_number = 2
        self.pr = PyRep()
        self.params = params
        self.start_focus_now = True

        self.root_path = self.params["NBV"]["rootPath"]

        SCENE_FILE = join(dirname(abspath(__file__)),
                          self.root_path+self.params["SimulatorSettings"]["sceneNameVelocity"])

        self.pr.launch(SCENE_FILE, headless=False)
        self.simulation_time_step = self.params["SimulatorSettings"]["simTimeStepVelocity"]

        self.pr.set_simulation_timestep(self.simulation_time_step)
        self.pr.start()

        # Create robots` models
        self.robot_model = YouBotModel(
            is_velocity_control=True)
        self.comm_agent = BaseCommunication()

        #### Controllers#####
        self.qp_solver = DQ_QuadprogSolver_Custom()
        self.controller_gain = self.params["VelocityControllerSettings"]["controllerGain"]
        self.controller_damping = self.params["VelocityControllerSettings"]["controllerDamping"]
        self.controller_err_tolerance = self.params["VelocityControllerSettings"]["errorTolerance"]
        self.controller_stab_threshold = self.params["VelocityControllerSettings"]["stabilityThreshold"]
        self.controller_stab_max = self.params["VelocityControllerSettings"]["maxStabilityCount"]
        self.max_time_limit = self.params["VelocityControllerSettings"]["maxTimeLimit"]

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
        # self.base_obstacle_poses = self.get_base_obstacles()
        self.enable_collision_constraints = self.params["ProcessVariables"]["enableCollisionConst"]
        self.enable_circulation_constraints = True
        self.robot_to_far = False
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

        # self.shape_net_path = self.root_path+self.params["NBV"]["objectsPath"]
        # self.gt_path = self.root_path+self.params["NBV"]["groundTruthPath"]
        self.shape_net_path = "/media/fth/T9/ShapeNetCore_v1"
        self.gt_path = "/home/fth/focus_ws/src/nbv_coppelia/DatasetGT"
        self.object_path = self.gt_path+"/used_objects.txt"
        self.nbv_result_path = self.root_path+self.params["NBV"]["resultSavePath"] + \
            "/Circulation_3m"
        self.search_space_path = self.root_path + \
            self.params["NBV"]["searchSpacePath"]
        self.launch_file_path = self.root_path + \
            self.params["NBV"]["launchFilePath"]
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
        self.search_space_radius = 3
        # Parameters for the forward-check

        self.robot_safe_rad = 0.8  # Do not generate obstacles too close to the robot
        self.target_safe_rad = 0.8  # Do not generate obstacles too close to the target
        self.robot_diam = 0.72
        # Minimum distance between obstacles
        self.min_dist_between_obs = self.robot_diam
        self.num_obstacle = 10
        self.obstacle_rads = [0.1, 0.3]

        # Get obstacle planes
        self.plane_pose_list = []
        self.plane_0_pose = self.comm_agent.get_object_pose(
            PyRepObj.Object.get_object("plane_0"))
        self.plane_pose_list.append(self.plane_0_pose)  # Min y (Plane 0)
        self.plane_0_psn = vec3(translation(self.plane_0_pose))
        self.plane_1_pose = self.comm_agent.get_object_pose(
            PyRepObj.Object.get_object("plane_1"))
        self.plane_pose_list.append(self.plane_1_pose)  # Max y (Plane 1)
        self.plane_1_psn = vec3(translation(self.plane_1_pose))
        self.plane_2_pose = self.comm_agent.get_object_pose(
            PyRepObj.Object.get_object("plane_2"))
        self.plane_pose_list.append(self.plane_2_pose)
        self.plane_2_psn = vec3(translation(
            self.plane_2_pose))  # Max x (Plane 2)
        self.plane_3_pose = self.comm_agent.get_object_pose(
            PyRepObj.Object.get_object("plane_3"))
        self.plane_pose_list.append(self.plane_3_pose)
        self.plane_3_psn = vec3(translation(
            self.plane_3_pose))  # Min x (Plane 3)

        self.offset_workspace_planes = 1
        self.limits_obs_region = [
            self.plane_3_psn[0]+self.offset_workspace_planes,  self.plane_2_psn[0]-self.offset_workspace_planes, self.plane_0_psn[1]+self.offset_workspace_planes, self.plane_1_psn[1]-self.offset_workspace_planes]

        self.robot_initial_q = self.robot_model.get_q_from_sim()
        self.robot_initial_pose = self.robot_model.BaseKinematics.raw_fkm(
            np.array(self.robot_initial_q[:3]))

        self.random_env_params = {
            "obs_x_lims": self.limits_obs_region[:2],
            "obs_y_lims": self.limits_obs_region[2:],
            "min_dist_between_obs": self.min_dist_between_obs,
            "num_obstacle": self.num_obstacle,
            "obstacle_rads_range": self.obstacle_rads,
            "robot_position": self.robot_initial_q[:2],
            "cly_position": np.array([0, 0]),
            "cly_rad": self.search_space_radius,
            "robot_safe_rad": self.robot_safe_rad
        }

        self.is_pose_control = False
        self.b = 0.044  # 0.044
        self.D_0 = 0.20

        self.termination_time_th = 5
        self.rand_env_gen = RandomEnv(self.random_env_params)

        self.im_width_m = abs(self.limits_obs_region[0]) + \
            abs(self.limits_obs_region[1])
        self.im_height_m = abs(self.limits_obs_region[2]) + \
            abs(self.limits_obs_region[3])
        self.map_resolution = 0.01  # Each voxel correspongs map_resolution value in the map
        self.image_height = math.floor(self.im_height_m/self.map_resolution)
        self.image_width = math.floor(self.im_width_m/self.map_resolution)
        rot_ = cos(-pi2/2)+k_*sin(-pi2/2)
        tra_ = -self.limits_obs_region[1] * \
            i_+self.limits_obs_region[-1]*j_
        self.upper_corner_pose = (rot_+0.5*E_*tra_*rot_)
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
        self.gt_pcl_o3d = self.gt_pcl.copy()
        self.total_point_o3d = self.gt_pcl_o3d.shape[0]
        self.total_point = self.gt_pcl.shape[0]
        self.current_point = 0
        self.current_point_torch = 0
        self.total_cov_o3d = 0
        self.previous_position_focus = None
        self.coverage_list_o3d = []
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
        self.base_path_len_list = []
        self.eef_path_len_list = []

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

        for _ in range(40):
            self.pr.step()
        time.sleep(5)

        if self.enable_visibility_constraint:
            if self.enable_focus_point:
                self.used_strategy = "adaptive_vis"
            else:
                self.used_strategy = "fixed_vis"
        elif self.enable_rrt_based_ipp:
            self.used_strategy = "sampling_based"
        else:
            self.used_strategy = "no_path"

        # For termintation criteria, we terminate if the ig values do not change more than %5 of the first maximum ig
        self.previous_ig_result = np.zeros(self.view_space.shape[0])
        self.first_ig_val = 0
        self.ig_threshold = 0
        self.remaining_views = self.view_space
        self.nbv_start_time = time.time()

        # Random Environment
        if self.generate_random_env:
            # self.rand_env_gen.num_obstacle = random.randint(10, 25)
            self.rand_env_gen.num_obstacle = 10
            [self.obs_pose_list_dq,
                self.obs_rad_list] = self.rand_env_gen.generate_random_env()

        idx_ = 0
        obs_xyrad = []
        self.obs_handle_list = []
        for obs_pose_, obs_rad_ in zip(self.obs_pose_list_dq, self.obs_rad_list):
            self.obs_visual = Shape.create(type=PrimitiveShape.CYLINDER,
                                           size=[2*obs_rad_,
                                                 2*obs_rad_, 0.22],
                                           color=[0.0, 0.0, 1.0],
                                           static=True, respondable=False, renderable=False)
            self.obs_visual.set_name(f"base_obs_{idx_}")
            self.obs_visual.set_color((0, 0, 1))

            self.comm_agent.set_object_pose(self.obs_visual, obs_pose_)
            self.obs_handle_list.append(self.obs_visual)
            idx_ += 1
            obs_position = vec3(translation(obs_pose_))
            obs_xyrad.append([obs_position[0], obs_position[1], obs_rad_])
            self.pr.step()

        map_obs_point_list = []
        map_obs_rad_list = []

        for obs_, rad_ in zip(self.obs_pose_list_dq+[DQ([1])], self.obs_rad_list+[2.85]):
            # Convert obstacle pose w.r.t upper left corner
            local_pose = conj(self.upper_corner_pose)*obs_
            tr = vec3(translation(local_pose))[:2]  # Get x and y
            x_local = tr[0]  # Row
            y_local = tr[1]  # Col
            row_map_idx = min(math.floor(
                x_local/self.map_resolution), self.image_height-1)

            col_map_idx = min(math.floor(
                y_local/self.map_resolution), self.image_width-1)
            map_obs_point_list.append([row_map_idx, col_map_idx])
            map_obs_rad = math.floor(rad_/self.map_resolution)
            map_obs_rad_list.append(map_obs_rad)

        start_time = time.time()
        # Define the distance threshold for clustering
        distance_threshold = self.robot_diam
        # Compute the distance matrix
        distance_matrix = compute_distance_matrix(obs_xyrad)
        # Apply DBSCAN with precomputed distance matrix
        dbscan = DBSCAN(eps=distance_threshold,
                        min_samples=1, metric='precomputed')
        labels = dbscan.fit_predict(distance_matrix)
        # print("Labels", labels)
        # Creata a dictionary to store the obstacle indexes and positions for the groups
        num_group = max(labels)+1
        obs_groups = {}
        for i in range(num_group):
            obs_groups[f"Group{i}"] = {"idx": np.where(labels == i),
                                       "psn_workspace": np.array(obs_xyrad)[np.where(labels == i)],
                                       "psn_workspace_dq": np.array(self.obs_pose_list_dq)[np.where(labels == i)],
                                       "psn_map": np.fliplr(np.array(map_obs_point_list)[np.where(labels == i)])
                                       }
        obstacle_count = len(self.obs_pose_list_dq)
        # To start indexing virtual obstacles from the number of the original obstacles
        obs_idx_visual = obstacle_count
        virtual_diam = 0.3
        virtual_off = 0.15
        for key, value in obs_groups.items():
            virtual_obs_list = []
            # Check if there is more than one obstacle in the group
            if value["idx"][0].shape[0] > 1:
                # Check if there are more than 2 obstacles
                if value["idx"][0].shape[0] > 2:
                    # Use convex hull if there is more than two obstacles to generate virtual obstacles
                    hull = ConvexHull(value["psn_map"])
                    verts = hull.vertices
                else:
                    # Generate virtual obstacles between two obstacle in the group
                    verts = np.array([0, 1])

                # Fill the gap between vertices
                pnts_workspace = value["psn_workspace"]
                for i in range(verts.shape[0]):
                    pnt = pnts_workspace[verts[i]
                                         ][:2]
                    dist_2_centre = np.linalg.norm(
                        pnt)-self.search_space_radius-pnts_workspace[verts[i]
                                                                     ][-1]
                    # If the robot cannot pass the gap between obstacle and workspace
                    if dist_2_centre < self.robot_diam:
                        pnt_1 = pnts_workspace[verts[i]
                                               ][:2]
                        pnt_2 = np.array([0, 0])
                        rad_1 = pnts_workspace[verts[i]
                                               ][-1]
                        rad_2 = self.search_space_radius-0.1

                        dist_vec = pnt_2-pnt_1
                        # Direction vector between two vertices
                        dist_vec_norm = dist_vec/np.linalg.norm(dist_vec)
                        start_position = pnt_1+rad_1*dist_vec_norm  # Start position on the vector
                        end_position = pnt_2-rad_2*dist_vec_norm  # End posiiton on the vector
                        # Find the distance (gap) between obstacle excluding their radiuses
                        dist_norm = np.linalg.norm(dist_vec)-(rad_1+rad_2)
                        obs_diam_ = rad_1*2  # Diameter of the virtual obtacles
                        off_set = virtual_off  # Ofset between obstacles
                        # The number of obstalces which can fit the gap
                        num_obs = math.floor(dist_norm/off_set)+1
                        # remain_ = dist_norm-num_obs*obs_diam_
                        for r in range(num_obs+1):
                            if r == num_obs:
                                # Ensure that the final obstacle is positioned at the end position
                                obs_position = end_position
                            else:
                                # Move on the directon line to place the new obstacle
                                obs_position = start_position + \
                                    r*off_set*dist_vec_norm

                            virtual_obs_list.append(
                                [obs_position[0], obs_position[1], obs_diam_/2])

                            obs_visual = Shape.create(type=PrimitiveShape.CYLINDER,
                                                      size=[obs_diam_,
                                                            obs_diam_, 0.2],
                                                      color=[1.0, 0.0, 0.0],
                                                      static=True, respondable=False, renderable=False)
                            obs_visual.set_name(f"base_obs_{obs_idx_visual}")
                            obs_idx_visual += 1
                            self.obs_handle_list.append(obs_visual)
                            self.comm_agent.set_object_pose(
                                obs_visual, 1+E_*0.5*(obs_position[0]*i_+obs_position[1]*j_+0.1*k_))
                            self.pr.step()

                    if i == (verts.shape[0]-1):
                        # If the final vertices is reached, create virtual obstacles between starting and final verctices
                        pnt_1 = pnts_workspace[verts[0]
                                               ][:2]
                        pnt_2 = pnts_workspace[verts[-1]
                                               ][:2]
                        rad_1 = pnts_workspace[verts[0]
                                               ][-1]
                        rad_2 = pnts_workspace[verts[-1]
                                               ][-1]
                    else:
                        # Generate virtual obstacles between succesive vertices
                        pnt_1 = pnts_workspace[verts[i]
                                               ][:2]
                        pnt_2 = pnts_workspace[verts[i+1]
                                               ][:2]
                        rad_1 = pnts_workspace[verts[i]
                                               ][-1]
                        rad_2 = pnts_workspace[verts[i+1]
                                               ][-1]

                    dist_vec = pnt_2-pnt_1
                    # Direction vector between two vertices
                    dist_vec_norm = dist_vec/np.linalg.norm(dist_vec)
                    start_position = pnt_1+rad_1*dist_vec_norm  # Start position on the vector
                    end_position = pnt_2-rad_2*dist_vec_norm  # End posiiton on the vector
                    # Find the distance (gap) between obstacle excluding their radiuses
                    dist_norm = np.linalg.norm(dist_vec)-(rad_1+rad_2)
                    obs_diam_ = virtual_diam  # Diameter of the virtual obtacles
                    off_set = virtual_off  # Ofset between obstacles
                    # The number of obstalces which can fit the gap
                    num_obs = math.floor(dist_norm/off_set)+1
                    # remain_ = dist_norm-num_obs*obs_diam_
                    for r in range(num_obs+1):
                        if r == num_obs:
                            # Ensure that the final obstacle is positioned at the end position
                            obs_position = end_position
                        else:
                            # Move on the directon line to place the new obstacle
                            obs_position = start_position + \
                                r*off_set*dist_vec_norm

                        virtual_obs_list.append(
                            [obs_position[0], obs_position[1], obs_diam_/2])

                        obs_visual = Shape.create(type=PrimitiveShape.CYLINDER,
                                                  size=[obs_diam_,
                                                        obs_diam_, 0.4],
                                                  color=[
                                                      1.0, 0.0, 1.0],
                                                  static=True, respondable=False, renderable=False)
                        obs_visual.set_name(f"base_obs_{obs_idx_visual}")
                        obs_idx_visual += 1
                        self.obs_handle_list.append(obs_visual)
                        self.comm_agent.set_object_pose(
                            obs_visual, 1+E_*0.5*(obs_position[0]*i_+obs_position[1]*j_))
                        self.pr.step()

                obs_groups[key]["virtual_obstacles"] = np.array(
                    virtual_obs_list)
                # Assign new indexes to the included constraints starting from the number of obstacles from the initial obs list
                obs_groups[key]["idx"] = np.concatenate([obs_groups[key]["idx"][0],
                                                         np.array([x for x in range(
                                                             obstacle_count, obstacle_count+len(virtual_obs_list))])])
                obstacle_count += len(virtual_obs_list)

            else:
                # obs_groups[key]["virtual_obstacles"] = None
                pnts_workspace = value["psn_workspace"]
                pnt = pnts_workspace[0][:2]
                a = pnts_workspace[0][-1]
                dist_2_centre = np.linalg.norm(
                    pnt-np.array([0, 0]))-self.search_space_radius-pnts_workspace[0][-1]

                if dist_2_centre < self.robot_diam:
                    pnt_1 = pnts_workspace[0
                                           ][:2]
                    pnt_2 = np.array([0, 0])
                    rad_1 = pnts_workspace[0
                                           ][-1]
                    rad_2 = self.search_space_radius-0.1

                    dist_vec = pnt_2-pnt_1
                    # Direction vector between two vertices
                    dist_vec_norm = dist_vec/np.linalg.norm(dist_vec)
                    start_position = pnt_1+rad_1*dist_vec_norm  # Start position on the vector
                    end_position = pnt_2-rad_2*dist_vec_norm  # End posiiton on the vector
                    # Find the distance (gap) between obstacle excluding their radiuses
                    dist_norm = np.linalg.norm(dist_vec)-(rad_1+rad_2)
                    obs_diam_ = rad_1*2  # Diameter of the virtual obtacles
                    off_set = virtual_off  # Ofset between obstacles
                    # The number of obstalces which can fit the gap
                    num_obs = math.floor(dist_norm/off_set)+1
                    # remain_ = dist_norm-num_obs*obs_diam_
                    for r in range(num_obs+1):
                        if r == num_obs:
                            # Ensure that the final obstacle is positioned at the end position
                            obs_position = end_position
                        else:
                            # Move on the directon line to place the new obstacle
                            obs_position = start_position + \
                                r*off_set*dist_vec_norm

                        virtual_obs_list.append(
                            [obs_position[0], obs_position[1], obs_diam_/2])

                        obs_visual = Shape.create(type=PrimitiveShape.CYLINDER,
                                                  size=[obs_diam_,
                                                        obs_diam_, 0.4],
                                                  color=[
                                                      0.0, 0.0, 1.0],
                                                  static=True, respondable=False, renderable=False)
                        obs_visual.set_name(f"base_obs_{obs_idx_visual}")
                        obs_idx_visual += 1
                        self.obs_handle_list.append(obs_visual)
                        self.comm_agent.set_object_pose(
                            obs_visual, 1+E_*0.5*(obs_position[0]*i_+obs_position[1]*j_))
                        self.pr.step()
                    obs_groups[key]["virtual_obstacles"] = np.array(
                        virtual_obs_list)
                    # Assign new indexes to the included constraints starting from the number of obstacles from the initial obs list
                    obs_groups[key]["idx"] = np.concatenate([obs_groups[key]["idx"][0],
                                                            np.array([x for x in range(
                                                                obstacle_count, obstacle_count+len(virtual_obs_list))])])
                    obstacle_count += len(virtual_obs_list)
                else:
                    obs_groups[key]["virtual_obstacles"] = None
        # Include search space to obstacles

        self.obs_groups = obs_groups
        self.num_group = num_group

    def run_planner(self):
        method_counter = 0  # To switch between methods automatically
        self.generate_random_env = True
        while True:
            self.change_method(method_counter)
            self.make_ready_for_new_iteration()
            print(f"Object Number:{self.object_number} Method{method_counter}")
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
                self.nbv_time_list.append(self.nbv_time)
                # print("Elapsed Time for NBV:", time.time()-prev_time)
                view_ig_result = np.array(response.view_igs)

                # Find the best view
                max_ig = np.max(view_ig_result)
                self.ig_max_list.append(max_ig)
                best_view_idx = np.argmax(view_ig_result)
                best_view = self.remaining_views[best_view_idx]
                # Delete the best view for the next iteration
                self.remaining_views = np.delete(
                    self.remaining_views, (best_view_idx), axis=0)
                # If information gain does not change, move to the next object
                ig_diff = np.linalg.norm(
                    max_ig-self.previous_ig_result)
                # print("IG Difference", ig_diff)
                self.previous_ig_result = max_ig
                if ig_diff == -10000:
                    data = {
                        "coverage_list_o3d": self.coverage_list_o3d,
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
                        "cov_volume_list": self.cov_volume_list,
                        "best_ig_val_list": self.ig_max_list
                    }

                    file = self.result_save_path + f"/{self.used_strategy}"
                    with open(file, 'w') as f:
                        json.dump(data, f)
                    self.roslaunch_parent.shutdown()
                    self.rec_obj_ref.remove()
                    # self.object_number += 1

                    # self.make_ready_for_new_iteration()
                    break

                tan_dirs_init = self.num_group*[0]
                succ = self.move_to_nbv(
                    best_view, obstacles_dic=self.obs_groups, group_dir=tan_dirs_init)
                if not succ:
                    self.roslaunch_parent.shutdown()
                    self.rec_obj_ref.remove()
                    time.sleep(1)
                    break

            method_counter += 1
            # Use the next object after finishing all three methods
            self.generate_random_env = False
            if method_counter >= 1 and method_counter % 3 == 0:
                self.generate_random_env = True
                self.object_number += 1
                method_counter = 0

    def change_method(self, idx):
        if idx == 0:  # Activate focus point method
            self.enable_visibility_constraint = True
            self.enable_focus_point = True
            self.enable_rrt_based_ipp = False
            self.max_time_limit = 60
        if idx == 1:  # Activate IPP method
            self.enable_visibility_constraint = False
            self.enable_focus_point = False
            self.enable_rrt_based_ipp = True
            self.max_time_limit = 15
        if idx == 2:  # No Path Method
            self.enable_visibility_constraint = False
            self.enable_focus_point = False
            self.enable_rrt_based_ipp = False
            self.max_time_limit = 60
        if idx == 3:  # Focus the centre
            self.enable_visibility_constraint = True
            self.enable_focus_point = False
            self.enable_rrt_based_ipp = False
            self.max_time_limit = 60

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

    def move_to_nbv(self, nbv_pose, obstacles_dic=None, group_dir=None):

        nbv_start_time = time.time()
        path_length_base = 0
        path_length_eef = 0
        init_q = self.robot_model.get_q_from_sim()
        pose_ = self.robot_model.Kinematics.fkm(init_q)
        prev_robot_base_position = np.array(
            [init_q[0], init_q[1]], dtype=np.float128)
        prev_robot_eef_position = np.array([vec3(translation(pose_))[0], vec3(
            translation(pose_))[1], vec3(translation(pose_))[2]], dtype=np.float128)

        # Convert NBV to dq pose
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
        update_freq = 300
        # Check if the virtual obstacle worked
        if obstacles_dic is None:
            obs_pose_list = self.obs_pose_list_dq
            obs_rad_list = self.obs_rad_list
        else:
            vobs_pose_list = []
            vobs_radius_list = []

            for key, value in obstacles_dic.items():
                vobs = value["virtual_obstacles"]
                if vobs is not None:
                    for obs_ in vobs:
                        vobs_pose_list.append(
                            1+E_*0.5*(obs_[0]*i_+obs_[1]*j_))
                        vobs_radius_list.append(obs_[2])

            obs_pose_list = self.obs_pose_list_dq+vobs_pose_list
            obs_rad_list = self.obs_rad_list+vobs_radius_list
            obs_dir_list = []
            for id in range(len(obs_pose_list)):
                for i, (key, val) in enumerate(obstacles_dic.items()):
                    if id in val["idx"]:
                        obs_dir_list.append(group_dir[i])

        # Create array of the all obstacle positions and their radius
        obsxy_rad_list = []
        for pse, rad in zip(obs_pose_list, obs_rad_list):
            psn = vec3(translation(pse))
            obsxy_rad_list.append([psn[0], psn[1], rad])

        # If the IPP is activated
        if self.enable_rrt_based_ipp:

            # Keep trying to find a proper path until it finds one
            is_plan_found = False
            rrt_fail_count = 0
            while not is_plan_found:
                nbv_position_np = vec3(translation(target_nbv))
                current_position = self.camera_ref.get_position()

                ipp = IPP(start=[current_position[0], current_position[1], current_position[2]],
                          goal=[nbv_position_np[0], nbv_position_np[1], nbv_position_np[2]], max_iter=5000, expand_dis=0.5, obsxyr_list=obsxy_rad_list)
                is_plan_found, rrt_result_dic = ipp.get_ipp_result()
                if not is_plan_found:
                    print("RRT Failed")
                    rrt_fail_count += 1
                if rrt_fail_count >= 10:
                    is_plan_found = False
                    break
            if not is_plan_found:
                return False
            # Rule out the path nodes that are too close to target and initial positions
            rrt_path = []
            dist_threshold = 1
            for rrt_pstn in np.array(rrt_result_dic["rrt_positions"]):
                dist_to_start = np.linalg.norm(rrt_pstn-current_position)
                dist_to_target = np.linalg.norm(rrt_pstn-nbv_position_np)
                if not (dist_to_start < dist_threshold or dist_to_target < dist_threshold):
                    rrt_path.append(rrt_pstn)
            # Reverse the path node list to make it from start to finish
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
                # After showing the nodes, wait a little bit
                for _ in range(100):
                    self.pr.step()
                    time.sleep(0.01)

            # Show RRT nodes and draw lines between the nodes
            if self.params["Visual"]["showAllNodes"]:
                # First draw the lines between nodes
                for rrt_node in rrt_result_dic["rrt_node_list"][:70]:
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
                        if i % update_freq == 0:
                            self.pr.step()
                        self.rrt_shape_list.append(target_a1)
                        self.rrt_shape_list.append(target_a2)
                        self.rrt_shape_list.append(target_cyl)
                # Visualize tree nodes
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

        # If it is the first NBV, save data for the initial state of the robot(First recording)
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
            # convert numpy array of current view point cloud to o3d point cloud
            current_pcl_o3d = o3d.geometry.PointCloud()
            current_pcl_o3d.points = o3d.utility.Vector3dVector(
                xyz_np)

            remaining_model_o3d = o3d.geometry.PointCloud()
            remaining_model_o3d.points = o3d.utility.Vector3dVector(
                self.gt_pcl_o3d)
            # calculate the distance between remaining model and view pcl
            dists = remaining_model_o3d.compute_point_cloud_distance(
                current_pcl_o3d)
            dists = np.asarray(dists)
            dist_filter = dists <= 0.008
            # view coverage is the amount of coverage that the view
            # has with remaining gt model which reflects the importance of the view.
            cov_o3d = np.sum(dist_filter == True)/self.total_point_o3d
            self.total_cov_o3d += cov_o3d
            self.coverage_list_o3d.append(self.total_cov_o3d)
            self.gt_pcl_o3d = self.gt_pcl_o3d[~dist_filter]

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
        self.distance_list.append(nbv_dist)  # Save data to to distance_list

        # To calculate focus point if it is activated
        # Find the direction from camera centre to search space cylinder centre.
        # It ensures that the robot find a focus point in a suitable position
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

        else:
            vis_target_position = 0.45*k_
            self.focus_target.set_position([0, 0, 0.45])
            self.pr.step()

        # Get the current position of the eef where a focus point is calculated
        self.previous_position_focus = self.camera_ref.get_position()
        self.focus_prev_ref.set_position(self.previous_position_focus)

        # This ensures that the robot visits all the nodes on the path when IPP method is activated
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
            initial_base_position = np.array(youbot_q[:2])

            while err > self.controller_err_tolerance:
                trans = self.camera_ref.get_position()
                if self.enable_visibility_constraint:
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
                if self.enable_focus_point and dist_to_previous > self.params["NBV"]["focusUpdateDistance"] and dist_to_goal > 0.0*self.params["NBV"]["focusUpdateDistance"]:
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

                eef_line = vec3(get_direction(pose_, k_))
                target_line = vec3(get_direction(robot_target_pose, k_))
                direction_err = eef_line - target_line

                if self.is_pose_control:
                    whole_J = pose_J
                    whole_err = pose_err
                else:
                    whole_J = np.vstack([line_J, tra_J])
                    whole_err = np.vstack([direction_err[:, np.newaxis],
                                           tra_err[:, np.newaxis]])

                hard_const_J = {}
                hard_const_b = {}

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

                    hard_const_J["soft"] = J_soft
                    hard_const_J["slack"] = J_slack

                    hard_const_b["soft"] = b_soft
                    hard_const_b["slack"] = b_slack

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
                    hard_const_J["limits"] = J_limit_whole
                    hard_const_b["limits"] = b_limit_whole

                # Add the constraint for the workspace always
                if self.enable_collision_constraints:

                    J_base, b_base = compute_base_constraints_p2p(pose_list=obs_pose_list + [DQ([1])], radius_list=obs_rad_list+[self.search_space_radius-0.15],
                                                                  robot_kin=self.robot_model.Kinematics, youbot_q=youbot_q)

                    # Calculate softmin without search space cylinder constraint
                    # D is fed to softmin to calculate softminimum
                    # Tangent vector is calculated based on ΔD
                    softmin_b_base = softmin(b_base[:-1])
                    softmin_J_base = softmin_gradient(
                        b_base[:-1], J_base[:-1, :])
                    softmin_J_base = softmin_J_base[None, :]
                    self.softmin_J_base = softmin_J_base[:]
                    self.softmin_b_base = softmin_b_base
                    if self.enable_visibility_constraint:
                        softmin_J_base = np.concatenate(
                            (softmin_J_base, np.zeros((softmin_J_base.shape[0], slack_count))), axis=1)

                    hard_const_J["D"] = -softmin_J_base
                    hard_const_b["D"] = np.array([softmin_b_base])

                    # We consider search space cylinder as seperate constraint
                    if self.enable_visibility_constraint:
                        J_base = np.concatenate(
                            (J_base, np.zeros((J_base.shape[0], slack_count))), axis=1)

                    J_search = -J_base[-1, :]
                    J_search = J_search[None, :]
                    b_search = np.array([b_base[-1]])
                    hard_const_J["search"] = J_search
                    hard_const_b["search"] = b_search

                    # Compute eef collision constraint
                    cylinder_dq = pose_to_line(DQ([1]), k_)
                    position_eef = translation(pose_)
                    J_eef = self.robot_model.Kinematics.point_to_line_distance_jacobian(
                        tra_J, position_eef, cylinder_dq)
                    if self.enable_visibility_constraint:
                        J_eef = np.concatenate(
                            (J_eef, np.zeros((J_eef.shape[0], slack_count))), axis=1)

                    b_eef = DQ_Geometry.point_to_line_squared_distance(
                        position_eef, cylinder_dq) - (self.search_space_radius-0.25)**2
                    hard_const_J["eef"] = -J_eef
                    hard_const_b["eef"] = np.array([b_eef])

                    if self.used_strategy == "no_path":

                        # If the not path strategy is used, the robot might move to unsafe positions.
                        # Thus, we limit the maximum and minimum height
                        plane_pose_ = 1+0.5*E_*0.25*k_
                        plane_ = pose_to_plane(plane_pose_, k_)
                        J_eef_plane = self.robot_model.Kinematics.point_to_plane_distance_jacobian(
                            tra_J, position_eef, plane_)

                        if self.enable_visibility_constraint:
                            J_eef_plane = np.concatenate(
                                (J_eef_plane, np.zeros((J_eef_plane.shape[0], slack_count))), axis=1)

                        b_eef_plane = DQ_Geometry.point_to_plane_distance(
                            position_eef, plane_) - 0.05
                        hard_const_J["eef_lp"] = -J_eef_plane
                        hard_const_b["eef_lp"] = np.array([b_eef_plane])

                        plane_pose_ = 1+0.5*E_*0.75*k_
                        plane_ = pose_to_plane(plane_pose_, -k_)
                        J_eef_plane = self.robot_model.Kinematics.point_to_plane_distance_jacobian(
                            tra_J, position_eef, plane_)

                        if self.enable_visibility_constraint:
                            J_eef_plane = np.concatenate(
                                (J_eef_plane, np.zeros((J_eef_plane.shape[0], slack_count))), axis=1)

                        b_eef_plane = DQ_Geometry.point_to_plane_distance(
                            position_eef, plane_) - 0.05
                        hard_const_J["eef_hp"] = -J_eef_plane
                        hard_const_b["eef_hp"] = np.array([b_eef_plane])

                if self.enable_circulation_constraints:

                    obs_indexes_, dist_list_obs_ = compute_clossest_obs_wb(
                        obs_pose_list, obs_rad_list, youbot_q)

                    if dist_list_obs_[obs_indexes_[0]] < 0.2 and np.linalg.norm(tra_err) > 0.8:
                        if obstacles_dic is not None:
                            # Find the group of the closest obstacle
                            for i, (key, val) in enumerate(obstacles_dic.items()):
                                if obs_indexes_[0] in val["idx"]:
                                    closest_group_key = key
                                    break

                            # Find the distance between centre and closest obstacle in the group
                            min_dist_to_centre_ = 10
                            min_dist_vec = []
                            obs_positions = obstacles_dic[closest_group_key]["psn_workspace"]
                            obs_idxs = obstacles_dic[closest_group_key]["idx"]
                            for obs_position_rad, obs_ix in zip(obs_positions, obs_idxs):
                                obs_x = obs_position_rad[0]
                                obs_y = obs_position_rad[1]
                                rad_ = obs_position_rad[2]
                                dist_ = (math.sqrt((obs_x)**2 +
                                                   (obs_y)**2)-self.search_space_radius-rad_)
                                if dist_ < min_dist_to_centre_:
                                    min_dist_to_centre_ = dist_
                                    min_dist_vec = np.array(
                                        [-obs_x, -obs_y, 0])
                                    min_dist_vec = dist_*min_dist_vec / \
                                        np.linalg.norm(min_dist_vec)

                            base_target = vec3(translation(robot_target_pose))
                            base_target[2] = 0

                            angle_rad = cross_product_2d(
                                base_target, initial_base_position)
                            circ_dir = 1
                            if angle_rad < 0:
                                circ_dir = -1

                            if min_dist_to_centre_ > self.robot_diam:
                                circ_dir = -circ_dir

                            robot_base_pose = self.robot_model.BaseKinematics.raw_fkm(
                                youbot_q)
                            robot_base_position = vec3(
                                translation(robot_base_pose))

                            J_tang_list = []
                            b_tang_list = []

                            norm_vec_list = []
                            dist_list = []

                            dir_tan = circ_dir
                            obs_pose, _ = obs_pose_list[obs_indexes_[
                                0]], obs_rad_list[obs_indexes_[0]]

                            obs_position = vec3(translation(obs_pose))
                            if np.linalg.norm(base_target-obs_position) > 0.4:

                                x_diff = robot_base_position[0]-obs_position[0]
                                y_diff = robot_base_position[1]-obs_position[1]
                                dist_vec = np.array([x_diff, y_diff, 0])

                                dist_norm = dist_vec/np.linalg.norm(dist_vec)

                                norm_vec_list.append(dist_norm)
                                omega = np.array([[0, 1, 0],
                                                  [-1, 0, 0],
                                                  [0, 0, 0],
                                                  ])

                                tan_vect = omega@self.softmin_J_base[:, :3].T
                                tan_vect_norm = dir_tan * \
                                    tan_vect/np.linalg.norm(tan_vect)

                                tan_vect_norm = np.squeeze(
                                    tan_vect_norm, axis=1)
                                J_tang_list.append(
                                    tan_vect_norm.tolist()+5*[0])
                                D = self.softmin_b_base
                                dist_list.append(D)
                                b = self.b
                                D_0 = self.D_0

                                beta_ = b*(1-D/D_0)

                                b_tang_list.append(beta_)

                                J_tang = np.array(J_tang_list)
                                b_tang = np.array(b_tang_list)

                                if self.enable_visibility_constraint:
                                    J_tang = np.concatenate(
                                        (J_tang, np.zeros((J_tang.shape[0], slack_count))), axis=1)
                                hard_const_J["circ"] = -J_tang
                                hard_const_b["circ"] = -b_tang

                                # If the robot starts circulation, disable search space constraint
                                del hard_const_J["search"]
                                del hard_const_b["search"]

                hard_const_J_list = []
                hard_const_b_list = []
                for (constJ, J), (constb, bt) in zip(hard_const_J.items(), hard_const_b.items()):
                    hard_const_J_list.append(J)
                    hard_const_b_list.append(bt)

                if len(hard_const_J_list) == 0:
                    u = self.qp_solver.compute_control_signal(
                        whole_J, whole_err, Aineq=None, bineq=None, gain=self.controller_gain,
                        damping=self.controller_damping)
                else:
                    hard_const_J = np.concatenate(hard_const_J_list, axis=0)
                    hard_const_b = np.concatenate(hard_const_b_list, axis=0)
                    try:
                        if self.enable_visibility_constraint:
                            u = self.qp_solver.compute_control_signal(
                                whole_J_slack, whole_err_slack, Aineq=hard_const_J, bineq=hard_const_b, gain=self.controller_gain,
                                damping=self.controller_damping)
                        else:
                            u = self.qp_solver.compute_control_signal(
                                whole_J, whole_err, Aineq=hard_const_J, bineq=hard_const_b, gain=self.controller_gain,
                                damping=self.controller_damping)
                    except:
                        print("Controller Fails")
                        termination_cause = "CONTROLLER"
                        break

                base_phi = youbot_q[2]
                rot_mat = np.array([[math.cos(base_phi), math.sin(base_phi), 0],
                                    [-math.sin(base_phi),
                                    math.cos(base_phi), 0],
                                    [0, 0, 1]])
                u = u[:8]
                base_vel_1 = rot_mat@u[:3]

                u[:3] = base_vel_1
                self.robot_model.send_velocities(u)
                self.pr.step()

                _q = self.robot_model.get_q_from_sim()
                current_robot_base_position = np.array(
                    [_q[0], _q[1]], dtype=np.float128)
                current_robot_eef_position = np.array([vec3(translation(pose_))[0], vec3(
                    translation(pose_))[1], vec3(translation(pose_))[2]], dtype=np.float128)

                travelled_dist_base = np.linalg.norm(
                    current_robot_base_position - prev_robot_base_position)
                travelled_dist_eef = np.linalg.norm(
                    current_robot_eef_position - prev_robot_eef_position)

                path_length_base += travelled_dist_base
                path_length_eef += travelled_dist_eef
                prev_robot_base_position = current_robot_base_position
                prev_robot_eef_position = current_robot_eef_position

                err = np.linalg.norm(whole_err)

                youbot_q = self.robot_model.get_q_from_sim()

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

        self.base_path_len_list.append(float(path_length_base))
        self.eef_path_len_list.append(float(path_length_eef))
        robot_motion_time = time.time()-nbv_start_time
        # Time passed from the NBV call to moving NBV
        self.robot_motion_time_list.append(robot_motion_time)
        self.termination_cause_list.append(termination_cause)
        self.travel_time = time.time()-self.movement_start
        # The time passed to reach NBV starting from control loop
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

        current_pcl_o3d = o3d.geometry.PointCloud()
        current_pcl_o3d.points = o3d.utility.Vector3dVector(
            xyz_np)

        remaining_model_o3d = o3d.geometry.PointCloud()
        remaining_model_o3d.points = o3d.utility.Vector3dVector(
            self.gt_pcl_o3d)
        # calculate the distance between remaining model and view pcl
        dists = remaining_model_o3d.compute_point_cloud_distance(
            current_pcl_o3d)
        dists = np.asarray(dists)
        dist_filter = dists <= 0.008

        # view coverage is the amount of coverage that the view
        # has with remaining gt model which reflects the importance of the view.
        cov_o3d = np.sum(dist_filter == True)/self.total_point_o3d
        self.total_cov_o3d += cov_o3d
        self.coverage_list_o3d.append(self.total_cov_o3d)
        self.gt_pcl_o3d = self.gt_pcl_o3d[~dist_filter]

        print("Current Coverage o3d", self.total_cov_o3d)

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
                "coverage_list_o3d": self.coverage_list_o3d,
                "entropy_list": self.entropy_list,
                "base_path_len_list": self.base_path_len_list,
                "eef_path_len_list": self.eef_path_len_list,
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
                "best_ig_val_list": self.ig_max_list,
                "obs_dic": serialize_obs_dic(self.obs_groups)

            }

            file = self.result_save_path + f"/{self.used_strategy}"
            with open(file, 'w') as f:
                json.dump(data, f, indent=4)
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
