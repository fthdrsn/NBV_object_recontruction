# Standard library imports
import dis
import sys
from os.path import abspath, dirname
from turtle import right

# Third-party imports
import numpy as np
from dqrobotics import *
from dqrobotics.robot_modeling import DQ_HolonomicBase
from dqrobotics.utils import DQ_Geometry

# Local package imports
from QP import DQ_QuadprogSolver_Custom
from Robots.utils import *


class RobotController:
    """Controller for a holonomic robot with visibility, velocity limit, collision avoidance, and circulation constraints."""

    def __init__(self, params):
        self.robot_dim = 0.7
        self.robot_model = YouBotModel(robot_dim=self.robot_dim)
        self.qp_solver = DQ_QuadprogSolver_Custom()
        self.params = params

        self.controller_params = params["ControllerSettings"]
        self.constraint_switches = params["ConstraintSwitches"]
        self.visibility_constraint_params = params["VisibilityConstraint"]
        self.robot_limits = params["RobotConstraints"]
        self.collision_avoidance_params = params["CollisionAvoidance"]
        self.circulation_params = params["CirculationConstraint"]

        self.left_plane_pose, self.right_plane_pose, self.up_plane_pose, self.down_plane_pose = get_camera_fov_planes(
            self.params["CameraParameters"])

    def compute_one_step_u(self, target_pose_dq, robot_q, obs_pose_list, obs_radius_list, focus_point=[0, 0, 0], circulation_dir=1, method="NOPATH"):
        """Compute the control signal for one time step given the target, robot state, and obstacles.
        Args:
             target (dict): Target pose with keys 'position' (list of x, y) and 'phi' (orientation in radians).
             robot (dict): Robot state with keys 'position' (list of x, y), 'phi' (orientation in radians), and 'radius' (float).
             obs_dic (list): List of obstacle dictionaries, each with keys depending on the type ('circle' or 'polygon').
             focus_point (list): The point [x, y] that should remain visible to the robot.
             circulation_dir (float or None): Direction for circulation constraint; if None, use default.
        Returns:
             u (np.array): Control signal [vx, vy, omega].
             const_dic (dict): Dictionary of active constraints with their Jacobians and bounds.
             task_err (np.array): Task error vector.
        """

        robot_pose_J = self.robot_model.Kinematics.pose_jacobian(
            robot_q)  # Calculate the pose Jacobian of the robot end effector
        robot_pose_dq = self.robot_model.Kinematics.fkm(robot_q)
        line_J = self.robot_model.Kinematics.line_jacobian(
            # Line (z-axis of the robot end effector) jacobian
            robot_pose_J, robot_pose_dq, k_)
        line_J = line_J[1:4]

        # Translation Jacobian of the robot end effector
        tra_J = self.robot_model.Kinematics.translation_jacobian(
            robot_pose_J, robot_pose_dq)
        tra_J = tra_J[1:4, :]

        # Translation error between the robot end effector and the target pose
        tra_err = vec3(translation(robot_pose_dq)) - \
            vec3(translation(target_pose_dq))

        eef_line = vec3(get_direction(robot_pose_dq, k_))
        target_line = vec3(get_direction(target_pose_dq, k_))
        direction_err = eef_line - target_line
        # Stack line and translation Jacobians and errors to form the whole task of position-direction control
        task_J = np.vstack([line_J, tra_J])
        task_err = np.vstack([direction_err[:, np.newaxis],
                              tra_err[:, np.newaxis]])

        const_dic = {}
        # Add constraints based on the enabled switches

        if self.constraint_switches["enableVisibilityConst"]:
            # Add slack variables for soft visibility constraint
            W_visibility = self.visibility_constraint_params["softAlpha"] * \
                np.linalg.norm(
                    tra_err)**self.visibility_constraint_params["softBeta"]
            slack_count = 4
            task_J = np.block([
                [task_J, np.zeros((task_J.shape[0], slack_count))],
                [np.zeros((slack_count, task_J.shape[1])),
                    W_visibility * np.eye(slack_count)]
            ])
            task_err = np.concatenate(
                [task_err, np.zeros((slack_count, 1))], axis=0)
            J_vis, b_vis = self.visibility_constraint(
                robot_q, focus_point)
            const_dic["visibility"] = (J_vis, b_vis)

        if self.constraint_switches["enableVelocityConst"]:
            J_vel, b_vel = self.velocity_limit_constraint()
            const_dic["velocity"] = (J_vel, b_vel)

        if self.constraint_switches["enableJointConst"]:
            J_joint, b_joint = self.joint_position_limit_constraint(
                robot_q)
            const_dic["joint_position"] = (J_joint, b_joint)

        if self.constraint_switches["enableCollisionConst"]:
            J_col, b_col = self.collision_avoidance_constraint_base(
                obs_pose_list, obs_radius_list, robot_q)
            const_dic["collision_base"] = (J_col, b_col)

            J_eef_col, b_eef_col = self.eef_collision_avoidance_constraint(
                robot_q)
            const_dic["collision_eef"] = (J_eef_col, b_eef_col)

            if method == "NOPATH":
                J_eef_plane, b_eef_plane = self.eef_up_down_limit_constraint(
                    robot_q)
                const_dic["eef_up_down"] = (J_eef_plane, b_eef_plane)

        if self.constraint_switches["enableCirculationConst"]:
            col_J, col_b = const_dic["collision_base"]
            col_J = col_J[0:1]
            col_b = col_b[0:1]
            J_circ, b_circ = self.circulation_constraint(
                col_J, col_b, circulation_dir=circulation_dir)
            if np.linalg.norm(tra_err) < self.circulation_params["noCirculationRadius"] or circulation_dir == 0:
                J_circ = np.zeros_like(J_circ)
                b_circ = np.array([0.0])
            else:
                const_dic["collision_base"] = (col_J, col_b)

            const_dic["circulation"] = (J_circ, b_circ)

        const_J_list = []
        const_b_list = []

        for cont, (J, b) in const_dic.items():
            if self.constraint_switches["enableVisibilityConst"]:
                if J.shape[1] < 12:  # We use this to filter our visibility constraint
                    # since we have already included slack variables to it.
                    J = np.concatenate(
                        (J, np.zeros((J.shape[0], slack_count))), axis=1)
            const_J_list.append(J)
            const_b_list.append(b)

        A = None
        b = None
        if len(const_J_list) != 0:
            A = np.concatenate(const_J_list, axis=0)
            b = np.concatenate(const_b_list, axis=0)
        try:
            u = self.qp_solver.compute_control_signal(
                task_J, task_err, gain=self.controller_params["controllerGain"], Aineq=A, bineq=b,
                damping=self.controller_params["controllerDamping"])
        except:
            print("QP solver failed, returning zero control signal")
            u = np.zeros(8)

        return u, const_dic, task_err

    def circulation_constraint(self, col_J, col_b, circulation_dir=1):
        """Compute the circulation constraint based on collision avoidance Jacobian and bounds.
        Args:
             col_J (np.array): Collision avoidance constraint Jacobian.
             col_b (np.array): Collision avoidance constraint bounds.
             circulation_dir (float or None): Direction for circulation constraint; if None, use default.
        Returns:
             J_c (np.array): Circulation constraint Jacobian.
             b_c (float): Circulation constraint bound.
        """
        omega = np.array([[0, 1, 0],
                          [-1, 0, 0],
                          [0, 0, 0]])

        tan_vect = omega@col_J[:, :3].T
        b = self.circulation_params["b"]
        D_0 = self.circulation_params["D0"]
        D = col_b
        beta_ = b*(1-D/D_0)
        J_c = circulation_dir*tan_vect.T/np.linalg.norm(tan_vect)
        J_c = np.concatenate((J_c, np.zeros((1, 5))), axis=1)
        b_c = -beta_
        return (J_c, b_c)

    def collision_avoidance_constraint_base(self, obs_pose_list, obs_radius_list, robot_q):
        """Compute the collision avoidance constraint based on obstacles and robot state.
        Args:
             obs_dic (list): List of obstacle dictionaries, each with keys depending on the type ('circle' or 'polygon').
             robot (dict): Robot state with keys 'position' (list of x, y), 'phi' (orientation in radians), and 'radius' (float).
        Returns:
             J_col (np.array): Collision avoidance constraint Jacobian.
             b_col (np.array): Collision avoidance constraint bounds.
        """

        J_base, b_base = compute_base_constraints_p2p(pose_list=obs_pose_list + [DQ([1])], radius_list=obs_radius_list+[2.9],
                                                      robot_kin=self.robot_model.Kinematics, youbot_q=robot_q)

        softmin_b_base = softmin(
            b_base, h=self.collision_avoidance_params["softminH"], delta=self.collision_avoidance_params["softminDelta"])
        softmin_J_base = softmin_gradient(
            b_base, J_base, h=self.collision_avoidance_params["softminH"])
        softmin_J_base = softmin_J_base[None, :]

        return (-softmin_J_base, np.array([softmin_b_base]))

    def eef_collision_avoidance_constraint(self, robot_q):
        cylinder_dq = pose_to_line(DQ([1]), k_)
        pose_ = self.robot_model.Kinematics.fkm(robot_q)
        tra_J = self.robot_model.Kinematics.translation_jacobian(
            self.robot_model.Kinematics.pose_jacobian(robot_q), pose_)
        position_eef = translation(pose_)
        J_eef = self.robot_model.Kinematics.point_to_line_distance_jacobian(
            tra_J, position_eef, cylinder_dq)

        b_eef = DQ_Geometry.point_to_line_squared_distance(
            position_eef, cylinder_dq) - (2.75)**2
        return (-J_eef, np.array([b_eef]))

    def eef_up_down_limit_constraint(self, robot_q):
        # If the not path strategy is used, the robot might move to unsafe positions.
        # Thus, we limit the maximum and minimum height
        plane_pose_ = 1+0.5*E_*0.25*k_
        plane_ = pose_to_plane(plane_pose_, k_)
        pose_ = self.robot_model.Kinematics.fkm(robot_q)
        tra_J = self.robot_model.Kinematics.translation_jacobian(
            self.robot_model.Kinematics.pose_jacobian(robot_q), pose_)
        position_eef = translation(pose_)
        J_eef_plane_down = self.robot_model.Kinematics.point_to_plane_distance_jacobian(
            tra_J, position_eef, plane_)
        b_eef_plane_down = DQ_Geometry.point_to_plane_distance(
            position_eef, plane_) - 0.05

        plane_pose_ = 1+0.5*E_*0.75*k_
        plane_ = pose_to_plane(plane_pose_, -k_)
        J_eef_plane_up = self.robot_model.Kinematics.point_to_plane_distance_jacobian(
            tra_J, position_eef, plane_)

        b_eef_plane_up = DQ_Geometry.point_to_plane_distance(
            position_eef, plane_) - 0.05
        return (np.concatenate((-J_eef_plane_down, -J_eef_plane_up), axis=0),
                np.array([b_eef_plane_down, b_eef_plane_up]))

    def velocity_limit_constraint(self):
        """Compute the velocity limit constraint based on robot velocity limits.
        Returns:
             J_limit_vel (np.array): Velocity limit constraint Jacobian.
             b_limit_vel (np.array): Velocity limit constraint bounds.
        """
        robot_vel_lim = self.robot_limits["velLimBase"] + \
            self.robot_limits["velLimArm"]
        J_limit_vel = np.concatenate(
            (np.eye(8, 8), -np.eye(8, 8)), axis=0)
        b_limit_vel = np.array(
            robot_vel_lim + robot_vel_lim)
        return (J_limit_vel, b_limit_vel)

    def joint_position_limit_constraint(self, robot_q):
        """Compute the joint position limit constraint based on robot joint limits.
        Returns:
             J_limit_angle (np.array): Joint position limit constraint Jacobian.
             b_limit_angle (np.array): Joint position limit constraint bounds.
        """

        # Arm Joint Position Limits
        self.joint_pos_max = np.array(
            [ang*pi/180 for ang in self.params["RobotConstraints"]["armJointLimMax"]])
        self.joint_pos_min = np.array(
            [ang*pi/180 for ang in self.params["RobotConstraints"]["armJointLimMin"]])

        # Limit the positions
        B_mat = np.concatenate(
            (np.zeros((5, 3)), np.eye(5)), axis=1)
        J_limit_angle = np.concatenate((B_mat, -B_mat), axis=0)
        b_limit_angle = np.concatenate(
            (-(robot_q[3:]-self.joint_pos_max), (robot_q[3:]-self.joint_pos_min)), axis=0)
        return (J_limit_angle, b_limit_angle)

    def visibility_constraint(self, robot_q, vis_target_position):

        # Calculate the pose of the planes w.r.t world frame, and their plane jacobians
        left_plane_pose, left_plane_J = calculate_plane_jacobian(
            self.robot_model.Kinematics, robot_q, self.left_plane_pose)
        right_plane_pose, right_plane_J = calculate_plane_jacobian(
            self.robot_model.Kinematics, robot_q, self.right_plane_pose)
        up_plane_pose, up_plane_J = calculate_plane_jacobian(
            self.robot_model.Kinematics, robot_q, self.up_plane_pose)
        down_plane_pose, down_plane_J = calculate_plane_jacobian(
            self.robot_model.Kinematics, robot_q, self.down_plane_pose)

        # Calculate planes in dual quaternion form their poses and normal vector
        left_plane = pose_to_plane(left_plane_pose, k_)
        right_plane = pose_to_plane(right_plane_pose, k_)
        up_plane = pose_to_plane(up_plane_pose, k_)
        down_plane = pose_to_plane(down_plane_pose, k_)
        vis_target_position_dq = vis_target_position[0]*i_ + \
            vis_target_position[1]*j_ + vis_target_position[2]*k_
        # Calculate the distance Jacobians and distances from the visibility target point to each plane
        left_dist_J = self.robot_model.Kinematics.plane_to_point_distance_jacobian(
            left_plane_J, vis_target_position_dq)
        right_dist_J = self.robot_model.Kinematics.plane_to_point_distance_jacobian(
            right_plane_J, vis_target_position_dq)
        up_dist_J = self.robot_model.Kinematics.plane_to_point_distance_jacobian(
            up_plane_J, vis_target_position_dq)
        down_dist_J = self.robot_model.Kinematics.plane_to_point_distance_jacobian(
            down_plane_J, vis_target_position_dq)

        dist2left = DQ_Geometry.point_to_plane_distance(
            vis_target_position_dq, left_plane) - self.visibility_constraint_params["safeDistVisibility"]
        dist2right = DQ_Geometry.point_to_plane_distance(
            vis_target_position_dq, right_plane) - self.visibility_constraint_params["safeDistVisibility"]
        dist2up = DQ_Geometry.point_to_plane_distance(
            vis_target_position_dq, up_plane) - self.visibility_constraint_params["safeDistVisibility"]
        dist2down = DQ_Geometry.point_to_plane_distance(
            vis_target_position_dq, down_plane) - self.visibility_constraint_params["safeDistVisibility"]
        J_visibility = np.concatenate((left_dist_J, right_dist_J,
                                       up_dist_J, down_dist_J), axis=0)
        b_visibility = np.array([dist2left, dist2right, dist2up, dist2down])

        # Make the visibility constraint soft by adding slack variables
        slack_count = 4
        J_soft_1 = np.concatenate((J_visibility, np.zeros(
            (J_visibility.shape[0], slack_count))), axis=1)
        J_soft_2 = np.concatenate((np.zeros_like(J_visibility), np.eye(
            J_visibility.shape[0], slack_count)), axis=1)
        J_soft = -J_soft_1-J_soft_2
        b_soft = b_visibility
        J_slack = -J_soft_2
        b_slack = np.array(slack_count*[0])

        return (np.concatenate((J_soft, J_slack), axis=0), np.concatenate((b_soft, b_slack), axis=0))
