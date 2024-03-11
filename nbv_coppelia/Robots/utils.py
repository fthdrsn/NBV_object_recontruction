from Robots.RobotsInteraction import *
from dqrobotics import *
import numpy as np
from scipy.spatial.transform import Rotation as R
from pyquaternion import Quaternion
from dqrobotics.utils import DQ_Geometry
pi2 = math.pi/2


def compute_base_constraints(pose_list, radius_list, robot_kin, youbot_q):
    youbot_q = np.array(youbot_q)

    youbot_base = robot_kin.get_chain_as_holonomic_base(0)
    # Do not take into account frame_displacement where arm connected(find fkm w.r.t ref frame of base)
    youbot_base_pose = youbot_base.raw_fkm(youbot_q)
    # fkm =raw_fkm(q)*frame_displacement
    # pose_jacob=haminus8(frame_displacement_)*raw_pose_jacob
    # Do not take into account frame_displacement where arm connected(find jacob w.r.t ref frame of base)
    youbot__base_Jx = youbot_base.raw_pose_jacobian(youbot_q, 2)

    t_inner = translation(youbot_base_pose)
    base_jt = robot_kin.translation_jacobian(
        youbot__base_Jx, youbot_base_pose)
    Jt = np.concatenate((base_jt, np.zeros((4, 5))), axis=1)
    b_constraint = []
    eta = 1
    idx = 0
    for pose_, rad_ in zip(pose_list, radius_list):

        j_dist = robot_kin.point_to_line_distance_jacobian(
            Jt, t_inner, Ad(pose_, k_))
        dist = DQ_Geometry.point_to_line_squared_distance(
            t_inner, Ad(pose_, k_)) - (rad_ + 0.35) ** 2

        if idx == 0:
            j_constraint = np.array(j_dist)
        else:
            j_constraint = np.concatenate((j_constraint, j_dist), axis=0)
        b_constraint.append(eta*dist)
        idx += 1

    return j_constraint, np.array(b_constraint)


def compute_eef_constraints(pose_list, radius_list, robot_kin, youbot_q):

    # Compute eef collision constraint with search space cylinder (point to line jacobian)
    youbot_q = np.array(youbot_q)
    cylinder_dq = pose_to_line(pose_list[0], k_)
    pose_ = robot_kin.fkm(youbot_q)
    position_eef = translation(pose_)
    pose_J = robot_kin.pose_jacobian(youbot_q)
    tra_J = robot_kin.translation_jacobian(
        pose_J, pose_)
    J_eef_cyl = robot_kin.point_to_line_distance_jacobian(
        tra_J, position_eef, cylinder_dq)

    b_eef_cyl = DQ_Geometry.point_to_line_squared_distance(
        position_eef, cylinder_dq) - radius_list[0]**2
    # Compute constraints between eef and work space obstacles(we model obtsancles as sphere since the line is infinite)
    b_constraint = []
    eta = 1
    idx = 0
    for pose_, rad_ in zip(pose_list[1:], radius_list[1:]):

        j_dist = robot_kin.point_to_point_distance_jacobian(
            tra_J, position_eef, translation(pose_))
        dist = DQ_Geometry.point_to_point_squared_distance(
            position_eef, translation(pose_)) - (rad_+0.05) ** 2

        if idx == 0:
            j_constraint = np.array(j_dist)
        else:
            j_constraint = np.concatenate((j_constraint, j_dist), axis=0)
        b_constraint.append(eta*dist)
        idx += 1
    j_constraint = np.concatenate((j_constraint, J_eef_cyl), axis=0)
    b_constraint.append(eta*b_eef_cyl)
    return j_constraint, np.array(b_constraint)


def pose_to_line(line_pose, direction):
    p = translation(line_pose)
    r = rotation(line_pose)
    l = Ad(r, direction)
    m = cross(p, l)
    return l + E_ * m


def get_direction(line_pose, direction):
    p = translation(line_pose)
    r = rotation(line_pose)
    l = Ad(r, direction)

    return l


def pose_to_plane(plane_pose, normal):
    p = translation(plane_pose)
    r = rotation(plane_pose)
    n = Ad(r, normal)
    d = dot(p, n)
    return n + E_ * d


def direction_to_pose(direction):
  # Normalize the direction vector
    direction /= np.linalg.norm(direction)

    # Calculate the y-axis as the cross product of the direction vector and the [0, 1, 0] vector
    right = np.cross(np.array([0, 0, 1]), direction)
    right /= np.linalg.norm(right)

    # Calculate the x-axis as the cross product of the up vector and the direction vector
    y_vec = np.cross(direction, right)
    y_vec /= np.linalg.norm(y_vec)
    # Create the transformation matrix
    pose = np.eye(3)

    pose[:, 0] = right
    pose[:, 1] = y_vec
    pose[:, 2] = direction
    quat = Quaternion(matrix=pose)

    return normalize(quat[0]+quat[1]*i_+quat[2]*j_+quat[3]*k_)


def generate_points_on_cylinder(theta, cly_centre=(0, 0, 0), z=0.2, r=0.7):
    x = r*math.cos(theta)
    y = r*math.sin(theta)

    pose_point = (1+E_*0.5*(x*i_+y*j_+z*k_)) * \
        (1+E_*0.5*(cly_centre[0]*i_+cly_centre[1]*j_+cly_centre[2]*k_))

    return pose_point


def transform_to_image_plane(pnt, im_width=256, im_height=256, cam_fov=57):
    A = max(im_width, im_height)/2
    focal_length = A/math.tan(0.5*math.pi*cam_fov/180)
    ox = im_width/2  # In pixel
    oy = im_height/2
    x = pnt[0]
    y = pnt[1]
    # Project the centre point of the sphere on image plane
    x_im_plane = -(x-ox)/focal_length*0.1
    y_im_plane = -(y-oy)/focal_length*0.1
    z_im_plane = 1*0.1
    return [x_im_plane, y_im_plane, z_im_plane]


def get_camera_fov_planes(im_width, im_height, cam_fov):

    tra_left = transform_to_image_plane(
        [0, im_height/2], im_width, im_height, cam_fov)
    tra_left_dq = tra_left[0]*i_+tra_left[1]*j_+tra_left[2]*k_
    th_left = math.atan(abs(tra_left[0]/tra_left[2]))
    rot_left = math.cos(-(pi2-th_left)/2)+math.sin(-(pi2-th_left)/2)*j_
    left_pose = rot_left+E_*0.5*tra_left_dq*rot_left

    tra_right = transform_to_image_plane(
        [im_width, im_height/2], im_width, im_height, cam_fov)
    tra_right_dq = tra_right[0]*i_+tra_right[1]*j_+tra_right[2]*k_
    th_right = math.atan(abs(tra_right[0]/tra_right[2]))
    rot_right = math.cos((pi2-th_right)/2)+math.sin((pi2-th_right)/2)*j_
    right_pose = rot_right+E_*0.5*tra_right_dq*rot_right

    tra_up = transform_to_image_plane(
        [im_width/2, 0], im_width, im_height, cam_fov)
    tra_up_dq = tra_up[0]*i_+tra_up[1]*j_+tra_up[2]*k_
    th_up = math.atan(abs(tra_up[1]/tra_up[2]))
    rot_up = math.cos((pi2-th_up)/2)+math.sin((pi2-th_up)/2)*i_
    up_pose = rot_up+E_*0.5*tra_up_dq*rot_up

    tra_down = transform_to_image_plane(
        [im_width/2, im_height], im_width, im_height, cam_fov)
    tra_down_dq = tra_down[0]*i_+tra_down[1]*j_+tra_down[2]*k_
    th_down = math.atan(abs(tra_down[1]/tra_down[2]))
    rot_down = math.cos(-(pi2-th_down)/2)+math.sin(-(pi2-th_down)/2)*i_
    down_pose = rot_down+E_*0.5*tra_down_dq*rot_down

    return (left_pose, right_pose, up_pose, down_pose)


def calculate_plane_jacobian(robot_kin, joint_vals, plane_pose):

    plane_pose_wrt_world = robot_kin.fkm(joint_vals)*plane_pose
    plane_pose_jacob = haminus8(plane_pose)@robot_kin.pose_jacobian(joint_vals)
    plane_jacob = robot_kin.plane_jacobian(
        plane_pose_jacob, plane_pose_wrt_world, k_)

    return (plane_pose_wrt_world, plane_jacob)
