from scipy.spatial import cKDTree
from dqrobotics import *
import numpy as np
from scipy.spatial.transform import Rotation as R
from pyquaternion import Quaternion
from dqrobotics.utils import DQ_Geometry
import math
pi2 = math.pi/2


def compute_base_constraints(pose_list, radius_list, robot_kin, youbot_q):
    """Compute the base constraints for the YouBot robot using point-to-line distance.
    Args:
        pose_list: List of poses of the obstacles as dual quaternions.
        radius_list: List of radii of the obstacles.
        robot_kin: The robot kinematic model.
        youbot_q: Joint values of the YouBot robot.
    Returns:
        A tuple containing the Jacobian of the base constraints and the corresponding b vector.
    """
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


def compute_base_constraints_p2p(pose_list, radius_list, robot_kin, youbot_q):
    """Compute the base constraints for the YouBot robot using point-to-point distance.
    Args:
        pose_list: List of poses of the obstacles as dual quaternions.
        radius_list: List of radii of the obstacles.
        robot_kin: The robot kinematic model.
        youbot_q: Joint values of the YouBot robot.
    Returns:
        A tuple containing the Jacobian of the base constraints and the corresponding b vector.
    """

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

        j_dist = robot_kin.point_to_point_distance_jacobian(
            Jt, t_inner, translation(pose_))
        dist = DQ_Geometry.point_to_point_squared_distance(
            t_inner, translation(pose_)) - (rad_ + 0.35) ** 2

        if idx == 0:
            j_constraint = np.array(j_dist)
        else:
            j_constraint = np.concatenate((j_constraint, j_dist), axis=0)
        b_constraint.append(eta*dist)
        idx += 1

    return j_constraint, np.array(b_constraint)


def serialize_obs_dic(obs_dic):
    """
    Convert obs_dic to a JSON-serializable dictionary.
    obs_dic is assumed to have keys that are strings and values that may contain numpy arrays or custom objects.
    """
    serialized_dic = {}
    for key, value in obs_dic.items():
        serialized_value = {}

        for inner_key, inner_value in value.items():
            if inner_key == "idx" and isinstance(inner_value, tuple):
                inner_value = inner_value[0]
            if inner_key is not "psn_workspace_dq":
                if isinstance(inner_value, np.ndarray):
                    # Convert numpy arrays to lists
                    serialized_value[inner_key] = inner_value.tolist()
                elif isinstance(inner_value, list):
                    # Convert any numpy array in the list to a list
                    serialized_value[inner_key] = [
                        item.tolist() if isinstance(item, np.ndarray) else item for item in inner_value
                    ]
                else:
                    serialized_value[inner_key] = inner_value

        serialized_dic[key] = serialized_value
    return serialized_dic


def compute_eef_constraints(pose_list, radius_list, robot_kin, youbot_q):
    """Compute the end-effector constraints for the YouBot robot.
    Args:
        pose_list: List of poses of the obstacles as dual quaternions.
        radius_list: List of radii of the obstacles.
        robot_kin: The robot kinematic model.
        youbot_q: Joint values of the YouBot robot.
    Returns:
        A tuple containing the Jacobian of the end-effector constraints and the corresponding b vector.
    """

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
    """Convert a pose to a line represented as a dual quaternion.
    Args:
        line_pose: A dual quaternion representing the pose of the line.
        direction: A 3D direction vector as a numpy array or list.
    Returns:
        A dual quaternion representing the line.
    """
    p = translation(line_pose)
    r = rotation(line_pose)
    l = Ad(r, direction)
    m = cross(p, l)
    return l + E_ * m


def get_direction(line_pose, direction):
    """Extract the direction vector from a pose represented as a dual quaternion.
    Args:
        line_pose: A dual quaternion representing the pose of the line.
        direction: A 3D direction vector as a numpy array or list.
    Returns:
        A dual quaternion representing the direction of the line.
    """
    p = translation(line_pose)
    r = rotation(line_pose)
    l = Ad(r, direction)

    return l


def pose_to_plane(plane_pose, normal):
    """Convert a pose to a plane represented as a dual quaternion.
    Args:
        plane_pose: A dual quaternion representing the pose of the plane.
        normal: A 3D normal vector as a numpy array or list.
    Returns:
        A dual quaternion representing the plane.
    """
    p = translation(plane_pose)
    r = rotation(plane_pose)
    n = Ad(r, normal)
    d = dot(p, n)
    return n + E_ * d


def direction_to_orientation(direction):
    """Convert a direction vector to a pose represented as a dual quaternion.
    Args:
        direction: A 3D direction vector as a numpy array or list.
    Returns:
        A dual quaternion representing the pose with the direction as the z-axis.
    """

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
    """Generate a point on a cylinder defined by its centre, radius, and height.
    Args:
        theta: Angle in radians to define the point's position on the cylinder.
        cly_centre: Centre of the cylinder as a tuple (x, y, z).
        z: Height of the cylinder.
        r: Radius of the cylinder.
    Returns:
        A point on the cylinder as a dual quaternion.
    """
    x = r*math.cos(theta)
    y = r*math.sin(theta)

    pose_point = (1+E_*0.5*(x*i_+y*j_+z*k_)) * \
        (1+E_*0.5*(cly_centre[0]*i_+cly_centre[1]*j_+cly_centre[2]*k_))

    return pose_point


def transform_to_image_plane(pnt, im_width=256, im_height=256, cam_fov=57):
    """Transform a point in the image plane to the camera's field of view.
    Args:
        pnt: A 2D point in the image plane (x, y).
        im_width: Width of the image in pixels.
        im_height: Height of the image in pixels.
        cam_fov: Camera field of view in degrees.
    Returns:
        A 3D point in the camera's field of view.
    """
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


def get_camera_fov_planes(camera_params):
    """Calculate the poses of the four planes (left, right, up, down) in the camera's field of view.
    Args:
        im_width: Width of the image in pixels.
        im_height: Height of the image in pixels.
        cam_fov: Camera field of view in degrees.
    Returns:
        A tuple containing the poses of the left, right, up, and down planes as dual quaternions.
    """

    im_width = camera_params["imWidth"]
    im_height = camera_params["imHeight"]
    cam_fov = camera_params["camFov"]

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
    """Calculate the Jacobian of a plane defined by its pose with respect to the robot's end-effector.
    Args:
        robot_kin: The robot kinematic model.
        joint_vals: The joint values of the robot.
        plane_pose: The pose of the plane as a dual quaternion.
    Returns:
        A tuple containing the pose of the plane with respect to the world frame and the Jacobian of the plane.
    """

    plane_pose_wrt_world = robot_kin.fkm(joint_vals)*plane_pose
    plane_pose_jacob = haminus8(plane_pose)@robot_kin.pose_jacobian(joint_vals)
    plane_jacob = robot_kin.plane_jacobian(
        plane_pose_jacob, plane_pose_wrt_world, k_)

    return (plane_pose_wrt_world, plane_jacob)


def cross_product_2d(v1, v2):
    """Compute the 2D cross product of two vectors.
    v1 and v2 are both 2D vectors represented as tuples (x, y) or numpy arrays."""
    return v1[0] * v2[1] - v1[1] * v2[0]


def compute_clossest_obs_wb(obs_pose_list, obs_rad_list, q_):
    """Compute the closest obstacle to the robot base pose q_.
    Returns the indices of the obstacles sorted by distance."""

    dist_list = []
    for pos_, rad_ in zip(obs_pose_list, obs_rad_list):
        obs_position = vec3(translation(pos_))
        obs_x = obs_position[0]
        obs_y = obs_position[1]
        dist_ = (math.sqrt((q_[0]-obs_x)**2 +
                           (q_[1]-obs_y)**2)-0.35-rad_)
        dist_list.append(dist_)
    all_obs_idxs = np.argsort(np.array(dist_list))
    return all_obs_idxs, dist_list


def edge_distance(cyl1, cyl2):
    """Compute the edge distance between two cylinders."""
    x1, y1, r1 = cyl1
    x2, y2, r2 = cyl2
    center_distance = np.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    edge_dist = center_distance - (r1 + r2)
    return max(0, edge_dist)


def compute_distance_matrix(cylinders):
    """Compute the distance matrix for a list of cylinders."""
    num_cylinders = len(cylinders)
    dist_matrix = np.zeros((num_cylinders, num_cylinders))

    for i in range(num_cylinders):
        for j in range(i + 1, num_cylinders):
            dist = edge_distance(cylinders[i], cylinders[j])
            dist_matrix[i, j] = dist
            dist_matrix[j, i] = dist

    return dist_matrix


def softmin(distances,  h=0.03, delta=0.1):
    """
    Compute the softmin of a list of distances.

    Parameters:
        distances (list or numpy array): List of distances [F1, F2, ..., Fm].
        h (float): Smoothing parameter (smaller h makes softmin closer to min).

    Returns:
        float: Softmin value.
    """
    # Avoid numerical instability by subtracting the minimum distance
    min_dist = np.min(distances)
    exp_terms = np.exp(-(distances - min_dist) / h)
    softmin_value = -h * np.log(np.mean(exp_terms)) + min_dist - delta
    return softmin_value


def softmin_gradient(distances, gradients, h=0.03):
    """
    Compute the gradient of the softmin function.

    Parameters:
        distances (list or numpy array): List of distances [F1, F2, ..., Fm].
        gradients (list of numpy arrays): List of gradients [∇F1, ∇F2, ..., ∇Fm].
        h (float): Smoothing parameter (smaller h makes softmin closer to min).

    Returns:
        numpy array: Gradient of the softmin function.
    """
    # Avoid numerical instability by subtracting the minimum distance
    min_dist = np.min(distances)
    exp_terms = np.exp(-(distances - min_dist) / h)
    weights = exp_terms / np.sum(exp_terms)  # Normalize to get weights

    # Compute the weighted average of the gradients
    softmin_grad = np.zeros_like(gradients[0])  # Initialize gradient
    for i in range(len(gradients)):
        softmin_grad += weights[i] * gradients[i]

    return softmin_grad


def pcl_coverage(source_pts, target_pts, threshold=0.008):
    """
    Compute coverage ration what ratio of source points are within 
    threshold distance of the nearest target point.

    :param source_pts: Points to evaluate coverage for (Nx3)
    :param target_pts: Reference point set (Mx3)
    :param threshold: Distance threshold in meters
    :return: idexes of covered source points, number of covered points
    """
    A = np.asarray(source_pts)
    B = np.asarray(target_pts)

    treeB = cKDTree(B)
    distances, _ = treeB.query(A, k=1)  # nearest distance for each point in A

    # Count points within threshold
    covered_idxes = distances <= threshold
    num_of_covered = np.sum(covered_idxes)

    return covered_idxes, num_of_covered


def pcl_chamfer_distance(a_pts, b_pts):
    """
    Bidirectional Chamfer (mean nearest neighbour).
    returns (cd_ab, cd_ba, cd_mean)
    """
    A = np.asarray(a_pts)
    B = np.asarray(b_pts)
    treeB = cKDTree(B)          # KD-tree for point set B
    treeA = cKDTree(A)          # KD-tree for point set A

    # For each point in A, find distance to nearest point in B
    d_ab, _ = treeB.query(A, k=1)   # shape (N,)

    # For each point in B, find distance to nearest point in A
    d_ba, _ = treeA.query(B, k=1)   # shape (M,)

    # Return three variants:
    return d_ab.mean(), d_ba.mean(), 0.5*(d_ab.mean()+d_ba.mean())


def calculate_area_under_curve(x, y, x_max, y_max):
    """
    Calculate the area under the curve using the trapezoidal rule.

    Parameters:
        x (list or numpy array): x-coordinates of the points.
        y (list or numpy array): y-coordinates of the points.

    Returns:
        float: Area under the curve.
    """

    x_normalized = np.array(x) / x_max
    y_normalized = np.array(y) / y_max

    auc = np.trapz(y_normalized, x_normalized)
    return auc
