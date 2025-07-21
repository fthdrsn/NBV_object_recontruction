from Robots.RobotsInteraction import *
from dqrobotics import *
import numpy as np
from scipy.spatial.transform import Rotation as R
from pyquaternion import Quaternion
from dqrobotics.utils import DQ_Geometry
import random
pi2 = math.pi/2


class RandomEnv:
    """
    This class generates a random environment with obstacles for the robot to navigate.
    It creates obstacles within specified limits, ensuring they are not too close to the robot or the target position.
    The obstacles are generated with random positions and radii, and the environment can be regenerated multiple times.
    """

    def __init__(self, params) -> None:
        self.params = params  # Store the parameters for the environment
        # Limits for obstacle x-coordinates
        self.obs_x_lims = params["obs_x_lims"]
        # Limits for obstacle y-coordinates
        self.obs_y_lims = params["obs_y_lims"]
        # Minimum distance between obstacles
        self.min_dist_between_obs = params["min_dist_between_obs"]
        # Number of obstacles to generate
        self.num_obstacle = params["num_obstacle"]
        # Range of radii for obstacles
        self.obstacle_rads = params["obstacle_rads_range"]
        # Position of the robot in the environment
        self.robot_position = params["robot_position"]
        # Position of the search cylinder in the environment
        self.search_cylinder_position = params["cly_position"]
        # Radius of the search cylinder
        self.search_cylinder_rad = params["cly_rad"]
        # Safe radius around the robot to avoid obstacles
        self.robot_safe_rad = params["robot_safe_rad"]
        self.robot_rad = 0.35  # Radius of the robot

    def random_obstacles(self):
        """
        Generates random obstacles within the specified limits and conditions.
        Ensures that obstacles are not too close to the robot or the target position,
        and that they maintain a minimum distance from each other.
        """

        [x_min, x_max, y_min, y_max] = self.obs_x_lims+self.obs_y_lims
        obs_idx = 0
        self.obs_position_list = []
        self.obs_pose_list_dq = []
        self.obs_rad_list = []
        while obs_idx < self.num_obstacle:
            while True:
                # Randomly select an obstacle radius
                obs_rad = random.uniform(*self.obstacle_rads)
                # Randomly select an x-coordinate for the obstacle
                obs_x = random.uniform(x_min+obs_rad, x_max-obs_rad)
                # Randomly select an y-coordinate for the obstacle
                obs_y = random.uniform(y_min+obs_rad, y_max-obs_rad)

                # Convert to numpy array for distance calculations
                obs_position_np = np.array([obs_x, obs_y])
                is_valid_obs = True

                # Check if the obstacle is too close to the robot or the search cylinder
                if np.linalg.norm(obs_position_np-self.robot_position) > self.robot_safe_rad and np.linalg.norm(obs_position_np-self.search_cylinder_position) > (self.search_cylinder_rad+obs_rad):

                    # Check if the obstacle is too close to any existing obstacles
                    for pos_, rad_ in zip(self.obs_position_list, self.obs_rad_list):
                        if (math.sqrt((pos_[0]-obs_x)**2+(pos_[1]-obs_y)**2)-rad_-obs_rad) < self.min_dist_between_obs:
                            is_valid_obs = False
                            break
                else:
                    is_valid_obs = False

                if is_valid_obs:
                    # Create a DQ pose for the obstacle
                    obs_pose = 1+0.5*E_*(obs_x*i_+obs_y*j_+0.11*k_)
                    self.obs_position_list.append([obs_x, obs_y])
                    self.obs_pose_list_dq.append(
                        obs_pose)
                    self.obs_rad_list.append(obs_rad)
                    obs_idx += 1
                    break

    def generate_random_env(self):
        """
        Generates a random environment with obstacles.
        This method can be called multiple times to regenerate the environment.
        """
        self.random_obstacles()
        return [self.obs_pose_list_dq, self.obs_rad_list]
