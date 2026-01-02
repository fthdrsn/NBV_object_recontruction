from math import pi, sin, cos
from pyrep.robots.arms.youBot import youBot
from pyrep.robots.mobiles.youbot import YouBot
import pyrep.objects as PyRepObj
from dqrobotics import *
from dqrobotics.robot_modeling import DQ_SerialManipulatorDH, DQ_HolonomicBase, DQ_SerialWholeBody
from dqrobotics.interfaces.vrep.robots import *
import math
import numpy as np
from RobotModel.dq_pyrep import BaseCommunication
from typing import List


class YouBotModel(BaseCommunication):
    """ YouBotModel class is used to interact with the YouBot robot in CoppeliaSim.
    It provides methods to get and set the robot's pose and joint positions, as well as to control the robot in velocity mode.
    It uses the dqrobotics library to create a kinematic model of the YouBot mobile manipulator.
    The YouBot consists of a holonomic base and a 5-DOF arm.
    The robot can be controlled in position or velocity mode.
    The robot's base is modeled as a holonomic base, and the arm is modeled as a serial manipulator with DH parameters.
    The robot's kinematic model is created using the dqrobotics library, which allows for easy manipulation of the robot's pose and joint positions.
    The robot can be controlled in velocity mode by setting the wheel velocities of the holonomic base and the joint velocities of the arm.
    """

    def __init__(self, robot_dim: float):

        super(YouBotModel, self).__init__()
        self.robot_radius = robot_dim/2

        # Ref dummy frame attached to the robot base(x: forward, z:up)
        # Reference frame of the YouBot base in CoppeliaSim
        self.youbot_base_ref = PyRepObj.Object.get_object("dqRef")
        self.youbot_base_pyrep = YouBot()  # Pyrep youbot base model
        self.youbot_arm_pyrep = youBot()   # Pyrep youbot arm model

        is_camera_active = True  # If true, add camera transformation to kinematic chain

        # DH Parameters of the Youbot arm
        pi2 = math.pi / 2
        self.dh_mat = np.array([[0,      pi2,       0,      pi2,    0],
                               [0.147,    0,       0,        0,    0.218],
                               [0.033,    0.155,   0.135,    0,    0],
                               [pi2,      0,       0,      pi2,    0],
                               [0, 0, 0, 0, 0]])

        # Create the kinematic model of mobile maniopulator
        arm = DQ_SerialManipulatorDH(self.dh_mat)
        base = DQ_HolonomicBase()
        x_bm = 1 + E_ * 0.5 * (0.156 * i_ + 0.085 * k_)
        base.set_frame_displacement(x_bm)
        kin = DQ_SerialWholeBody(base)
        if is_camera_active:  # If camera is active, add camera transformation to the kinematic chain
            rot = math.cos(-pi2/2)+math.sin(-pi2/2)*k_
            arm.set_effector(rot)
        kin.add(arm)
        self.youbot_kinematic = kin

    def get_q_from_sim(self):
        """ Get the pose of the holonomic base [x,y,phi] using dq reference frame(self.youbot_base_ref )
        and joint positions of the arm.
        :return: base pose and arm joint position as list [x,y,phi,th0,th1,th2,th3,th4]
        """

        return self.get_q_from_sim_vel()

    def get_q_from_sim_vel(self) -> List:
        """ Get the pose of the holonomic base [x,y,phi] using dq reference frame(self.youbot_base_ref )
        and joint positions of the arm.

        :return: base pose and arm joint position as list [x,y,phi,th0,th1,th2,th3,th4]
        """

        base_position = self.youbot_base_ref.get_position()
        base_phi = self.youbot_base_ref.get_orientation()[-1]
        # Get the joint values of arm directly
        joint_angles = self.youbot_arm_pyrep.get_joint_positions()
        return [base_position[0], base_position[1], base_phi, *joint_angles]

    @property
    def Kinematics(self):
        """ Get the kinematic model of the YouBot mobile manipulator.
        :return: kinematic model of the YouBot mobile manipulator
        """
        return self.youbot_kinematic

    @property
    def BaseKinematics(self):
        """ Get the kinematic model of the holonomic base.
        :return: kinematic model of the holonomic base
        """
        return self.youbot_kinematic.get_chain_as_holonomic_base(0)

    @property
    def ArmKinematics(self):
        """ Get the kinematic model of the arm.
        :return: kinematic model of the arm
        """
        return self.youbot_kinematic.get_chain_as_serial_manipulator_dh(1)

    def send_velocities(self, velocities: List) -> None:
        """ Set wheel velocities of the holonomic base and joint velocities of the arm.
        :param velocities: desired base and arm velocities as list [x_dot,y_dot,phi_dot,th0_dot,
                                                                    th1_dot,th2_dot,th3_dot,th4_dot]
        """

        forwBackVel = velocities[0]
        leftRightVel = -velocities[1]
        rotVel = -velocities[2]
        lx = 0.5*3.5707e-01  # Distance from wheel center to x axis from body fram in m
        ly = 0.5*5.7010e-01  # Distance from wheel center to y axis from body frame in m
        r = 0.5*9.9968e-02  # Radius of the wheel in m
        Wheel1 = (-forwBackVel-leftRightVel-(lx+ly)*rotVel)/r
        Wheel2 = (-forwBackVel+leftRightVel-(lx+ly)*rotVel)/r
        Wheel3 = (-forwBackVel-leftRightVel+(lx+ly)*rotVel)/r
        Wheel4 = (-forwBackVel+leftRightVel+(lx+ly)*rotVel)/r
        self.youbot_base_pyrep.set_joint_target_velocities([Wheel1,
                                                            Wheel2,
                                                            Wheel3,
                                                            Wheel4])
        arm_vel = velocities[3:]
        self.youbot_arm_pyrep.set_joint_target_velocities(arm_vel)
