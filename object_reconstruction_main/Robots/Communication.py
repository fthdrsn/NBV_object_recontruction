
""" 
This class deals with the convention between dqrobotics 
library and pyrep library for retrieving and sending data from/to coppeliaSim.

Note that:
In documentation, for the sake of clarity the term DQ form is used to refer to representation in dqrobotics library.
DQ Form refers to : 
x*i_+y*j_+z*k_  for position or translation
w+x*i_+y*j_+z*k_  for rotation or orientation

"""
import numpy as np
from dqrobotics import *
import pyrep.objects as PyRepObj
from collections import namedtuple
import time
from typing import Dict
objects = namedtuple("objectPoseInfo", "name pose type radius")


class BaseCommunication():
    def __init__(self):
        pass

    def get_object_rotation(self, obj_handle: int, relative_to: int = None) -> DQ:
        """ Get the quaternion from simulation as [x,y,z,w], and return orietation in DQ Form

        :param obj_handle: Object reference in the simulation
        :param relative_to: Orienation is calculated w.r.t this frame in simulation
        :return: quaternion in DQ Form
        """
        rp = obj_handle.get_quaternion(
            relative_to)  # get the rotation as quaternion

        # create dq rotation quaternion [w,x,y,z]
        r = DQ([rp[3], rp[0], rp[1], rp[2], 0, 0, 0, 0])
        return normalize(r)

    def set_object_rotation(self, obj_handle: int, rot: DQ) -> None:
        """ Set orientation of a simulation object

        :param obj_handle: Object reference in the simulation
        :param rot: Quaternion in DQ Form

        """
        r = vec4(rot)
        rp = [r[1], r[2], r[3], r[0]]  # [w,x,y,z] ->[x,y,z,w]
        obj_handle.set_quaternion(rp)

    def get_object_translation(self, obj_handle: int, relative_to: int = None) -> DQ:
        """ Get the position from simulation and return  position in DQ Form

        :param obj_handle: Object reference in the simulation
        :param relative_to: Position is calculated w.r.t this frame in simulation
        :return: translation in DQ Form
        """
        tp = obj_handle.get_position(relative_to)
        t = DQ([0, tp[0], tp[1], tp[2]])
        return t

    def set_object_translation(self, obj_handle: int, tra: DQ) -> None:
        """ Set position of an object in simulation

        :param obj_handle: Object reference in the simulation
        :param tra: translation in DQ Form

        """
        t = vec4(tra)[1:]
        obj_handle.set_position(list(t))

    def get_object_pose(self, obj_handle: int, relative_to: int = None) -> DQ:
        """ Get the pose from simulation and return pose in DQ Form

        :param obj_handle: Object reference in the simulation
        :param relative_to: Pose is calculated w.r.t this frame in simulation
        :return: pose in DQ Form
        """
        t = self.get_object_translation(obj_handle, relative_to)
        r = self.get_object_rotation(obj_handle, relative_to)
        h = r + 0.5 * E_ * t * r
        return h

    def set_object_pose(self, obj_handle: int, h: DQ) -> None:
        """ Get pose in DQ Form and set the object`s pose in simulation 

          :param obj_handle: Object reference in the simulation
          :param h: Pose in DQ Form

        """
        self.set_object_translation(obj_handle, translation(h))
        self.set_object_rotation(obj_handle, rotation(h))

    def get_plane_from_sim(self, plane_name: str, normal: DQ) -> DQ:
        """ Get the plane form simulation in DQ Form using the plane`s name and normal vector

        :param plane_name: Plane name in the simulation
        :param normal: Normal vector in DQ Form
        :return: Plane representation in DQ Form

        """
        plane = PyRepObj.Object.get_object(plane_name)
        plane_object_pose = self.get_object_pose(plane)
        p = translation(plane_object_pose)
        r = rotation(plane_object_pose)
        n = Ad(r, normal)
        d = dot(p, n)
        return n + E_ * d

    def get_line_from_sim(self, line_name: str, direction: DQ) -> DQ:
        """ Get a line form simulation in DQ Form using the line`s name and direction vector

        :param line_name: Line name in the simulation
        :param direciton: Direction vector in DQ Form
        :return: Line representation in DQ Form

        """
        line = PyRepObj.Object.get_object(line_name)
        line_object_pose = self.get_object_pose(line)
        p = translation(line_object_pose)
        r = rotation(line_object_pose)
        l = Ad(r, direction)
        m = cross(p, l)
        return l + E_ * m

    def generate_seq_of_objects(self, obj_dict: Dict):
        """ Gets sequence of shapes from simulation(objects could be cylinder or plane)

        :param obj_dict: Python dictiory storing object names, types and radius 
        :return: Namedtuple representing objects in DQ Form
        """
        shape_dq_list = []
        for obj in obj_dict["planes"]:
            shape_dq_list.append(
                objects(obj, self.get_plane_from_sim(obj, k_), "plane", " "))
        for obj in obj_dict["cylinders"]:
            shape_dq_list.append(
                objects(obj[0], self.get_plane_line_sim(obj[0], k_), "cylinder", obj[1]))
        return shape_dq_list
