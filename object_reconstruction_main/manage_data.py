
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
import os
import json


class ManageData:
    """Class to manage object data for reconstruction tasks.
    This class handles loading object names, paths, ground truth data, and candidate views.
    It also provides methods to save results and manipulate the object list."""

    def __init__(self):
        # TODO: get the paths from a config file
        self.data_root_path = "/media/fth/T9/ResultsWithPyRep"
        self.mesh_path = "/media/fth/T9/UpdatedCodeData/MeshData"
        self.gt_path = self.data_root_path+"/GtPclData"
        self.object_list_txt = self.gt_path+"/used_objects.txt"
        self.result_save_path = self.data_root_path + "/Results_Parallel"
        self.candidate_views_txt = self.data_root_path + \
            "/ViewSpace/sample_space_up_down.txt"
        self.object_name_list = []  # Stores object names
        self.object_mesh_path_list = []  # Stores full paths to object meshes
        self.object_gt_path_list = []  # Stores full paths to object ground truth pcl data
        self.load_object_list()

    def load_object_list(self):
        # load object names from file and populate both name and full-path lists
        self.object_name_list = []
        self.object_mesh_path_list = []
        self.object_gt_path_list = []
        try:
            with open(self.object_list_txt, "r") as f:
                lines = f.readlines()
                for idx, line in enumerate(lines):
                    obj = line.strip()
                    if obj == "":
                        continue
                    self.object_name_list.append(obj)
                    self.object_mesh_path_list.append(
                        self.mesh_path+"/"+obj)
                    self.object_gt_path_list.append(self.gt_path+"/"+obj)

        except FileNotFoundError:
            # file may not exist yet; start with empty lists
            self.object_name_list = []
            self.object_mesh_path_list = []
            self.object_gt_path_list = []

    def get_candidate_views(self):
        """Return candidate views as an Nx7 numpy array."""

        return np.loadtxt(self.candidate_views_txt)

    def get_object_name_list(self):
        """Return (names_list, full_path_list).

        Returns copies to avoid accidental external modification.
        """
        return list(self.object_name_list)

    def get_gt_data(self, index):
        """Return the ground truth point cloud data for the object at `index`."""
        # Set the reconstruction object pose
        gt_pose = self.object_gt_path_list[index]+"/gt_pose.npy"
        gt_pcl = self.object_gt_path_list[index]+"/gt.npy"
        gt_raw = np.load(gt_pcl)
        gt_filtered = gt_raw[gt_raw[:, 2] > 0.01]
        return (np.load(gt_pose), gt_filtered)

    def get_mesh_path(self, index):
        """Return the mesh path for the object at `index`."""
        return self.object_mesh_path_list[index]

    # --- Array-like manipulation methods ---
    def set_object_list(self, names):
        """Replace the current object list with `names` (iterable of names).

        This updates both the name list and the full-path list.
        """
        self.object_name_list = [str(n) for n in names]
        self.object_full_path_list = [
            self.mesh_path+"/"+n for n in self.object_name_list]

    def append_object(self, name):
        """Append `name` to the object list. If `save` is True, persist to `object_list_txt`."""
        name = str(name)
        if name in self.object_name_list:
            return False
        self.object_name_list.append(name)
        self.object_full_path_list.append(self.mesh_path+"/"+name)
        return True

    def insert_object(self, index, name):
        name = str(name)
        self.object_name_list.insert(index, name)
        self.object_full_path_list.insert(index, self.mesh_path+"/"+name)

    def remove_object_at(self, index):
        """Remove object at `index`. Returns the removed name."""
        name = self.object_name_list.pop(index)
        self.object_full_path_list.pop(index)
        return name

    def remove_object(self, name, save=False):
        """Remove first occurrence of `name`. Returns True if removed."""
        if name in self.object_name_list:
            idx = self.object_name_list.index(name)
            self.remove_object_at(idx)
            return True
        return False

    def clear_object_list(self):
        self.object_name_list = []
        self.object_full_path_list = []

    def get_object_list(self, index):
        """Return (name, full_path) for object at `index`."""
        return self.object_name_list[index], self.object_full_path_list[index]

    def length_object_list(self):
        return len(self.object_name_list)

    def save_nbv_result(self, object_index, nbv_index, result_data, method):
        """Save next best view (NBV) result for the object at `object_index`."""
        obj_name = self.object_name_list[object_index]
        nbv_dir = join(self.result_save_path, obj_name, method)

        os.makedirs(nbv_dir, exist_ok=True)

        result_file = join(nbv_dir, f"nbv_{nbv_index}.json")

        # Convert the result into JSON-serializable types.
        def _make_serializable(o):
            # Enums -> name
            try:
                import enum as _enum
            except Exception:
                _enum = None

            if _enum is not None and isinstance(o, _enum.Enum):
                return o.name
            if isinstance(o, np.ndarray):
                return o.tolist()
            if isinstance(o, dict):
                return {str(k): _make_serializable(v) for k, v in o.items()}
            if isinstance(o, (list, tuple)):
                return [_make_serializable(v) for v in o]
            # common numeric and string types are serializable
            if isinstance(o, (str, int, float, bool)) or o is None:
                return o
            # Fallback: try to convert to string
            try:
                return str(o)
            except Exception:
                return None

        serializable_data = _make_serializable(result_data)

        with open(result_file, 'w') as f:
            json.dump(serializable_data, f, indent=2)

    def save_partial_model_data(self, object_index,  nbv_index, partial_pcl, method):
        """Save the partial model data for the object, method and nbv_index to results folder."""
        result_path = join(self.result_save_path,
                           self.object_name_list[object_index],  method, "pcl_data")

        os.makedirs(result_path, exist_ok=True)
        np.save(result_path+f"/partial_model_{nbv_index}.npy", partial_pcl)
