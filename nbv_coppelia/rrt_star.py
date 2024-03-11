"""
This code was taken and adapted from:

https://github.com/ethz-asl/cvae_exploration_planning
https://arxiv.org/abs/2202.13715
"""

"""BSD 3-Clause License

Copyright (c) 2022, ETHZ ASL
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."""




import math
import random
import numpy as np
class RRT:
    """
    Class for RRT planning
    """

    class Node:
        """
        RRT Node
        """

        def __init__(self, x, y, z):
            self.x = x
            self.y = y
            self.z = z
            self.path_x = []
            self.path_y = []
            self.path_z = []
            self.parent = None

    def __init__(self,
                 robot_map,
                 start,
                 goal,
                 expand_dis=30.0,
                 path_resolution=1.0,
                 goal_sample_rate=20,
                 max_iter=300):
        """
        Setting Parameter
        start:Start Position [x,y]
        goal:Goal Position [x,y]
        randArea:Random Sampling Area [min,max]
        """
        robot_map = np.zeros((50, 50))
        self.start = self.Node(start[0], start[1], start[2])
        self.end = self.Node(goal[0], goal[1], goal[2])
        self.map_bounds = [np.shape(robot_map)[0] - 1,
                           np.shape(robot_map)[1] - 1]
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.map = robot_map
        self.min_height = 0.3
        self.max_height = 0.5
        self.min_rad = 2.75
        self.max_rad = 4
        self.min_angle = 0.01
        self.min_node_dist = 0.3

    def plan(self):
        """
        rrt path planning
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()

            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)

            if self.check_collision(new_node, self.map):
                self.node_list.append(new_node)

            if self.calc_dist_to_goal(self.node_list[-1].x,
                                      self.node_list[-1].y, self.node_list[-1].y) <= self.expand_dis:
                final_node = self.steer(self.node_list[-1], self.end,
                                        self.expand_dis)
                if self.check_collision(final_node, self.map):
                    return self.generate_final_course(len(self.node_list) - 1)

        return None  # cannot find path

    def steer(self, from_node, to_node, extend_length=float("inf")):

        new_node = self.Node(from_node.x, from_node.y, from_node.z)
        d = self.calc_distance(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]
        new_node.path_z = [new_node.z]

        if d < self.min_node_dist:
            d = self.min_node_dist

        if extend_length > d:
            extend_length = d

        from_arr = np.array([from_node.x, from_node.y, from_node.z])
        to_arr = np.array([to_node.x, to_node.y, to_node.z])
        dir_vec = (to_arr-from_arr)/np.linalg.norm(to_arr-from_arr)

        new_vec = from_arr+extend_length*dir_vec
        new_node.x = new_vec[0]
        new_node.y = new_vec[1]
        new_node.z = new_vec[2]
        # for _ in range(n_expand):
        #     new_node.x += self.path_resolution * math.cos(theta)
        #     new_node.y += self.path_resolution * math.sin(theta)
        #     new_node.path_x.append(new_node.x)
        #     new_node.path_y.append(new_node.y)

        # d, _ = self.calc_distance_and_angle(new_node, to_node)
        # if d <= self.path_resolution:
        #     new_node.path_x.append(to_node.x)
        #     new_node.path_y.append(to_node.y)
        #     new_node.x = to_node.x
        #     new_node.y = to_node.y

        new_node.parent = from_node

        return new_node

    def generate_final_course(self, goal_ind):
        path = [[self.end.x, self.end.y, self.end.z]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y, node.z])
            node = node.parent
        path.append([node.x, node.y, node.z])

        return path

    def calc_dist_to_goal(self, x, y, z):
        dx = x - self.end.x
        dy = y - self.end.y
        dz = z - self.end.z
        return dx**2 + dy**2+dz**2

    # Get random sample in limited region
    def get_random_node(self):
        start_vec = np.array([self.start.x, self.start.y, 0])
        goal_vec = np.array([self.end.x, self.end.y, 0])
        theta_signed = self.signed_angle_between_vectors(start_vec, goal_vec)

        current_line_dir = start_vec/np.linalg.norm(start_vec)
        rnd_angle = random.uniform(self.min_angle, 0.95*abs(theta_signed))
        rnd_z = random.uniform(self.min_height, self.max_height)
        rnd_rad = random.uniform(self.min_rad, self.max_rad)

        current_positon = rnd_rad*current_line_dir
        rot_mat = np.array([[math.cos(rnd_angle), -math.sin(rnd_angle)],
                            [math.sin(rnd_angle), math.cos(rnd_angle)]])
        if theta_signed < 0:
            rotated_position = rot_mat.T@current_positon[:2]
        else:
            rotated_position = rot_mat@current_positon[:2]
        rnd = self.Node(rotated_position[0], rotated_position[1], rnd_z)
        return rnd

    # def get_random_node(self):
    #     if random.randint(0, 100) > self.goal_sample_rate:
    #         rnd = self.Node(
    #             random.uniform(0, self.map_bounds[0]),
    #             random.uniform(0, self.map_bounds[1]))
    #     else:  # goal point sampling
    #         rnd = self.Node(self.end.x, self.end.y)
    #     return rnd

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y) ** 2+(node.z - rnd_node.z) ** 2
                 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind

    def check_collision(self, node, robot_map):

        if node is None:
            return False

        d = node.x**2+node.y**2+node.z**2
        d = np.sqrt(d)
        if d < self.min_rad or d > self.max_rad:
            return False
        if node.z < self.min_height or node.z > self.max_height:
            return False

        return True  # safe

    # @staticmethod
    # def check_collision(node, robot_map):

    #     # if node is None:
    #     #     return False

    #     # for i in range(len(node.path_x)):
    #     #     if robot_map[int(node.path_x[i]), int(node.path_y[i])] != 0:
    #     #         return False  # Unknown or Occupied map

    #     return True  # safe

    @staticmethod
    def calc_distance(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        dz = to_node.z - from_node.z
        d = dx**2+dy**2+dz**2
        return np.sqrt(d)

    @staticmethod
    def signed_angle_between_vectors(v1, v2):
        axis = np.array([0, 0, 1])
        dot_product = np.dot(v1, v2)
        cross_product = np.cross(v1, v2)
        angle = np.arctan2(np.linalg.norm(cross_product), dot_product)
        signed_angle = np.sign(np.dot(axis, cross_product)) * angle

        return signed_angle


class RRTStar(RRT):
    """
    Class for RRT Star planning
    """

    class Node(RRT.Node):
        def __init__(self, x, y, z):
            super().__init__(x, y, z)
            self.cost = 0.0
            self.min_dist = 0.2

    def __init__(self,
                 robot_map,
                 start,
                 goal,
                 expand_dis=30.0,
                 path_resolution=1.0,
                 goal_sample_rate=20,
                 max_iter=300,
                 min_iter=300,
                 connect_circle_dist=50.0):
        """
        Setting Parameter
        goal:Goal Position [x,y]
        map: robot map, 0 = free, 1 = occupied, 2 = unobserved
        """
        super().__init__(robot_map, start, goal, expand_dis, path_resolution,
                         goal_sample_rate, max_iter)
        self.connect_circle_dist = connect_circle_dist
        self.goal_node = self.Node(goal[0], goal[1], goal[2])
        self.min_iter = min_iter
        self.sample_list = []
        self.samples_all = []

    def plan(self):
        """
        rrt star path planning
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            rnd = self.get_random_node()
            self.samples_all.append([rnd.x, rnd.y, rnd.z])

            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            new_node = self.steer(self.node_list[nearest_ind], rnd,
                                  self.expand_dis)
            near_node = self.node_list[nearest_ind]
            curr_cost = self.calc_distance(new_node, near_node)
            new_node.cost = near_node.cost + curr_cost

            if self.check_collision(new_node, self.map):
                near_inds = self.find_near_nodes(new_node)
                node_with_updated_parent = self.choose_parent(
                    new_node, near_inds)
                if node_with_updated_parent:
                    self.rewire(node_with_updated_parent, near_inds)
                    self.node_list.append(node_with_updated_parent)
                else:
                    self.node_list.append(new_node)

            if i > self.min_iter and new_node:  # if reaches goal
                last_index = self.search_best_goal_node()
                if last_index is not None:
                    return self.generate_final_course(last_index)

        print("reached max iteration")

        last_index = self.search_best_goal_node()
        if last_index is not None:
            return self.generate_final_course(last_index)

        return []

    def choose_parent(self, new_node, near_inds):
        """
        Computes the cheapest point to new_node contained in the list
        near_inds and set such a node as the parent of new_node.
            Arguments:
            --------
                new_node, Node
                    randomly generated node with a path from its neared point
                    There are not coalitions between this node and th tree.
                near_inds: list
                    Indices of indices of the nodes what are near to new_node
            Returns.
            ------
                Node, a copy of new_node
        """
        if not near_inds:
            return None

        # search nearest cost in near_inds
        costs = []
        for i in near_inds:
            near_node = self.node_list[i]
            t_node = self.steer(near_node, new_node)
            if t_node and self.check_collision(t_node, self.map):
                costs.append(self.calc_new_cost(near_node, new_node))
            else:
                costs.append(float("inf"))  # the cost of collision node
        min_cost = min(costs)

        if min_cost == float("inf"):
            print("There is no good path.(min_cost is inf)")
            return None

        min_ind = near_inds[costs.index(min_cost)]
        new_node = self.steer(self.node_list[min_ind], new_node)
        new_node.cost = min_cost

        return new_node

    def search_best_goal_node(self):
        dist_to_goal_list = [
            self.calc_dist_to_goal(n.x, n.y, n.z) for n in self.node_list
        ]
        goal_inds = [
            dist_to_goal_list.index(i) for i in dist_to_goal_list
            if i <= self.expand_dis
        ]

        safe_goal_inds = []
        for goal_ind in goal_inds:
            t_node = self.steer(self.node_list[goal_ind], self.goal_node)
            if self.check_collision(t_node, self.map):
                safe_goal_inds.append(goal_ind)

        if not safe_goal_inds:
            return None

        min_cost = min([self.node_list[i].cost for i in safe_goal_inds])
        for i in safe_goal_inds:
            if self.node_list[i].cost == min_cost:
                return i

        return None

    def find_near_nodes(self, new_node):
        """
        1) defines a ball centered on new_node
        2) Returns all nodes of the three that are inside this ball
            Arguments:
            ---------
                new_node: Node
                    new randomly generated node, without collisions between
                    its nearest node
            Returns:
            -------
                list
                    List with the indices of the nodes inside the ball of
                    radius r
        """
        nnode = len(self.node_list) + 1
        r = self.connect_circle_dist * math.sqrt((math.log(nnode) / nnode))
        # if expand_dist exists, search vertices in a range no more than
        # expand_dist
        if hasattr(self, 'expand_dis'):
            r = min(r, self.expand_dis)
        dist_list = [(node.x - new_node.x) ** 2 + (node.y - new_node.y) ** 2+(node.z - new_node.z) ** 2
                     for node in self.node_list]
        near_inds = [dist_list.index(i) for i in dist_list if i <= r ** 2]
        return near_inds

    def rewire(self, new_node, near_inds):
        """
            For each node in near_inds, this will check if it is cheaper to
            arrive to them from new_node.
            In such a case, this will re-assign the parent of the nodes in
            near_inds to new_node.
            Parameters:
            ----------
                new_node, Node
                    Node randomly added which can be joined to the tree
                near_inds, list of uints
                    A list of indices of the self.new_node which contains
                    nodes within a circle of a given radius.
            Remark: parent is designated in choose_parent.
        """
        for i in near_inds:
            near_node = self.node_list[i]
            edge_node = self.steer(new_node, near_node)
            if not edge_node:
                continue
            edge_node.cost = self.calc_new_cost(new_node, near_node)

            no_collision = self.check_collision(edge_node, self.map)
            improved_cost = near_node.cost > edge_node.cost

            if no_collision and improved_cost:
                near_node.x = edge_node.x
                near_node.y = edge_node.y
                near_node.z = edge_node.z

                near_node.cost = edge_node.cost
                near_node.path_x = edge_node.path_x
                near_node.path_y = edge_node.path_y
                near_node.path_z = edge_node.path_z
                near_node.parent = edge_node.parent
                self.propagate_cost_to_leaves(new_node)

    def calc_new_cost(self, from_node, to_node):
        d = self.calc_distance(from_node, to_node)
        return from_node.cost + d

    def propagate_cost_to_leaves(self, parent_node):

        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)
