import numpy as np
from scipy.spatial import KDTree
sys.path.append('../pybullet_tools')
import pybullet_tools.utils as pb_utils

MOVABLE_JOINT_NUMBERS = [0, 1, 2, 3, 4, 5, 6]
class RRT:
    def __init__(self, robot, ndof, start_config, goal_config, epsilon):
        self.robot = robot
        self.nodes = []
        self.goal_config = None
        self.final_path = []
        self.cost = {}
        self.graph = {}
        self.epsilon = epsilon
        self.nodes.append(start_config)
        self.sample_fn = pb_utils.get_sample_fn(robot, MOVABLE_JOINT_NUMBERS, custom_limits={})
    
    def add_vertex(self, q):
        self.nodes.append(q)
    def add_edge(self, q_parent, q_child):
        self.graph[q_child] = q_parent

    def nearest_neighbor(self, q):
        _, idx = KDTree(data=self.nodes, leafsize=10).query(q)
        return idx

    def print_q(self, q):
        print(q)

    def new_config(self, q_near, q_rand):
        
        return q_new

    def extend_fn(self, q, q_goal):
        nn = self.nearest_neighbor(q)
        q_near = self.nodes[nn]


    def RRTPlanner(self, goal_config, max_iter=1000):
        self.goal_config = goal_config
        for i in range(max_iter):
            if np.random.random() < 0.5:
                q_rand = self.sample_fn()
            else:
                q_rand = self.goal_config

            q_near_idx = self.nearest_neighbor(q_rand)
            q_near = self.nodes[q_near_idx]
            q_new = self.steer(q_near, q_rand)
            if q_new is not None:
                self.add_vertex(q_new)
                self.add_edge(q_near, q_new)
                if self.check_goal(q_new):
                    self.final_path = self.get_path(q_new)
                    return self.final_path
        return None