import numpy as np
from scipy.spatial import KDTree
import sys
sys.path.append('../pybullet_tools')
import pybullet_tools.utils as pb_utils
from itertools import takewhile


MOVABLE_JOINT_NUMBERS = [0, 1, 2, 3, 4, 5, 6]
class RRT:
    def __init__(self, robot, ndof, start_config, goal_config, epsilon = 0.01):
        self.robot = robot
        self.nodes = []
        self.goal_config = None
        self.final_path = []
        self.cost = {}
        self.graph = {}
        self.epsilon = epsilon
        self.thresh = 0.4
        self.nodes.append(start_config)
        self.sample_fn = pb_utils.get_sample_fn(robot.franka, MOVABLE_JOINT_NUMBERS, custom_limits={})

    
    def add_vertex(self, q):
        self.nodes.append(np.array(q))
    def add_edge(self, q_parent, q_child):
        self.graph[q_child] = q_parent

    def nearest_neighbor(self, q):
        _, idx = KDTree(data=self.nodes, leafsize=10).query(q)
        return idx

    def print_q(self, q):
        print(q)

    def new_config(self, q_rand, q_near, ndof):
        dist = 0
        for i in range(ndof):
            if dist < abs(q_near[i] - q_rand[i]):
                dist = abs(q_near[i] - q_rand[i])
        numsamples = int(dist / self.epsilon)
        if numsamples < 2:
            return -1, q_rand
        trapped = 0
        q_new = None
        for i in range(numsamples):
            q_new_temp = q_near + (q_rand - q_near) * (i / (numsamples - 1))
            if not self.robot.collision_fn(q_new_temp):
                if(np.linalg.norm(q_new_temp - q_near) <= self.thresh):
                    trapped += 1
                    q_new = q_new_temp
            else:
                break
        return trapped, q_new

    def extend_fn(self, q, q_goal, ndof):
        nn = self.nearest_neighbor(q)
        q_near = self.nodes[nn]
        trapped, q_new = self.new_config(q, q_near, ndof)
        if trapped != 0:
            self.add_vertex(q_new)
            self.add_edge(nn, len(self.nodes) - 1)
            if trapped == -1:
                if np.array_equal(q_new,q_goal):
                    return len(self.nodes) - 1
        return -1


    def RRTConnPlanner(self, goal_config, ndof, max_iter=1000):
        self.goal_config = goal_config
        for i in range(max_iter):
            if np.random.random() < 0.5:
                q_rand = self.sample_fn()
            else:
                q_rand = self.goal_config
            ret = self.extend_fn(q_rand, self.goal_config, ndof)
            if ret != -1:
                while ret != 0:
                    self.final_path.append(self.nodes[ret])
                    ret = self.graph[ret]
                self.final_path.append(self.nodes[0])
                self.final_path.reverse()
                print(f"Path Found! #steps: {len(self.final_path)}")
                return self.final_path
        else:
            print("No path found")
            return None
        return self.final_path