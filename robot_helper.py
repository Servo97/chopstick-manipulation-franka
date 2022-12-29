import pybullet as p
import pybullet_tools.utils as pb_utils
import pybullet_data
import numpy as np
import time
from collections import namedtuple
from pybullet_tools.ikfast.ikfast import get_ik_joints, either_inverse_kinematics
import matplotlib.pyplot as plt
import cv2
from glob import glob
from scipy.spatial.transform import Rotation as R
import torch
import copy
from util import helper

BOARD_DIMS = np.array((0.457, 0.304))
WIDTH = 225
HEIGHT = 225
KERNEL = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
FIXED_ROTATION = [1, 0, 0, 0]
AMOUNT_TO_MOVE = 0.05  # 5cm
MOVABLE_JOINT_NUMBERS = [0, 1, 2, 3, 4, 5, 6]
CHOPSTICK_JOINT_NUMBERS = [8,9]
VIEWMATRIX = p.computeViewMatrix(
    cameraEyePosition=[0, 0, 0.6],
    cameraTargetPosition=[0, 0, 0],
    cameraUpVector=[0, 1, 0])

PROJECTIONMATRIX = p.computeProjectionMatrixFOV(
    fov=45,
    aspect=helper.WIDTH / helper.HEIGHT,
    nearVal=0.02,
    farVal=1)


class Robot:
    def __init__(self):
        pb_utils.connect(use_gui=True)
        pb_utils.add_data_path()
        p.setGravity(0, 0, -9.81)
        self.plane = pb_utils.load_pybullet("plane.urdf")
        # pb_utils.load_pybullet("assets/franka_description/robots/nugget.urdf")
        p.setTimeStep(1/1000, physicsClientId=pb_utils.CLIENT)
        p.setPhysicsEngineParameter(fixedTimeStep=1./1000.,solverResidualThreshold=0, physicsClientId=pb_utils.CLIENT)
        pb_utils.set_camera(75, -40, 0.9)

        shift = [0, 0, 0.65]
        meshScale = [0.003, 0.003, 0.003]
        with pb_utils.LockRenderer():
            self.franka = p.loadURDF('assets/franka_description/robots/franka_panda.urdf', basePosition=[-0.5, 0, 0.000000], baseOrientation=[
                        0.000000, 0.000000, 0.000000, 1.000000], useFixedBase=True, globalScaling=1, flags=p.URDF_USE_SELF_COLLISION)
            pb_utils.set_dynamics(self.franka, 8, linearDamping=0, lateralFriction=1)
            pb_utils.set_dynamics(self.franka, 9, linearDamping=0, lateralFriction=1)
            self.table1 = p.loadURDF("assets/table_collision/table.urdf", basePosition=[0,0,0.015], baseOrientation=[0, 0, 0.707, 0.707], globalScaling=0.3)
            self.cup_vis = p.createVisualShape(p.GEOM_MESH, fileName="assets/scene/bowl2.stl", meshScale=meshScale, rgbaColor=[0, 0, 1, 0.85],)
            self.cup_coll = p.createCollisionShape(p.GEOM_MESH, fileName="assets/scene/bowl2.stl", meshScale=meshScale,flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
            self.cup_1 = p.createMultiBody(baseMass=1,baseInertialFramePosition=[0, 0, 0],baseCollisionShapeIndex=self.cup_coll,baseVisualShapeIndex=self.cup_vis,
                                basePosition=[0, 0, 0.3], baseOrientation=[-0.7071068, 0, 0, 0.7071068],useMaximalCoordinates=True)

            self.table2 = p.loadURDF("assets/table_collision/table.urdf", basePosition=[-0.3,0.35,0.015], baseOrientation=[0, 0, 1, 0], globalScaling=0.3)
            self.cup_vis = p.createVisualShape(p.GEOM_MESH, fileName="assets/scene/bowl2.stl", meshScale=meshScale, rgbaColor=[0, 0, 1, 0.85],)
            self.cup_coll = p.createCollisionShape(p.GEOM_MESH, fileName="assets/scene/bowl2.stl", meshScale=meshScale,flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
            self.cup_2 = p.createMultiBody(baseMass=1,baseInertialFramePosition=[0, 0, 0],baseCollisionShapeIndex=self.cup_coll,baseVisualShapeIndex=self.cup_vis,
                                basePosition=[-0.3,0.35, 0.3], baseOrientation=[-0.7071068, 0, 0, 0.7071068],useMaximalCoordinates=True)
            self.nugget = p.loadURDF("assets/franka_description/robots/nugget.urdf", basePosition=[-0.15,0.35, 0.6], baseOrientation=[0, 0, 0, 1], globalScaling=1)
            # self.cup = p.loadURDF("assets/franka_description/robots/cup.urdf", 0,0,0.1, 0, 0, 0, 1)
            self.table3 = p.loadURDF("assets/table_collision/table.urdf", basePosition=[-0.3,0.35,0.17], baseOrientation=[0, 0, 1, 0], globalScaling=0.3)
        
        self.obstacles = [self.table1, self.cup_1, self.table2, self.cup_2, self.plane, self.table3]
        print(self.obstacles, self.franka)
        self.collision_fn = pb_utils.get_collision_fn(self.franka, MOVABLE_JOINT_NUMBERS , self.obstacles, self_collisions=True, max_distance=0)
        # helper.set_robot_to_reasonable_position(self.franka)
        for i in range(-1, 8):
            if i % 2 == 0: pb_utils.set_color(self.franka, link=i, color=(0.4, 0.4, 0.4, 1))
            else: pass
        

    def close_chopsticks(self, robot):
        print("Nugget: ", self.get_item_pose(self.nugget))
        print("EE: ", helper.get_gripper_position(self.franka))
        helper.control_joints(robot, CHOPSTICK_JOINT_NUMBERS, (-3, -3), velocity_scale=1, max_force=3)

    def open_chopsticks(self, robot):
        helper.control_joints(robot, CHOPSTICK_JOINT_NUMBERS, [10,10], velocity_scale=1)

    def get_item_pose(self, obj):
        return helper.get_obj_com_pose(obj)

    def check_pose_for_collision(self, robot, pos, rot=FIXED_ROTATION):
        desired_joints = helper.inverse_kinematics(robot, pos, rot)
        collision = self.collision_fn([np.radians(p) for p in desired_joints])
        return collision

    def go_to_position(self, robot, pos, rot = FIXED_ROTATION):
        desired_joints = helper.inverse_kinematics(robot, pos, rot)
        print(desired_joints)
        helper.control_joints_rad(robot, MOVABLE_JOINT_NUMBERS, desired_joints, velocity_scale=0.6)
    
    def go_to_position_rad(self, robot, joints):
        # desired_joints = helper.inverse_kinematics(robot, pos, rot)
        helper.control_joints_rad(robot, MOVABLE_JOINT_NUMBERS, joints, velocity_scale=1)

    def rotate_gripper(self, robot, pos, theta):
        ori = helper.get_quat(theta)
        self.go_to_position(robot, pos, ori)
        return ori
        
    def go_to_object(self, robot, obj, offset = [0,0,0]):
        # chopstick_tf_matrix = np.array(np.vstack((np.hstack([np.eye(3), [[-0.03], [0.0], [0.21]]]), [0,0,0,1])))
        ee_pose = helper.get_gripper_position(robot)
        ee_rot_matrix = pb_utils.matrix_from_tform(pb_utils.tform_from_pose(ee_pose))
        # chopstick_tf_matrix[:3, :3] = ee_rot_matrix
        obj_tf_matrix = pb_utils.tform_from_pose(self.get_item_pose(obj))
        obj_tf_matrix[0,3] += -0.02 + offset[0]
        obj_tf_matrix[2,3] += 0.205 + offset[2]
        obj_tf_matrix[1,3] += offset[1]
        ee_tf_matrix = pb_utils.pose_from_tform(obj_tf_matrix)
        # self.go_to_position(robot, ee_tf_matrix[0], ee_tf_matrix[1])
        self.go_to_position(robot, ee_tf_matrix[0], FIXED_ROTATION)

    def get_start_goal_config(self, robot, obj, offset = 0.05):
        # chopstick_tf_matrix = np.array(np.vstack((np.hstack([np.eye(3), [[-0.03], [0.0], [0.21]]]), [0,0,0,1])))
        start_config = pb_utils.get_joint_positions(robot, MOVABLE_JOINT_NUMBERS)
        ee_pose = helper.get_gripper_position(robot)
        ee_rot_matrix = pb_utils.matrix_from_tform(pb_utils.tform_from_pose(ee_pose))
        # chopstick_tf_matrix[:3, :3] = ee_rot_matrix
        obj_tf_matrix = pb_utils.tform_from_pose(self.get_item_pose(obj))
        obj_tf_matrix[2,3] += offset + 0.21
        obj_tf_matrix[0,3] += -0.02
        ee_tf_matrix = pb_utils.pose_from_tform(obj_tf_matrix)
        goal_config = helper.inverse_kinematics(robot, ee_tf_matrix[0], FIXED_ROTATION)
        return np.array(start_config), np.array(goal_config)