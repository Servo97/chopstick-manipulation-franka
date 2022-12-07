import numpy as np
import robot_helper as rh
import time
from util import helper
import pybullet_tools.utils as pb_utils
from planning import RRT

def execute_plan(robot):
    


    while True:
        robot.open_chopsticks(robot.franka)
        robot.go_to_object(robot.franka, robot.nugget)
        robot.close_chopsticks(robot.franka)
        time.sleep(1)
        robot.go_to_object(robot.franka, robot.nugget, offset = 0.2)
        time.sleep(1)

if __name__ == "__main__":
    # time.sleep(0.5)
    robot = rh.Robot()
    # robot.initialize_robot_arm(robot.franka, robot.nugget)
    time.sleep(0.5)

    helper.set_robot_to_reasonable_position(robot.franka)
    print(robot.check_pose_for_collision(robot.franka, [0,0,0.25]))
    print(robot.check_pose_for_collision(robot.franka, [-0.2,-0.4,0.25]))

    while True:
        robot.go_to_position(robot.franka, [0,-0.3,0.25])