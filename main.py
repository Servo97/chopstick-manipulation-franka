import numpy as np
import robot_helper as rh
import time
from util import helper
import pybullet_tools.utils as pb_utils
from planning import RRT

def execute_plan(robot, ndof, start_config, goal_config):
    rrtree = RRT.RRT(robot, ndof, start_config, goal_config)
    # Lock renderer
    # with pb_utils.LockRenderer():
    plan = rrtree.RRTConnPlanner(goal_config, ndof, max_iter=10000)
    print(plan)
    if plan is not None:
        for point in plan:
            print(point)
            robot.go_to_position_rad(robot.franka, list(point))
        robot.go_to_object(robot.franka, robot.nugget)
        time.sleep(0.1)
        robot.close_chopsticks(robot.franka)
        time.sleep(0.1)
        robot.go_to_object(robot.franka, robot.nugget, offset = [0, 0, 0.1])
        time.sleep(0.1)
        robot.go_to_object(robot.franka, robot.nugget, offset = [0, -0.1, 0])
        time.sleep(0.1)
        robot.go_to_object(robot.franka, robot.nugget, offset = [0, -0.1, 0])
        time.sleep(0.1)
        robot.go_to_object(robot.franka, robot.nugget, offset = [0, -0.05, 0])
        time.sleep(0.1)
        robot.go_to_object(robot.franka, robot.table1, offset = [0, 0, 0.4])
        time.sleep(0.1)
        robot.go_to_object(robot.franka, robot.table1, offset = [0, 0, 0.3])
        time.sleep(0.1)
        robot.go_to_object(robot.franka, robot.table1, offset = [0, 0, 0.2])
        time.sleep(0.1)
        robot.open_chopsticks(robot.franka)
        time.sleep(0.1)
        robot.go_to_object(robot.franka, robot.table1, offset = [0, 0, 0.4])
        time.sleep(0.1)
    # while True:
    #     robot.open_chopsticks(robot.franka)
    #     robot.go_to_object(robot.franka, robot.nugget)
    #     robot.close_chopsticks(robot.franka)
    #     time.sleep(1)
    #     robot.go_to_object(robot.franka, robot.nugget, offset = 0.2)
    #     time.sleep(1)

if __name__ == "__main__":
    # time.sleep(0.5)
    robot = rh.Robot()
    # robot.initialize_robot_arm(robot.franka, robot.nugget)
    time.sleep(0.5)

    helper.set_robot_to_reasonable_position(robot.franka)
    # print(robot.check_pose_for_collision(robot.franka, [0,0,0.25]))
    # print(robot.check_pose_for_collision(robot.franka, [-0.2,-0.4,0.25]))
    time.sleep(0.5)

    # while True:
    # robot.go_to_position(robot.franka, [0,0,0.25])
    # # robot.open_chopsticks(robot.franka)
    # # robot.go_to_object(robot.franka, robot.nugget, offset = [0, 0, 0.01])
    # # robot.close_chopsticks(robot.franka)
    # # time.sleep(0.1)
    # # # robot.go_to_object(robot.franka, robot.nugget, offset = [0, 0, 0.2])
    # # time.sleep(0.1)
    # # robot.go_to_object(robot.franka, robot.table1, offset = [0, 0, 0.3])

    # TESTING COMMENT COMPARISON
    robot.open_chopsticks(robot.franka)
    time.sleep(0.5)
    start, goal = robot.get_start_goal_config(robot.franka, robot.nugget)
    print(start)
    print(goal)
    execute_plan(robot, 7, start, goal)