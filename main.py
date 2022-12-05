import numpy as np
import robot_helper as rh
import time
from util import helper
import pybullet_tools.utils as pb_utils


if __name__ == "__main__":
    time.sleep(0.5)
    robot = rh.Robot()
    # robot.initialize_robot_arm(robot.franka, robot.nugget)

    helper.set_robot_to_reasonable_position(robot.franka)
    time.sleep(0.5)
    while True:
        robot.open_chopsticks(robot.franka)
        robot.go_to_object(robot.franka, robot.nugget)
        robot.close_chopsticks(robot.franka)
        time.sleep(1)
        robot.go_to_object(robot.franka, robot.nugget, offset = 0.2)
        time.sleep(1)