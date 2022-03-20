#!/usr/bin/env python3

import sys
import rospy
import numpy as np

from motion.commander import RobotMoveGroup
from motion.utils import offset_pose, offset_joint, make_pose


# taped container position pre grasp/home
HOME_POSITION_JOINT = [0.733216, -2.303996, 2.222002, -3.058677, -0.733774, 3.14139]
INGREDIENT_POSITION = [0.0099, -0.4699, 0.6450, 0.7071068, 0.0, 0.0, 0.7071068]
DISPENSE_POSITION = [-0.4698, 0.0, 0.2950, 0.5, -0.5, -0.5, 0.5]
PRE_DISPENSE_POSITION_CARTESIAN = [-0.2698, 0.0, 0.0, 0.6950, 0.5, -0.5, -0.5, 0.5]
PRE_DISPENSE_POSITION_JOINT = [
    -0.838740,
    -2.303996,
    2.222002,
    -3.058677,
    -0.733774,
    3.14139,
]


def run():
    rospy.init_node("ur5e_move_test")
    robot_mg = RobotMoveGroup(verbose=True)

    # open gripper and go to home position
    robot_mg.open_gripper() or sys.exit(1)
    robot_mg.go_to_joint_state(HOME_POSITION_JOINT) or sys.exit(1)

    # go to ingredient position and grip container
    robot_mg.go_to_pose_goal(
        make_pose(INGREDIENT_POSITION[:3], INGREDIENT_POSITION[3:]),
        cartesian_path=True,
        acc_scaling=0.1,
    ) or sys.exit(1)
    robot_mg.close_gripper() or sys.exit(1)

    print("Press any key to continue:")
    input()

    # pick up container
    robot_mg.go_to_pose_goal(
        offset_pose(robot_mg.get_current_pose(), [0, 0.2, 0.05]),
        cartesian_path=True,
        acc_scaling=0.1,
    ) or sys.exit(1)

    # rotate right / go to pre-dispense position
    robot_mg.go_to_joint_state(PRE_DISPENSE_POSITION_JOINT) or sys.exit(1)

    # go to dispense position
    robot_mg.go_to_pose_goal(
        make_pose(DISPENSE_POSITION[:3], DISPENSE_POSITION[3:]), cartesian_path=True
    ) or sys.exit(1)

    # dispense - container tilt
    robot_mg.go_to_joint_state(
        offset_joint(robot_mg.get_current_joints(), [0, 0, 0, 0, 0, 3 * np.pi / 4])
    ) or sys.exit(1)
    # container tilt upright
    robot_mg.go_to_joint_state(
        offset_joint(robot_mg.get_current_joints(), [0, 0, 0, 0, 0, -3 * np.pi / 4])
    ) or sys.exit(1)

    # go to pre-dispense position
    robot_mg.go_to_joint_state(PRE_DISPENSE_POSITION_JOINT) or sys.exit(1)

    # go to home position
    robot_mg.go_to_joint_state(HOME_POSITION_JOINT) or sys.exit(1)

    # go up a little to prevent container hitting the shelf
    robot_mg.go_to_pose_goal(
        offset_pose(robot_mg.get_current_pose(), [0, 0, 0.07]),
        cartesian_path=True,
        acc_scaling=0.1,
    ) or sys.exit(1)

    # go to ingredient position
    robot_mg.go_to_pose_goal(
        make_pose(INGREDIENT_POSITION[:3], INGREDIENT_POSITION[3:]),
        cartesian_path=True,
        acc_scaling=0.1,
    ) or sys.exit(1)

    print("Press any key to continue:")
    input()

    # release container
    robot_mg.open_gripper() or sys.exit(1)

    # go back from container
    robot_mg.go_to_pose_goal(
        offset_pose(robot_mg.get_current_pose(), [0, 0.2, 0.05]),
        cartesian_path=True,
        acc_scaling=0.1,
    ) or sys.exit(1)

    # go to home position
    robot_mg.go_to_joint_state(HOME_POSITION_JOINT) or sys.exit(1)


if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        sys.exit(1)
