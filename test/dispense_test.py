#!/usr/bin/env python3

import sys
import rospy
import numpy as np

from geometry_msgs.msg import Pose, Twist

from motion.commander import RobotMoveGroup
from motion.utils import offset_pose, offset_joint, make_pose

# taped container position pre grasp/home
HOME_JOINT = [0.733216, -2.303996, 2.222002, -3.058677, -0.733774, 3.14139]


def run():
    rospy.init_node("ur5e_move_test")
    robot_mg = RobotMoveGroup()

    robot_mg.go_to_joint_state(HOME_JOINT)

    robot_mg.go_to_pose_goal(
        offset_pose(robot_mg.get_current_pose(), [0, -0.2, 0]),
        cartesian_path=True,
        acc_scaling=0.1,
    )

    robot_mg.go_to_pose_goal(
        offset_pose(robot_mg.get_current_pose(), [0, 0.2, 0.05]),
        cartesian_path=True,
        acc_scaling=0.1,
    )

    robot_mg.go_to_joint_state(
        offset_joint(robot_mg.get_current_joints(), [-np.pi / 2, 0, 0, 0, 0, 0])
    )

    robot_mg.go_to_pose_goal(
        make_pose([-0.34, 0.08, 0.39], [0.5, -0.5, -0.5, 0.5]), cartesian_path=True
    )

    robot_mg.go_to_joint_state(
        offset_joint(robot_mg.get_current_joints(), [0, 0, 0, 0, 0, np.pi / 2])
    )
    robot_mg.go_to_joint_state(
        offset_joint(robot_mg.get_current_joints(), [0, 0, 0, 0, 0, -np.pi / 2])
    )

    robot_mg.go_to_pose_goal(
        make_pose([-0.25, 0.03, 0.6], [0.5, -0.5, -0.5, 0.5]), cartesian_path=True
    )

    robot_mg.go_to_joint_state(
        offset_joint(robot_mg.get_current_joints(), [np.pi / 2, 0, 0, 0, 0, 0])
    )

    robot_mg.go_to_pose_goal(
        make_pose([0.01, -0.27, 0.66], [0.7071068, 0, 0, 0.7071068]),
        cartesian_path=True,
    )

    robot_mg.go_to_pose_goal(
        offset_pose(robot_mg.get_current_pose(), [0, -0.2, -0.05]),
        cartesian_path=True,
        acc_scaling=0.1,
    )

    # robot_mg.go_to_pose_goal(Pose([0.01, -0.27, 0.61], [0.7071068, 0, 0, 0.7071068]), cartesian_path=True)

    robot_mg.go_to_pose_goal(
        offset_pose(robot_mg.get_current_pose(), [0, 0.2, 0]),
        cartesian_path=True,
        acc_scaling=0.1,
    )
    robot_mg.go_to_joint_state(HOME_JOINT)


if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        sys.exit(1)
