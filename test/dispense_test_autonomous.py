#!/usr/bin/env python3

import argparse
from faulthandler import disable
from multiprocessing.connection import wait
import sys
import time
import rospy
import numpy as np
from tf2_geometry_msgs.tf2_geometry_msgs import PoseStamped
from tf.transformations import *
from geometry_msgs.msg import Quaternion, TransformStamped, Pose

from motion.commander import RobotMoveGroup
from motion.utils import offset_pose, offset_joint, make_pose
from ratatouille_pose_transforms.transforms import PoseTransforms


# taped container position pre grasp/home
HOME_POSITION_JOINT = [0.733216, -2.303996, 2.222002, -3.058677, -0.733774, 3.14139]
INGREDIENT_POSITION = {
    1: [0.26, -0.2699, 0.66, 0.7071068, 0.0, 0.0, 0.7071068],
    2: [0.01, -0.2699, 0.66, 0.7071068, 0.0, 0.0, 0.7071068],
    3: [-0.24, -0.2699, 0.66, 0.7071068, 0.0, 0.0, 0.7071068],
}
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


def run(disable_gripper: bool = False):
    rospy.init_node("ur5e_move_test")
    rate = rospy.Rate(10.0)

    robot_mg = RobotMoveGroup(verbose=True)
    pose_transformer = PoseTransforms()

    ############################################################################
    # open gripper and go to home position

    if not disable_gripper:
        robot_mg.open_gripper() or sys.exit(1)
    robot_mg.go_to_joint_state(HOME_POSITION_JOINT) or sys.exit(1)

    print("Reached home position. Enter ingredient number:", end="")
    ingredient_id = int(input())

    ############################################################################
    # go to expected ingredient location (from where marker is visible)

    robot_mg.go_to_pose_goal(
        make_pose(
            INGREDIENT_POSITION[ingredient_id][:3],
            INGREDIENT_POSITION[ingredient_id][3:],
        ),
        cartesian_path=True,
        acc_scaling=0.1,
    ) or sys.exit(1)

    # print("Press any key to begin searching for markers")
    # input()
    time.sleep(1)
    ############################################################################
    # get pregrasp pose for ingredient

    marker_origin = Pose()
    marker_origin.position.x = 0.00
    marker_origin.position.y = 0.00
    marker_origin.position.z = 0.00
    quat = quaternion_from_euler(0, 0, 0)
    marker_origin.orientation.x = quat[0]
    marker_origin.orientation.y = quat[1]
    marker_origin.orientation.z = quat[2]
    marker_origin.orientation.w = quat[3]

    target_pose = pose_transformer.transform_pose_to_frame(
        pose_source=marker_origin,
        header_frame_id="pregrasp_" + str(ingredient_id),
        base_frame_id="base_link",
    )
    if target_pose is None:
        print(f"Can't find ingredient! Exiting.")
        sys.exit(1)

    # rospy.loginfo("Target pose:")
    # rospy.loginfo(target_pose.pose)

    # print("Press key to start moving towards object:")
    # input()

    ############################################################################
    # go to pre-grasp position

    robot_mg.go_to_pose_goal(
        target_pose.pose,
        cartesian_path=True,
        acc_scaling=0.1,
    )  # or sys.exit(1) #TODO-nevalsar, harshita

    # print("Reached pre-grasp. Going to gripping position next.")
    # input()

    ############################################################################
    # compute gripping pose

    pose_marker_wrist_frame = Pose()
    pose_marker_wrist_frame.position.z = 0.175
    pose_marker_wrist_frame.orientation.w = 1

    pose_marker_base_frame = pose_transformer.transform_pose_to_frame(
        pose_source=pose_marker_wrist_frame,
        header_frame_id="wrist_3_link",
        base_frame_id="base_link",
    )

    ############################################################################
    # go to ingredient gripping position

    robot_mg.go_to_pose_goal(
        pose_marker_base_frame.pose,
        cartesian_path=True,
        acc_scaling=0.1,
    ) or sys.exit(1)

    ############################################################################
    # grip container
    if not disable_gripper:
        robot_mg.close_gripper(wait=True) or sys.exit(1)

    print("Press any key to continue:")
    input()

    ############################################################################
    # pick container and go back

    robot_mg.go_to_pose_goal(
        offset_pose(robot_mg.get_current_pose(), [0, 0.2, 0.05]),
        cartesian_path=True,
        acc_scaling=0.1,
    ) or sys.exit(1)

    ############################################################################
    # go to home position
    robot_mg.go_to_joint_state(HOME_POSITION_JOINT) or sys.exit(1)

    ############################################################################
    # rotate right / go to pre-dispense position
    robot_mg.go_to_joint_state(PRE_DISPENSE_POSITION_JOINT) or sys.exit(1)

    ############################################################################
    # go to dispense position

    robot_mg.go_to_pose_goal(
        make_pose(DISPENSE_POSITION[:3], DISPENSE_POSITION[3:]), cartesian_path=True
    ) or sys.exit(1)

    ############################################################################
    # dispense - container tilt

    robot_mg.go_to_joint_state(
        offset_joint(robot_mg.get_current_joints(), [0, 0, 0, 0, 0, 3 * np.pi / 4])
    ) or sys.exit(1)

    ############################################################################
    # container tilt upright

    robot_mg.go_to_joint_state(
        offset_joint(robot_mg.get_current_joints(), [0, 0, 0, 0, 0, -3 * np.pi / 4])
    ) or sys.exit(1)

    ############################################################################
    # go to pre-dispense position

    robot_mg.go_to_joint_state(PRE_DISPENSE_POSITION_JOINT) or sys.exit(1)

    ############################################################################
    # go to home position

    robot_mg.go_to_joint_state(HOME_POSITION_JOINT) or sys.exit(1)


    ############################################################################
    # go to ingredient view position

    robot_mg.go_to_pose_goal(
        make_pose(
            INGREDIENT_POSITION[ingredient_id][:3],
            INGREDIENT_POSITION[ingredient_id][3:],
        ),
        cartesian_path=True,
        acc_scaling=0.1,
    ) or sys.exit(1)

    # print("At ingredient view position. Press key to continue:")
    # input()

    ############################################################################
    # go up a little (to prevent container hitting the shelf)

    robot_mg.go_to_pose_goal(
        offset_pose(robot_mg.get_current_pose(), [0, 0, 0.075]),
        cartesian_path=True,
        acc_scaling=0.1,
    ) or sys.exit(1)

    print("Press key to go to ingredient position:")
    input()

    ############################################################################
    # go to ingredient position

    robot_mg.go_to_pose_goal(
        offset_pose(robot_mg.get_current_pose(), [0, -0.2, -0.05]),
        cartesian_path=True,
        acc_scaling=0.1,
    ) or sys.exit(1)

    # print("Press any key to open gripper:")
    # input()

    ############################################################################
    # release gripper
    if not disable_gripper:
        robot_mg.open_gripper(wait=True) or sys.exit(1)

    ############################################################################
    # go back from container

    robot_mg.go_to_pose_goal(
        offset_pose(robot_mg.get_current_pose(), [0, 0.30, 0.05]),
        cartesian_path=True,
        acc_scaling=0.1,
    ) or sys.exit(1)

    ############################################################################
    # go to home position

    robot_mg.go_to_joint_state(HOME_POSITION_JOINT) or sys.exit(1)

    print("DONE")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--disable-gripper", help="Disable gripper commands", action="store_true"
    )
    args = parser.parse_args()

    try:
        run(disable_gripper=args.disable_gripper)
    except rospy.ROSInterruptException:
        sys.exit(1)
