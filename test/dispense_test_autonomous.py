#!/usr/bin/env python3

from multiprocessing.connection import wait
import sys
import rospy
import numpy as np
import tf2_ros
from tf2_geometry_msgs.tf2_geometry_msgs import PoseStamped
from tf.transformations import *
import math
import time
from geometry_msgs.msg import Quaternion, TransformStamped, Pose

from motion.commander import RobotMoveGroup
from motion.utils import offset_pose, offset_joint, make_pose



# taped container position pre grasp/home
HOME_POSITION_JOINT = [0.733216, -2.303996, 2.222002, -3.058677, -0.733774, 3.14139]
INGREDIENT_POSITION = {
    2: [0.01, -0.2699, 0.66, 0.7071068, 0.0, 0.0, 0.7071068]
}
# [-0.0216083012128164, -0.2808664510261085, 0.7377250673870849, -0.6857402840447018, -0.012217903053823087, -0.01605485599230219, -0.7273590500971724]
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

def get_marker_pose(ingredient_id):
    tfBuffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tfBuffer)
    marker_origin = Pose()
    marker_origin.position.x = 0.00
    marker_origin.position.y = 0.00
    marker_origin.position.z = 0.00
    quat = quaternion_from_euler(0, 0, 0)
    marker_origin.orientation.x = quat[0]
    marker_origin.orientation.y = quat[1]
    marker_origin.orientation.z = quat[2]
    marker_origin.orientation.w = quat[3]

    target_pose = PoseStamped()
    target_pose.pose = marker_origin
    target_pose.header.frame_id = 'pregrasp_' + str(ingredient_id)
    target_pose.header.stamp = rospy.Time.now()

    target_pose_base = None
    while target_pose_base is None:
        try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            target_pose_base = tfBuffer.transform(target_pose, 'base_link', rospy.Duration(1))
        # rospy.loginfo(target_pose_base)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

    return target_pose_base


def run():
    rospy.init_node("ur5e_move_test")
    rate = rospy.Rate(10.0)

    robot_mg = RobotMoveGroup(verbose=True)

    # open gripper and go to home position
    robot_mg.open_gripper() or sys.exit(1)
    robot_mg.go_to_joint_state(HOME_POSITION_JOINT) or sys.exit(1)

    print("Reach home position. Enter ingredient number:", end='')
    ingredient_id = int(input())

    # go to expected ingredient location (from where marker is visible)
    robot_mg.go_to_pose_goal(
        make_pose(INGREDIENT_POSITION[ingredient_id][:3], INGREDIENT_POSITION[ingredient_id][3:]),
        cartesian_path=True,
        acc_scaling=0.1,
    ) or sys.exit(1)

    time.sleep(2)

    target_pose = get_marker_pose(ingredient_id=ingredient_id)

    rospy.loginfo("Target pose:")
    rospy.loginfo(target_pose.pose)

    print("Press key to start moving towards object:")
    input()

    # # go to pre-grasp position
    robot_mg.go_to_pose_goal(
        target_pose.pose,
        cartesian_path=True,
        acc_scaling=0.1,
    ) #or sys.exit(1) #TODO-nevalsar, harshita


    print("Reached pre-grasp. Going to gripping position next.")
    input()

    tfBuffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tfBuffer)

    pose_marker_wrist_frame = Pose()
    pose_marker_wrist_frame.position.z = 0.175
    pose_marker_wrist_frame.orientation.w = 1

    pose_stamped_wrist_frame = PoseStamped()
    pose_stamped_wrist_frame.pose = pose_marker_wrist_frame
    pose_stamped_wrist_frame.header.frame_id = 'wrist_3_link'
    pose_stamped_wrist_frame.header.stamp = rospy.Time.now()


    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        pose_marker_base_frame = tfBuffer.transform(pose_stamped_wrist_frame, 'base_link', rospy.Duration(1))
        rospy.loginfo(pose_marker_base_frame)

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise

    # go to ingredient gripping position
    robot_mg.go_to_pose_goal(
        pose_marker_base_frame.pose,
        cartesian_path=True,
        acc_scaling=0.1,
    ) or sys.exit(1)

    robot_mg.close_gripper(wait=True) or sys.exit(1)

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
        offset_pose(robot_mg.get_current_pose(), [0, 0, 0.10]),
        cartesian_path=True,
        acc_scaling=0.1,
    ) or sys.exit(1)

    # go to ingredient position
    robot_mg.go_to_pose_goal(
        make_pose(INGREDIENT_POSITION[ingredient_id][:3], INGREDIENT_POSITION[ingredient_id][3:]),
        cartesian_path=True,
        acc_scaling=0.1,
    ) or sys.exit(1)

    print("Press any key to continue:")
    input()

    robot_mg.go_to_pose_goal(
        offset_pose(robot_mg.get_current_pose(), [0, -0.2, -0.05]),
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

    print("DONE")


if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        sys.exit(1)
