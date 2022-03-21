#!/usr/bin/env python3

from multiprocessing.connection import wait
import sys
import rospy
import numpy as np
import tf2_ros
from tf.transformations import *
import math
import time
from geometry_msgs.msg import Quaternion, TransformStamped

from motion.commander import RobotMoveGroup
from motion.utils import offset_pose, offset_joint, make_pose


# taped container position pre grasp/home
HOME_POSITION_JOINT = [0.733216, -2.303996, 2.222002, -3.058677, -0.733774, 3.14139]
INGREDIENT_POSITION = [0.0099, -0.4699, 0.6450, 0.7071068, 0.0, 0.0, 0.7071068]
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


def run():
    rospy.init_node("ur5e_move_test")
    rate = rospy.Rate(10.0)
    robot_mg = RobotMoveGroup(verbose=True)

    # open gripper and go to home position
    robot_mg.open_gripper() or sys.exit(1)
    robot_mg.go_to_joint_state(HOME_POSITION_JOINT) or sys.exit(1)

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "ar_marker_2"
    static_transformStamped.child_frame_id = "ar_marker_2_pregrasp"

    static_transformStamped.transform.translation.x = 0
    static_transformStamped.transform.translation.y = -0.05
    static_transformStamped.transform.translation.z = 0.30

    quat = quaternion_from_euler(0, math.pi, 0)
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]

    broadcaster.sendTransform(static_transformStamped)

    # while not rospy.is_shutdown():
    #     rate.sleep()

    tfBuffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tfBuffer)

    # q_rot = quaternion_from_euler(0, 0, math.pi)

    transform = None
    while not rospy.is_shutdown() and transform is None:
        try:
            transform = tfBuffer.lookup_transform(
                "base_link", "ar_marker_2_pregrasp", rospy.Time()
            )
            transform = transform.transform
            rospy.loginfo(transform)
            # q_new = q_rot.copy()
            # q_new[0] = transform.rotation.x
            # q_new[1] = transform.rotation.y
            # q_new[2] = transform.rotation.z
            # q_new[3] = transform.rotation.w
            # quaternion = quaternion_multiply(q_rot, q_new)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            rate.sleep()
            continue
        rate.sleep()

    # pose_marker = [
    #     transform.translation.x,
    #     transform.translation.y,
    #     transform.translation.z,
    #     quaternion[0],
    #     quaternion[1],
    #     quaternion[2],
    #     quaternion[3],
    # ]
    pose_marker = [
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z,
        transform.rotation.w,
    ]
    rospy.loginfo("Target pose:")
    rospy.loginfo(pose_marker)

    print("Press key to start moving towards object:")
    input()

    # # go to pre-grasp position
    robot_mg.go_to_pose_goal(
        make_pose(pose_marker[:3], pose_marker[3:]),
        cartesian_path=True,
        acc_scaling=0.1,
    ) #or sys.exit(1) #TODO-nevalsar, harshita

    print("Reached pre-grasp. Going to gripping position next.")
    input()
    # go to ingredient gripping position
    robot_mg.go_to_pose_goal(
        offset_pose(robot_mg.get_current_pose(), [0, -0.2, 0]),
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

    print("DONE")


if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        sys.exit(1)
