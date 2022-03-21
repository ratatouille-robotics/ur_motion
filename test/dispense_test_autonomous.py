#!/usr/bin/env python3

from multiprocessing.connection import wait
import sys
import rospy
import numpy as np
import tf2_ros
from tf.transformations import *
import math
import time
from geometry_msgs.msg import Quaternion

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
    rate = rospy.Rate(10.0)
    robot_mg = RobotMoveGroup(verbose=True)

    # open gripper and go to home position
    robot_mg.open_gripper() or sys.exit(1)
    robot_mg.go_to_joint_state(HOME_POSITION_JOINT) or sys.exit(1)

    tfBuffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tfBuffer)

    q_rot = quaternion_from_euler(0, 0, math.pi)

    transform = None
    while not rospy.is_shutdown() and transform is None:
        try:
            transform = tfBuffer.lookup_transform(
                "ar_marker_2", "base_link", rospy.Time()
            )
            transform = transform.transform
            q_new = q_rot.copy()
            q_new[0] = transform.rotation.x
            q_new[1] = transform.rotation.y
            q_new[2] = transform.rotation.z
            q_new[3] = transform.rotation.w
            quaternion = quaternion_multiply(q_rot, q_new)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            rate.sleep()
            continue
        rate.sleep()

    pose_marker = [
        transform.translation.x,
        transform.translation.y,
        transform.translation.z,
        quaternion[0],
        quaternion[1],
        quaternion[2],
        quaternion[3],
    ]
    rospy.loginfo("Target pose:")
    rospy.loginfo(pose_marker)

    print("Press key to start moving towards object:")
    input()

    # go to marker position and grip container
    robot_mg.go_to_pose_goal(
        make_pose(pose_marker[:3], pose_marker[3:]),
        cartesian_path=True,
        acc_scaling=0.1,
    ) or sys.exit(1)

    print("DONE")


if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        sys.exit(1)
