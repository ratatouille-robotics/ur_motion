#!/usr/bin/env python3

import rospy
import copy
import numpy as np

from geometry_msgs.msg import Pose, Twist

from ur_motion.motion_py import UR5eMoveGroup

def run():
    rospy.init_node("ur5e_move_test")
    robot_mg = UR5eMoveGroup()
    # check movements with joint targets
    target_joint = [0, -np.pi / 2, np.pi / 2, 0, np.pi / 2, 0]
    robot_mg.go_to_joint_state(target_joint, cartesian_path=False)
    # check movements with pose targets
    pose = Pose()
    pose.position.x = -0.6
    pose.position.y = -0.2
    pose.position.z = 0.6
    pose.orientation.x = -0.5
    pose.orientation.y = 0.5
    pose.orientation.z = 0.5
    pose.orientation.w = -0.5
    robot_mg.go_to_pose_goal(pose, cartesian_path=True)
    # check pose trajectory control
    pose_list = []
    for i in range(4):
        pose_list.append(copy.deepcopy(pose))
        pose_list[-1].position.y += 0.05 * (i + 1)
    duration_list = [1.5, 2.5, 3, 4]
    robot_mg.send_cartesian_pos_trajectory(pose_list, duration_list, wait=True)
    # check velocity trajectory control
    twist_time = 5
    rot_speed = -0.05
    t_step = 0.008
    twist = Twist()
    for t in np.arange(0, twist_time + t_step, t_step):
        if t > (twist_time - 1):
            twist.linear.y = rot_speed * max(0, (twist_time - t))
        else:
            twist.linear.y = rot_speed * min(1, t)
        robot_mg.send_cartesian_vel_trajectory(twist)
        rospy.sleep(t_step)
    twist.linear.y = 0
    robot_mg.send_cartesian_vel_trajectory(twist)


if __name__ == "__main__":
    run()