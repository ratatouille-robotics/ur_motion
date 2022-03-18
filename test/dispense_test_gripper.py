#!/usr/bin/env python3

import sys
import rospy
import numpy as np

from geometry_msgs.msg import Pose, Twist

from motion.commander import RobotMoveGroup


def run():
    rospy.init_node("ur5e_move_test")
    robot_mg = RobotMoveGroup()
    # joints_ang = robot_mg.robot.get_current_state()
    # rospy.loginfo(joints_ang)
    # target_joint = [0, -np.pi / 2, np.pi / 2, 0, np.pi / 2, 0]

    # grip_msg = wrappMsg()
    # grip_msg.pos = 0 #open
    # grip_msg.speed = 100
    # grip_msg.force = 100
    # robot_mg.pub_gripper.publish(grip_msg)
    # robot_mg.go_to_joint_state(6*[0,])
    target_joint = [0.7332163453102112, -2.3039962253966273, 2.2220028082477015, -3.0586778126158656, -0.7337749640094202, 3.141390323638916] #taped container position pre grasp/HOME
    robot_mg.go_to_joint_state(target_joint)


    pose = Pose()
    pose = robot_mg.move_group.get_current_pose().pose
    pose.position.x = -0.01
    pose.position.y = 0.27
    pose.position.z = 0.61
    # pose.orientation.x = 0.014
    # pose.orientation.y = 0.7004
    # pose.orientation.z = 0.713
    # pose.orientation.w = -0.0249
    pose.orientation.x = 0.0
    pose.orientation.y = 0.7071068
    pose.orientation.z = 0.7071068
    pose.orientation.w = 0.0
    target_joint = [-2.4083376137304597, -2.156204122800058, 1.6418000793587364, 0.514398895563934, -5.549929513955047, -3.988344559502366e-06]
    robot_mg.go_to_joint_state(target_joint, cartesian_path=False)

    cpose = robot_mg.move_group.get_current_pose().pose
    # rospy.loginfo(cpose)
    # cpose.position.x = -0.6
    # cpose.position.y = -0.2
    cpose.position.y += 0.2
    # cpose.orientation.x = 0.5
    # cpose.orientation.y = -0.5
    # cpose.orientation.z = -0.5
    # cpose.orientation.w = 0.5
    # rospy.loginfo(cpose)
    robot_mg.go_to_pose_goal(cpose, cartesian_path=True, acc_scaling=0.1)


    # grip_msg.pos = 250 #close
    # grip_msg.speed = 100
    # grip_msg.force = 100
    # # rospy.loginfo(grip_msg)
    # robot_mg.pub_gripper.publish(grip_msg)
    # rospy.sleep(0.2)
    # while(robot_mg.object_det not in [2, 3] and not rospy.is_shutdown()): #add timeout
    #     # rospy.loginfo(robot_mg.object_det)
    #     rospy.sleep(0.1)
    # rospy.loginfo(robot_mg.object_det)
    # rospy.sleep(5)
    # rospy.loginfo(robot_mg.object_det)

    cpose = robot_mg.move_group.get_current_pose().pose
    # rospy.loginfo(cpose)
    # cpose.position.x = -0.6
    # cpose.position.y = -0.2
    cpose.position.z += 0.05
    cpose.position.y -= 0.2
    # cpose.orientation.x = 0.5
    # cpose.orientation.y = -0.5
    # cpose.orientation.z = -0.5
    # cpose.orientation.w = 0.5
    # rospy.loginfo(cpose)
    robot_mg.go_to_pose_goal(cpose, cartesian_path=True, acc_scaling=0.1)


    # cpose = robot_mg.move_group.get_current_pose().pose
    # # rospy.loginfo(cpose)
    # cpose.position.x = 0.25
    # cpose.position.y = 0.03
    # cpose.position.z = 0.66
    # cpose.orientation.x = 0.5
    # cpose.orientation.y = 0.5
    # cpose.orientation.z = 0.5
    # cpose.orientation.w = 0.5
    # # rospy.loginfo(cpose)
    # robot_mg.go_to_pose_goal(cpose, cartesian_path=True)

    joints_state = robot_mg.robot.get_current_state()
    joints_state = list(joints_state.joint_state.position)
    #-0.7521870771991175
    joints_state[0] -= np.pi/2
    # rospy.loginfo(joints_ang)
    robot_mg.go_to_joint_state(joints_state)

    cpose = robot_mg.move_group.get_current_pose().pose
    # rospy.loginfo(cpose)
    cpose.position.x = 0.34
    cpose.position.y = -0.08
    cpose.position.z = 0.39
    cpose.orientation.x = 0.5
    cpose.orientation.y = 0.5
    cpose.orientation.z = 0.5
    cpose.orientation.w = 0.5
    # rospy.loginfo(cpose)
    robot_mg.go_to_pose_goal(cpose, cartesian_path=False)

    joints_state = robot_mg.robot.get_current_state()
    joints_ang = list(joints_state.joint_state.position)
    #-0.7521870771991175
    joints_ang[-1] += np.pi/2
    # rospy.loginfo(joints_ang)
    robot_mg.go_to_joint_state(joints_ang)

    joints_state = robot_mg.robot.get_current_state()
    joints_ang = list(joints_state.joint_state.position)
    #-0.7521870771991175
    joints_ang[-1] -= np.pi/2
    # rospy.loginfo(joints_ang)
    robot_mg.go_to_joint_state(joints_ang)

    cpose = robot_mg.move_group.get_current_pose().pose
    # rospy.loginfo(cpose)
    cpose.position.x = 0.25
    cpose.position.y = 0.03
    cpose.position.z = 0.66
    cpose.orientation.x = 0.5
    cpose.orientation.y = 0.5
    cpose.orientation.z = 0.5
    cpose.orientation.w = 0.5
    # rospy.loginfo(cpose)
    robot_mg.go_to_pose_goal(cpose, cartesian_path=True)

    joints_state = robot_mg.robot.get_current_state()
    joints_ang = list(joints_state.joint_state.position)
    #-0.7521870771991175
    joints_ang[0] += np.pi/2
    # rospy.loginfo(joints_ang)
    robot_mg.go_to_joint_state(joints_ang)

    pose = Pose()
    pose = robot_mg.move_group.get_current_pose().pose
    pose.position.x = -0.01
    pose.position.y = 0.27
    pose.position.z = 0.66
    # pose.orientation.x = 0.014
    # pose.orientation.y = 0.7004
    # pose.orientation.z = 0.713
    # pose.orientation.w = -0.0249
    pose.orientation.x = 0.0
    pose.orientation.y = 0.7071068
    pose.orientation.z = 0.7071068
    pose.orientation.w = 0.0
    robot_mg.go_to_pose_goal(pose, cartesian_path=True)

    cpose = robot_mg.move_group.get_current_pose().pose
    # rospy.loginfo(cpose)
    # cpose.position.x = -0.6
    # cpose.position.y = -0.2
    cpose.position.y += 0.2
    cpose.position.z -= 0.05
    # cpose.orientation.x = 0.5
    # cpose.orientation.y = -0.5
    # cpose.orientation.z = -0.5
    # cpose.orientation.w = 0.5
    # rospy.loginfo(cpose)
    robot_mg.go_to_pose_goal(cpose, cartesian_path=True)

    # grip_msg = wrappMsg()
    # grip_msg.pos = 0 #open
    # grip_msg.speed = 100
    # grip_msg.force = 100
    # robot_mg.pub_gripper.publish(grip_msg)

    pose = Pose()
    pose = robot_mg.move_group.get_current_pose().pose
    pose.position.x = -0.01
    pose.position.y = 0.27
    pose.position.z = 0.61
    # pose.orientation.x = 0.014
    # pose.orientation.y = 0.7004
    # pose.orientation.z = 0.713
    # pose.orientation.w = -0.0249
    pose.orientation.x = 0.0
    pose.orientation.y = 0.7071068
    pose.orientation.z = 0.7071068
    pose.orientation.w = 0.0
    robot_mg.go_to_pose_goal(pose, cartesian_path=True)
    # pose_list = []
    # for i in range(4):
    #     pose_list.append(copy.deepcopy(pose))
    #     pose_list[-1].position.y += 0.05 * (i+1)
    # duration_list = [1.5, 2.5, 3, 4]
    # robot_mg.send_cartesian_pos_trajectory(pose_list, duration_list, wait=True)

    # twist = geometry_msgs.msg.Twist()
    # twist.linear.y = -0.05
    # robot_mg.send_cartesian_vel_trajectory(twist)
    # rospy.sleep(5)
    # twist.linear.y = 0
    # robot_mg.send_cartesian_vel_trajectory(twist)


if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        sys.exit(1)