#!/usr/bin/env python3
from __future__ import print_function
from ntpath import join
from six.moves import input

import sys
import copy
from typing import List
import numpy as np

import rospy
import actionlib
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from math import dist, fabs, cos

from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    CartesianTrajectoryPoint,
)

from moveit_commander.conversions import pose_to_list

from controller_manager_msgs.srv import (
    LoadControllerRequest,
    LoadController,
    ListControllers,
    ListControllersRequest,
    SwitchController,
    SwitchControllerRequest,
)

MOVEIT_CONTROLLER = "scaled_pos_joint_traj_controller"
POSE_CONTROLLER = "pose_based_cartesian_traj_controller"
TWIST_CONTROLLER = "twist_controller"

avail_controllers = [MOVEIT_CONTROLLER, POSE_CONTROLLER, TWIST_CONTROLLER]


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class UR5eMoveGroup(object):
    def __init__(self, verbose: bool = False) -> None:

        rospy.init_node("ur5e_move_group_interface")
        self._verbose = verbose
        # initialize `moveit_commander` and a `rospy` node
        moveit_commander.roscpp_initialize(sys.argv)
        self.timeout = rospy.Duration(5)

        # setup controller-manager ROS services
        self.switch_srv = rospy.ServiceProxy(
            "controller_manager/switch_controller", SwitchController
        )
        self.load_srv = rospy.ServiceProxy(
            "controller_manager/load_controller", LoadController
        )
        self.list_srv = rospy.ServiceProxy(
            "controller_manager/list_controllers", ListControllers
        )
        try:
            self.switch_srv.wait_for_service(self.timeout.to_sec())
        except rospy.exceptions.ROSException as err:
            rospy.logerr(
                "Could not reach controller switch service. Msg: {}".format(err)
            )
            sys.exit(-1)
        self._load_controllers()
        self._active_controller = None

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        self.robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for maintaining the robot's internal understanding of the environment
        self.scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface to a
        ## planning group (group of joints). This interface is used to plan and execute motions
        group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # Misc variables
        self.box_name = ""
        self.display_trajectory_publisher = display_trajectory_publisher
        # Get the name of the reference frame for this robot
        self.planning_frame = self.move_group.get_planning_frame()
        # Get the name of the end-effector link for this group:
        self.eef_link = self.move_group.get_end_effector_link()
        # Get a list of all the groups in the robot:
        self.group_names = self.robot.get_group_names()

        if self._verbose:
            print("============ Planning frame: %s" % self.planning_frame)
            print("============ End effector link: %s" % self.eef_link)
            print(
                "============ Available Planning Groups:", self.robot.get_group_names()
            )
            print("============ Printing robot state")
            print(self.robot.get_current_state())
            print("")

    def _load_controllers(self):
        srv = ListControllersRequest()
        resp = self.list_srv(srv)
        loaded_controllers = [controller.name for controller in resp.controller]

        for controller in avail_controllers:
            if controller not in loaded_controllers:
                srv = LoadControllerRequest()
                srv.name = controller
                self.load_srv(srv)

    def switch_controller(self, target_controller: str):
        """
        Activates the desired controller configuration
        
        Note: This class keeps track of the any controller start/stop made through
        the class methods. Changes made outside the class will not be reflected.
        """
        if self._active_controller == target_controller:
            return

        srv = ListControllersRequest()
        resp = self.list_srv(srv)
        stop_controllers = [
            controller.name
            for controller in resp.controller
            if controller.state == "running"
            and controller.name != "joint_state_controller"
        ]
        if target_controller not in stop_controllers:
            start_controller = [target_controller]
        else:
            start_controller = []
            stop_controllers.remove(target_controller)

        if len(start_controller) != 0 or len(stop_controllers) != 0:
            srv = SwitchControllerRequest()
            srv.stop_controllers = stop_controllers
            srv.start_controllers = start_controller
            srv.strictness = SwitchControllerRequest.STRICT
            srv.start_asap = True
            resp = self.switch_srv(srv)
            if not resp.ok:
                rospy.logerr(
                    f"Error when starting {target_controller} and stopping {stop_controllers}"
                )

        if target_controller == POSE_CONTROLLER:
            self.trajectory_client = actionlib.SimpleActionClient(
                 "{}/follow_cartesian_trajectory".format(target_controller),
                FollowCartesianTrajectoryAction,
            )
            # wait for the connection with the server to be established before sending goals
            if not self.trajectory_client.wait_for_server(self.timeout):
                raise "Could not reach cartesian controller action"

        elif target_controller == TWIST_CONTROLLER:
            self.twist_pub = rospy.Publisher(
                "/twist_controller/command", geometry_msgs.msg.Twist, queue_size=1
            )
            rospy.sleep(1)

        self._active_controller = target_controller

    def get_current_pose(self, stamped: bool = False):
        """
        position in metres. orientation in radians
        """
        if stamped:
            return self.move_group.get_current_pose()
        else:
            return self.move_group.get_current_pose().pose

    def go_to_joint_state(self, joint_goal: List[float], tol: float = 0.01):
        self.switch_controller(MOVEIT_CONTROLLER)
        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.move_group.go(joint_goal, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()
        return all_close(joint_goal, self.move_group.get_current_joint_values(), 0.01)

    def go_to_pose_goal(
        self, pose_goal: geometry_msgs.msg.Pose, cartesian_path=True, tol: float = 0.01
    ):
        self.switch_controller(MOVEIT_CONTROLLER)
        if not cartesian_path:
            self.move_group.set_pose_target(pose_goal)
            # Call the planner to compute the plan and execute it.
            plan = self.move_group.go(wait=True)
            # Calling `stop()` ensures that there is no residual movement
            self.move_group.stop()
            # It is always good to clear your targets after planning with poses.
            # Note: there is no equivalent function for clear_joint_value_targets()
            self.move_group.clear_pose_targets()
        else:
            # eef_step specifies the resolution of interpolation jump threshold in joint space
            (plan, fraction) = self.move_group.compute_cartesian_path(
                waypoints=[pose_goal], eef_step=0.01, jump_threshold=0.0
            )

            if fraction < 1.0:
                raise "Full trajectory could not be followed"
            ## **Note:** The robot's current joint state must be within some tolerance of the
            ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
            self.move_group.execute(plan, wait=True)

    def send_cartesian_pos_trajectory(
        self,
        pose_list: List[geometry_msgs.msg.Pose],
        t_durations: List[float],
        wait: bool = False,
    ):
        """
        Sends a cartesian position trajectory to the robot
        """
        self.switch_controller(POSE_CONTROLLER)

        goal = FollowCartesianTrajectoryGoal()

        for i, pose in enumerate(pose_list):
            point = CartesianTrajectoryPoint()
            point.pose = pose
            point.time_from_start = rospy.Duration(t_durations[i])
            goal.trajectory.points.append(point)

        self.trajectory_client.send_goal(goal)
        if wait:
            self.trajectory_client.wait_for_result()
            result = self.trajectory_client.get_result()
            rospy.loginfo(f"Trajectory execution finished in state {result.error_code}")

    def send_cartesian_vel_trajectory(self, twist: geometry_msgs.msg.Twist):
        """
        Sends a cartesian velocity set points to the robot
        """
        self.switch_controller(TWIST_CONTROLLER)
        self.twist_pub.publish(twist)


def run():
    rospy.sleep(10)
    robot_mg = UR5eMoveGroup()

    target_joint = [0, -np.pi / 2, np.pi / 2, 0, np.pi / 2, 0]
    robot_mg.go_to_joint_state(target_joint)

    pose = geometry_msgs.msg.Pose()
    pose.position.x = -0.6
    pose.position.y = -0.2
    pose.position.z = 0.6
    pose.orientation.x = 0.5
    pose.orientation.y = -0.5
    pose.orientation.z = -0.5
    pose.orientation.w = 0.5
    robot_mg.go_to_pose_goal(pose, cartesian_path=True)

    pose_list = []
    for i in range(4):
        pose_list.append(copy.deepcopy(pose))
        pose_list[-1].position.y += 0.05 * (i+1)
    duration_list = [1.5, 2.5, 3, 4]
    robot_mg.send_cartesian_pos_trajectory(pose_list, duration_list, wait=True)

    twist = geometry_msgs.msg.Twist()
    twist.linear.y = -0.05
    robot_mg.send_cartesian_vel_trajectory(twist)
    rospy.sleep(5)
    twist.linear.y = 0
    robot_mg.send_cartesian_vel_trajectory(twist)


if __name__ == "__main__":
    run()
