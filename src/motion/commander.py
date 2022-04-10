#!/usr/bin/env python3
from __future__ import print_function

import sys
import math
from typing import List, Union
import numpy as np
import time

import rospy
import actionlib
import moveit_commander
import std_msgs.msg
import moveit_msgs.msg as mi_msg

from geometry_msgs.msg import Pose, PoseStamped, Twist

from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    CartesianTrajectoryPoint,
)

from moveit.core.kinematic_constraints import constructGoalConstraints
from moveit_commander.conversions import pose_to_list
from moveit_msgs.srv import GetMotionPlan
from robotiq_urcap_control.msg import wrapper as gripperMsg
from robotiq_urcap_control.msg import Robotiq2FGripper_robot_input as gripperStateMsg

from controller_manager_msgs.srv import (
    LoadControllerRequest,
    LoadController,
    ListControllers,
    ListControllersRequest,
    SwitchController,
    SwitchControllerRequest,
)
from enum import Enum, IntEnum


## Uncomment to use with Gazebo
# MOVEIT_CONTROLLER = "pos_joint_traj_controller"
# POSE_CONTROLLER = "pose_based_cartesian_traj_controller"
# TWIST_CONTROLLER = "twist_controller"
# avail_controllers = [MOVEIT_CONTROLLER]


class Controllers(str, Enum):
    MOVEIT = "scaled_pos_joint_traj_controller"
    POSE = "pose_based_cartesian_traj_controller"
    TWIST = "twist_controller"
    JOINT_STATE = "joint_state_controller"
    FORCE = "force_torque_sensor_controller"


class GripperStates(IntEnum):
    OPEN = 0
    CLOSE = 255


class GripperMotionStates(IntEnum):
    MOVING = 0
    STOPPED_BARRIER_OUTSIDE = 1
    STOPPED_BARRIER_INSIDE = 2
    REACHED = 3


_available_controllers = [
    Controllers.MOVEIT,
    Controllers.POSE,
    Controllers.TWIST,
]


def _joints_close(goal: List, actual: List, tolerance: float):
    """
    Check if the joint values in two lists are within a tolerance of each other
    """
    for index in range(len(goal)):
        if abs(actual[index] - goal[index]) > tolerance:
            err = goal[index] - actual[index]
            rospy.logwarn(
                f"Joint error ({err} rad) is greater than the tolerance ({tolerance} rad)"
            )
            return False
    return True


def _poses_close(
    goal: Union[Pose, PoseStamped],
    actual: Union[Pose, PoseStamped],
    pos_tolerance: float,
    orient_tolerance: float,
):
    """
    Check if the actual and goal poses are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    """

    goal = goal.pose if isinstance(goal, PoseStamped) else goal
    actual = actual.pose if isinstance(actual, PoseStamped) else actual

    x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
    x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
    # Euclidean distance
    d = math.dist((x1, y1, z1), (x0, y0, z0))
    # angle between orientations
    cos_phi_half = math.fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
    phi = 2 * np.arccos(cos_phi_half)
    result = True
    if d > pos_tolerance:
        rospy.logwarn(
            f"Position error ({d} m) is greater than the tolerance ({pos_tolerance} m)"
        )
        result = False
    if phi > orient_tolerance:
        rospy.logwarn(
            f"Orientation error ({phi} rad) is greater than the tolerance ({orient_tolerance} rad)"
        )
        result = False
    return result


class RobotMoveGroup(object):

    JOINTS = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]

    GRIPPER_SPEED_DEFAULT = 100
    GRIPPER_FORCE_DEFAULT = 100

    def __init__(self, verbose: bool = False) -> None:

        self._verbose = verbose
        # initialize `moveit_commander` and a `rospy` node
        moveit_commander.roscpp_initialize(sys.argv)
        # set a default timeout threshold non-motion requests
        self.timeout = rospy.Duration(5)

        self.gripper_pub = rospy.Publisher("/move_gripper", gripperMsg, queue_size=10)
        rospy.Subscriber(
            "/Robotiq2FGripperRobotInput", gripperStateMsg, self.update_gripper_state
        )
        self.gripper_motion_state = GripperMotionStates.REACHED

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
        # setup moveit publishers, services, and actions
        self.get_plan = rospy.ServiceProxy("/plan_kinematic_path", GetMotionPlan)
        self.execute_plan = actionlib.SimpleActionClient(
            "/execute_trajectory", mi_msg.ExecuteTrajectoryAction
        )

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
        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        self.display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path", mi_msg.DisplayTrajectory, queue_size=20
        )

        # Get the name of the reference frame for this robot
        self.planning_frame = self.move_group.get_planning_frame()
        # Get the name of the end-effector link for this group:
        self.eef_frame = self.move_group.get_end_effector_link()
        # Get a list of all the groups in the robot:
        self.group_names = self.robot.get_group_names()

        if self._verbose:
            print("============ Planning frame: %s" % self.planning_frame)
            print("============ End effector link: %s" % self.eef_frame)
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

        try:
            self.load_srv.wait_for_service(self.timeout.to_sec())
        except rospy.exceptions.ROSException as err:
            rospy.logerr("Could not reach Load Controller service. Msg: {}".format(err))
            sys.exit(-1)

        for controller in _available_controllers:
            print(f"Controller : {controller.name}")
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

        self.move_group.stop()
        rospy.sleep(0.1)

        srv = ListControllersRequest()
        resp = self.list_srv(srv)
        stop_controllers = [
            controller.name
            for controller in resp.controller
            if controller.state == "running"
            and controller.name not in [Controllers.JOINT_STATE, Controllers.FORCE]
        ]
        if target_controller not in stop_controllers:
            start_controller = [target_controller]
        else:
            start_controller = []
            stop_controllers.remove(target_controller)

        if len(start_controller) != 0 or len(stop_controllers) != 0:
            try:
                self.switch_srv.wait_for_service(self.timeout.to_sec())
            except rospy.exceptions.ROSException as err:
                rospy.logerr(
                    "Could not reach Switch Controller service. Msg: {}".format(err)
                )
                sys.exit(-1)

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

        if target_controller == Controllers.POSE:
            self.trajectory_client = actionlib.SimpleActionClient(
                "{}/follow_cartesian_trajectory".format(target_controller),
                FollowCartesianTrajectoryAction,
            )
            # wait for the connection with the server to be established before sending goals
            if not self.trajectory_client.wait_for_server(self.timeout):
                raise "Could not reach cartesian controller action"

        elif target_controller == Controllers.TWIST:
            self.twist_pub = rospy.Publisher(
                "/twist_controller/command", Twist, queue_size=1
            )
            rospy.sleep(0.1)

        self._active_controller = target_controller

    def get_current_pose(self, stamped: bool = False) -> Union[Pose, PoseStamped]:
        """
        position in metres. orientation in radians
        """
        if stamped:
            return self.move_group.get_current_pose()
        else:
            return self.move_group.get_current_pose().pose

    def get_current_joints(self, in_degrees: bool = False) -> List:
        """
        joint angle values in radians (or) degrees
        """
        joint_state = self.move_group.get_current_joint_values()
        if in_degrees:
            joint_state = [np.rad2deg(joint) for joint in joint_state]
        return joint_state

    def go_to_joint_state(
        self,
        joint_goal: List[float],
        cartesian_path: bool = False,
        tolerance: float = 0.001,
        velocity_scaling: float = 0.2,
        acc_scaling: float = 0.2,
        wait: bool = True,
    ) -> bool:
        self.switch_controller(Controllers.MOVEIT)
        # Check if MoveIt planner is running
        rospy.wait_for_service("/plan_kinematic_path", self.timeout)
        # Create a motion planning request with all necessary goals and constraints
        mp_req = mi_msg.MotionPlanRequest()
        mp_req.pipeline_id = "pilz_industrial_motion_planner"
        mp_req.planner_id = "LIN" if cartesian_path else "PTP"
        mp_req.group_name = "manipulator"
        mp_req.num_planning_attempts = 1

        constraints = mi_msg.Constraints()
        for joint_no in range(len(self.JOINTS)):
            constraints.joint_constraints.append(mi_msg.JointConstraint())
            constraints.joint_constraints[-1].joint_name = self.JOINTS[joint_no]
            constraints.joint_constraints[-1].position = joint_goal[joint_no]
            constraints.joint_constraints[-1].tolerance_above = tolerance
            constraints.joint_constraints[-1].tolerance_below = tolerance

        mp_req.goal_constraints.append(constraints)
        mp_req.max_velocity_scaling_factor = velocity_scaling
        mp_req.max_acceleration_scaling_factor = acc_scaling

        mp_res = self.get_plan(mp_req).motion_plan_response
        if mp_res.error_code.val != mp_res.error_code.SUCCESS:
            rospy.logerr(
                "Planner failed to generate a valid plan to the goal joint_state"
            )
            return False
        if (
            len(mp_res.trajectory.joint_trajectory.points) > 1
            and mp_res.trajectory.joint_trajectory.points[-1].time_from_start
            == mp_res.trajectory.joint_trajectory.points[-2].time_from_start
        ):
            mp_res.trajectory.joint_trajectory.points.pop(-2)
            rospy.logwarn(
                "Duplicate time stamp in the planned trajectory. Second last way-point was removed."
            )
        goal = mi_msg.ExecuteTrajectoryGoal(trajectory=mp_res.trajectory)
        self.execute_plan.wait_for_server()
        self.execute_plan.send_goal(goal)
        if wait:
            self.execute_plan.wait_for_result()
            # Calling ``stop()`` ensures that there is no residual movement
            self.move_group.stop()
            return _joints_close(joint_goal, self.get_current_joints(), tolerance)
        return True

    def go_to_pose_goal(
        self,
        pose_goal: Pose,
        cartesian_path=True,
        pos_tolerance: float = 0.001,
        orient_tolerance: float = 0.01,
        velocity_scaling: float = 0.2,
        acc_scaling: float = 0.2,
        wait: bool = True,
    ) -> bool:
        self.switch_controller(Controllers.MOVEIT)
        # Check if MoveIt planner is running
        rospy.wait_for_service("/plan_kinematic_path", self.timeout)
        # Create a motion planning request with all necessary goals and constraints
        mp_req = mi_msg.MotionPlanRequest()
        mp_req.pipeline_id = "pilz_industrial_motion_planner"
        mp_req.planner_id = "LIN" if cartesian_path else "PTP"
        mp_req.group_name = "manipulator"
        mp_req.num_planning_attempts = 1

        mp_req_pose_goal = PoseStamped(
            header=std_msgs.msg.Header(frame_id=self.planning_frame), pose=pose_goal
        )

        constraints = constructGoalConstraints(
            self.eef_frame, mp_req_pose_goal, pos_tolerance, orient_tolerance
        )
        mp_req.goal_constraints.append(constraints)
        mp_req.max_velocity_scaling_factor = velocity_scaling
        mp_req.max_acceleration_scaling_factor = acc_scaling

        mp_res = self.get_plan(mp_req).motion_plan_response
        if mp_res.error_code.val != mp_res.error_code.SUCCESS:
            rospy.logerr("Planner failed to generate a valid plan to the goal pose")
            return False
        goal = mi_msg.ExecuteTrajectoryGoal(trajectory=mp_res.trajectory)
        self.execute_plan.wait_for_server()
        self.execute_plan.send_goal(goal)
        if wait:
            self.execute_plan.wait_for_result()
            # Calling ``stop()`` ensures that there is no residual movement
            self.move_group.stop()
            return _poses_close(
                pose_goal, self.get_current_pose(), pos_tolerance, orient_tolerance
            )
        return True

    def send_cartesian_pos_trajectory(
        self, pose_list: List[Pose], t_durations: List[float], wait: bool = False
    ):
        """
        Sends a cartesian position trajectory to the robot
        """
        self.switch_controller(Controllers.POSE)

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

    def send_cartesian_vel_trajectory(self, twist: Twist):
        """
        Sends a cartesian velocity set points to the robot
        """
        self.switch_controller(Controllers.TWIST)
        self.twist_pub.publish(twist)

    def update_gripper_state(self, data) -> None:
        """
        Updates self.gripper_motion_state to reflect current motion state of gripper.
        """
        self.gripper_motion_state = data.gOBJ

    def go_to_gripper_state(self, target_state: int, wait=False) -> bool:
        """
        Moves gripper to given state.
        """
        assert GripperStates.OPEN <= target_state <= GripperStates.CLOSE
        if self._verbose:
            print(f"Moving gripper to state {target_state}")
        gripper_message = gripperMsg()
        gripper_message.pos = target_state
        gripper_message.speed = 100
        gripper_message.force = 100
        self.gripper_pub.publish(gripper_message)
        start = time.time()
        if wait:
            while self.gripper_motion_state == GripperMotionStates.MOVING:
                rospy.sleep(0.1)
                if time.time() > start + 10:
                    return False
        return True

    def open_gripper(self, wait=False) -> bool:
        """
        Opens gripper.
        """
        if self._verbose:
            print("Opening gripper")
        return self.go_to_gripper_state(GripperStates.OPEN, wait)

    def close_gripper(self, wait=False) -> bool:
        """
        Closes gripper.
        """
        if self._verbose:
            print("Closing gripper")
        return self.go_to_gripper_state(GripperStates.CLOSE, wait)
