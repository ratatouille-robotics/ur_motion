import copy
from typing import List
from geometry_msgs.msg import Pose, Vector3, Quaternion


def offset_pose(pose: Pose, trans_offset: List = None, rot_offset: List = None) -> Pose:
    pose = copy.deepcopy(pose)
    if trans_offset is not None:
        assert len(trans_offset) == 3, "trans_offset should be of length 3"
        pose.position.x += trans_offset[0]
        pose.position.y += trans_offset[1]
        pose.position.z += trans_offset[2]
    if rot_offset is not None:
        raise NotImplementedError
    return pose

def offset_pose_relative(poseA: Pose, poseB: Pose, cartesianOnly: bool = True) -> Pose:
    pose = Pose()
    pose.position.x = poseA.position.x - poseB.position.x
    pose.position.y = poseA.position.y - poseB.position.y
    pose.position.z = poseA.position.z - poseB.position.z
    if not cartesianOnly:
        return NotImplementedError
    return pose

def offset_joint(joint_state: List, joint_offset: List) -> List:
    assert len(joint_state) == len(
        joint_offset
    ), "joint_state and joint_offset should be of same length"

    joint_state = copy.deepcopy(joint_state)
    for i in range(len(joint_state)):
        joint_state[i] += joint_offset[i]
    return joint_state


def make_pose(position: List = [0, 0, 0], orientation: List = [0, 0, 0, 0]) -> Pose:
    assert len(position) == 3, "Incorrect dimensions for position"
    assert len(orientation) == 4, "Incorrect dimensions for orientation"
    return Pose(Vector3(*position), Quaternion(*orientation))
