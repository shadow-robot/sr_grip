#!/usr/bin/env python3

# Copyright 2020, 2023 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

import rospy
from grip_core.srv import GetJointState, GetPoseStamped, GetMoveitPlan, GetJointTrajectory
from grip_core.srv import AddJointState, AddPoseStamped, AddMoveitPlan, AddJointTrajectory
from grip_core.srv import ReinitManager
from grip_api.utils.files_specifics import ALL_MANAGER_TYPE_CHOICE


def get_retrieve_service_from_type(message_type):
    """
        Return the service allowing to retrieve messages of type message_type from its manager

        @param message_type: Type (i.e. string) of a message (must be either joint state, pose, plan or trajectory)
        @return: Client to retrieve messages if message_type is invalid, oterwise None
    """
    if message_type not in ALL_MANAGER_TYPE_CHOICE[1:]:
        return None

    retriever_service = None
    # If we need to lookup a joint state in the manager
    if message_type == "joint state":
        retriever_service = rospy.ServiceProxy("get_joint_state", GetJointState)
    # Same thing for the pose
    elif message_type == "pose":
        retriever_service = rospy.ServiceProxy("get_pose", GetPoseStamped)
    # Same thing for the plan
    elif message_type == "plan":
        retriever_service = rospy.ServiceProxy("get_plan", GetMoveitPlan)
    # Same thing for the joint trajectory
    elif message_type == "trajectory":
        retriever_service = rospy.ServiceProxy("get_trajectory", GetJointTrajectory)

    return retriever_service


def get_add_service_from_type(message_type):
    """
        Return the service allowing to add messages of type message_type to its manager

        @param message_type: Type (i.e. string) of a message (must be either joint state, pose, plan or trajectory)
        @return: Client to add messages if message_type is invalid, oterwise None
    """
    if message_type not in ALL_MANAGER_TYPE_CHOICE[1:]:
        return None

    adder_service = None
    # If we need to add a joint state in the manager
    if message_type == "joint state":
        adder_service = rospy.ServiceProxy("add_joint_state", AddJointState)
    # Same thing for the pose
    elif message_type == "pose":
        adder_service = rospy.ServiceProxy("add_pose", AddPoseStamped)
    # Same thing for the plan
    elif message_type == "plan":
        adder_service = rospy.ServiceProxy("add_plan", AddMoveitPlan)
    # Same thing for the joint trajectory
    elif message_type == "trajectory":
        adder_service = rospy.ServiceProxy("add_trajectory", AddJointTrajectory)

    return adder_service


def reinitialise_manager(manager_type, file_path=""):
    """
        Reinitialise a given manager, i.e. clear it and fills it with some messages defined in a file

        @param manager_type: Name of the manager. Valid strings are "joint state", "pose", "robot pose", "plan" and
                             "trajectory"
        @param file_path: Optional path to a file that defines some messages to add to the manager once cleared
    """
    # Make sure the required manager is valid
    valid_manager_type = ALL_MANAGER_TYPE_CHOICE[1:] + ["robot pose"]
    if manager_type not in (valid_manager_type):
        rospy.logwarn("Invalid manager type. Valid manager types are: {}".format(", ".join(valid_manager_type)))
        return None
    # Set teh proper service name depending on the input type
    if manager_type == "joint state":
        service_name = "reinitialise_joint_state_manager"
    elif manager_type == "pose":
        service_name = "reinitialise_pose_manager"
    elif manager_type == "robot pose":
        service_name = "reinitialise_robot_pose_manager"
    elif manager_type == "plan":
        service_name = "reinitialise_plan_manager"
    elif manager_type == "trajectory":
        service_name = "reinitialise_trajectory_manager"
    # Wait for the service and stop after waiting for 1s
    try:
        rospy.wait_for_service(service_name, 1)
    except rospy.ROSException:
        rospy.logwarn("Could not find service named {}".format(service_name))
        return None
    # If the service is found then calls it
    service = rospy.ServiceProxy(service_name, ReinitManager)
    return service(file_path)


def reinitialise_managers(joint_state_file, poses_file, trajectory_file):
    """
        Reinitialise all the managers

        @param joint_state_file: Path to a file that defines the joint states to add to its manager once cleared
        @param poses_file: Path to a file that defines the poses and robot  poses to add to their managers once cleared
        @param trajectory_file: Path to a file that defines the trajecttories to add to its manager once cleared
    """
    # Call the reinitialise_manager for all the managers
    joint_state_result = reinitialise_manager("joint state", joint_state_file)
    plan_result = reinitialise_manager("plan")
    pose_result = reinitialise_manager("pose", poses_file)
    robot_pose_result = reinitialise_manager("robot pose", poses_file)
    trajectory_result = reinitialise_manager("trajectory", trajectory_file)
    # If any is not found, return False
    if any(x is None for x in (joint_state_result, plan_result, pose_result, robot_pose_result, trajectory_result)):
        return False
    # Return True only if all the managers were successfully reinitialised
    return all(x.success for x in (joint_state_result, plan_result, pose_result, robot_pose_result, trajectory_result))
