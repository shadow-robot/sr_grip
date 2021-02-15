#!/usr/bin/env python

# Copyright 2020 Shadow Robot Company Ltd.
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
