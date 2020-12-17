#!/usr/bin/env python

# Copyright 2019 Shadow Robot Company Ltd.
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

from grip_core.msg import StandardisedGrasp
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_euler


def get_standardised_grasp(manipulator_joint_names, pregrasp_joint_values, grasp_joint_values, postgrasp_joint_values,
                           torque_intensity, pregrasp_pose, grasp_pose, postgrasp_pose, grasp_quality=0,
                           grasp_id="", hand_id="", object_id=""):
    """
        Given all required information, return a StandardisedGrasp message compatible with the framework

        @param manipulator_joint_names: List containing the name of each joint, e.g ["joint1", "joint2", ...]
        @param pregrasp_joint_values: List of floats containing the position of each joint for the pregrasp state
        @param grasp_joint_values: List of floats containing the position of each joint for the grasp state
        @param postgrasp_joint_values: List of floats containing the position of each joint for the postgrasp state
        @param torque_intensity: List of torque intensity (percentage) to apply with each joint, e.g [0.4, 0.2, ...]
        @param pregrasp_pose: PoseStamped message of the end-effector corresponding to the pregrasp state
        @param grasp_pose: PoseStamped message of the end-effector corresponding to the grasp state
        @param postgrasp_pose: PoseStamped message of the end-effector corresponding to the postgrasp state
        @param grasp_quality: Float evaluating how good the grasp should be.
        @param grasp_id: String specifying the name of the grasp.
        @param hand_id: String specifying the name of the hand for which the grasp has been generated.
        @param object_id: String specifying the name of the object that has been used to generate the grasp.

        @return: StandardisedGrasp message

    """
    standardised_grasp = StandardisedGrasp()

    standardised_grasp.grasp_id = grasp_id
    standardised_grasp.hand_id = hand_id
    standardised_grasp.object_id = object_id

    standardised_grasp.torque_intensity.joint_names = manipulator_joint_names
    standardised_grasp.torque_intensity.torque_intensity = torque_intensity

    # Fill information related the grasp context (frame, object...)
    standardised_grasp.pregrasp.posture.header.stamp = rospy.Time.now()
    standardised_grasp.pregrasp.posture.header.frame_id = ""
    standardised_grasp.pregrasp.posture.name = manipulator_joint_names

    standardised_grasp.grasp.posture.header.stamp = rospy.Time.now()
    standardised_grasp.grasp.posture.header.frame_id = ""
    standardised_grasp.grasp.posture.name = manipulator_joint_names

    standardised_grasp.postgrasp.posture.header.stamp = rospy.Time.now()
    standardised_grasp.postgrasp.posture.header.frame_id = ""
    standardised_grasp.postgrasp.posture.name = manipulator_joint_names

    # Add the pregrasp posture to the message
    standardised_grasp.pregrasp.posture.position = pregrasp_joint_values
    standardised_grasp.pregrasp.pose = pregrasp_pose

    # Add the pregrasp posture to the message
    standardised_grasp.grasp.posture.position = grasp_joint_values
    standardised_grasp.grasp.pose = grasp_pose

    standardised_grasp.postgrasp.posture.position = postgrasp_joint_values
    standardised_grasp.postgrasp.pose = postgrasp_pose

    standardised_grasp.grasp_quality = grasp_quality

    return standardised_grasp


def generate_pose_stamped_message(reference_frame_name, position, orientation):
    """
        Given an orientation and a position provided in a given reference frame
        return the corresponding PoseStamped message

        @param reference_frame_name: String containing the name of the reference frame
        @param position: List or tuple containing the position (x, y, z)
        @param orientation: List or tuple containing the orientation in RPY system (radians)

        @return: PoseStamped message
    """
    joint_state_to_return = PoseStamped()
    joint_state_to_return.header.frame_id = reference_frame_name
    joint_state_to_return.header.stamp = rospy.Time.now()
    joint_state_to_return.pose.position.x = position[0]
    joint_state_to_return.pose.position.y = position[1]
    joint_state_to_return.pose.position.z = position[2]
    quaternion = quaternion_from_euler(orientation[0], orientation[1], orientation[2])
    joint_state_to_return.pose.orientation.x = quaternion[0]
    joint_state_to_return.pose.orientation.y = quaternion[1]
    joint_state_to_return.pose.orientation.z = quaternion[2]
    joint_state_to_return.pose.orientation.w = quaternion[3]
    return joint_state_to_return


def generate_joint_state_message(joint_names, position, velocity=[], effort=[], reference_frame_name="world"):
    """
        Given the position, velocity (optional) and effort (optional) given in a reference frame
        return the corresponding JointState message

        @param joint_names: List of string stating which joints are refered to
        @param position: List or tuple containing the position (rad or m) of the joints defined in joint_names
        @param position: List or tuple containing the velocity of the joints defined in joint_names
        @param position: List or tuple containing the effort of the joints defined in joint_names
        @param reference_frame_name: String containing the name of the reference frame

        @return: JointState message
    """
    joint_state_to_return = JointState()
    joint_state_to_return.header.frame_id = reference_frame_name
    joint_state_to_return.header.stamp = rospy.Time.now()
    joint_state_to_return.name = joint_names
    joint_state_to_return.position = position
    joint_state_to_return.velocity = velocity
    joint_state_to_return.effort = effort
    return joint_state_to_return
