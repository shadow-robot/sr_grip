#!/usr/bin/env python3

# Copyright 2019, 2023 Shadow Robot Company Ltd.
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
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_euler


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
