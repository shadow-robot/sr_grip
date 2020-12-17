/*
* Copyright 2020 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation version 2 of the License.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <vector>
#include <string>
#include <grip_core/message_utils.hpp>

grip_core::StandardisedGrasp
get_standardised_grasp(std::vector<std::string> manipulator_joint_names, std::vector<double> pregrasp_joint_values,
                       std::vector<double> grasp_joint_values, std::vector<double> postgrasp_joint_values,
                       std::vector<double> torque_intensity, geometry_msgs::PoseStamped pregrasp_pose,
                       geometry_msgs::PoseStamped grasp_pose, geometry_msgs::PoseStamped postgrasp_pose,
                       double grasp_quality, std::string grasp_id, std::string hand_id, std::string object_id)
{
    grip_core::StandardisedGrasp standardised_grasp;

    standardised_grasp.grasp_id = grasp_id;
    standardised_grasp.hand_id = hand_id;
    standardised_grasp.object_id = object_id;

    standardised_grasp.torque_intensity.joint_names = manipulator_joint_names;
    standardised_grasp.torque_intensity.torque_intensity = torque_intensity;

    // Fill information related the grasp context (frame, object...)
    standardised_grasp.pregrasp.posture.header.stamp = ros::Time::now();
    standardised_grasp.pregrasp.posture.header.frame_id = "";
    standardised_grasp.pregrasp.posture.name = manipulator_joint_names;

    standardised_grasp.grasp.posture.header.stamp = ros::Time::now();
    standardised_grasp.grasp.posture.header.frame_id = "";
    standardised_grasp.grasp.posture.name = manipulator_joint_names;

    standardised_grasp.postgrasp.posture.header.stamp = ros::Time::now();
    standardised_grasp.postgrasp.posture.header.frame_id = "";
    standardised_grasp.postgrasp.posture.name = manipulator_joint_names;

    // Affect the joint values and pose to the pregrasp field
    standardised_grasp.pregrasp.posture.position = pregrasp_joint_values;
    standardised_grasp.pregrasp.pose = pregrasp_pose;

    // Affect the joint values and pose to the grasp field
    standardised_grasp.grasp.posture.position = grasp_joint_values;
    standardised_grasp.grasp.pose = grasp_pose;

    // Affect the joint values and pose to the postgrasp field
    standardised_grasp.postgrasp.posture.position = postgrasp_joint_values;
    standardised_grasp.postgrasp.pose = postgrasp_pose;

    // Specify the grasp quality
    standardised_grasp.grasp_quality = grasp_quality;

    return standardised_grasp;
}

geometry_msgs::PoseStamped generate_pose_stamped_message(std::string reference_frame_name, Eigen::Vector3f position,
                                                         Eigen::Vector3f orientation)
{
    geometry_msgs::PoseStamped message_to_return;
    tf2::Quaternion quaternion;
    message_to_return.header.frame_id = reference_frame_name;
    message_to_return.header.stamp = ros::Time::now();
    message_to_return.pose.position.x = position[0];
    message_to_return.pose.position.y = position[1];
    message_to_return.pose.position.z = position[2];
    quaternion.setRPY(orientation[0], orientation[1], orientation[2]);
    message_to_return.pose.orientation.x = quaternion.x();
    message_to_return.pose.orientation.y = quaternion.y();
    message_to_return.pose.orientation.z = quaternion.z();
    message_to_return.pose.orientation.w = quaternion.w();
    return message_to_return;
}

geometry_msgs::PoseStamped generate_pose_stamped_message(std::string reference_frame_name, Eigen::Vector3f position,
                                                         Eigen::Vector4f orientation)
{
    geometry_msgs::PoseStamped message_to_return;
    tf2::Quaternion quaternion;
    message_to_return.header.frame_id = reference_frame_name;
    message_to_return.header.stamp = ros::Time::now();
    message_to_return.pose.position.x = position[0];
    message_to_return.pose.position.y = position[1];
    message_to_return.pose.position.z = position[2];
    message_to_return.pose.orientation.x = orientation[0];
    message_to_return.pose.orientation.y = orientation[1];
    message_to_return.pose.orientation.z = orientation[2];
    message_to_return.pose.orientation.w = orientation[3];
    return message_to_return;
}

sensor_msgs::JointState generate_joint_state_message(std::vector<std::string> joint_names, std::vector<double> position,
                                                     std::vector<double> velocity, std::vector<double> effort,
                                                     std::string reference_frame_name)
{
    sensor_msgs::JointState message_to_return;
    message_to_return.header.frame_id = reference_frame_name;
    message_to_return.header.stamp = ros::Time::now();
    message_to_return.name = joint_names;
    message_to_return.position = position;
    message_to_return.velocity = velocity;
    message_to_return.effort = effort;
    return message_to_return;
}
