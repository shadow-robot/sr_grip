/*
* Copyright 2020, 2023 Shadow Robot Company Ltd.
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
