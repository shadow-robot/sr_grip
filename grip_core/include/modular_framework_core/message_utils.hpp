/*
* Copyright 2019 Shadow Robot Company Ltd.
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

#ifndef GRIP_CORE_MESSAGE_UTILS_H
#define GRIP_CORE_MESSAGE_UTILS_H

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Dense>
#include <grip_core/StandardisedGrasp.h>
#include <string>
#include <vector>

geometry_msgs::PoseStamped generate_pose_stamped_message(std::string reference_frame_name, Eigen::Vector3f position,
                                                         Eigen::Vector3f orientation);
geometry_msgs::PoseStamped generate_pose_stamped_message(std::string reference_frame_name, Eigen::Vector3f position,
                                                         Eigen::Vector4f orientation);

grip_core::StandardisedGrasp get_standardised_grasp(
    std::vector<std::string> manipulator_joint_names, std::vector<double> pregrasp_joint_values,
    std::vector<double> grasp_joint_values, std::vector<double> postgrasp_joint_values,
    std::vector<double> squeezing_intensity, geometry_msgs::PoseStamped pregrasp_pose,
    geometry_msgs::PoseStamped grasp_pose, geometry_msgs::PoseStamped postgrasp_pose, double grasp_quality = 0.0,
    std::string grasp_id = "", std::string hand_id = "", std::string object_id = "");

sensor_msgs::JointState generate_joint_state_message(std::vector<std::string> joint_names, std::vector<double> position,
                                                     std::vector<double> velocity, std::vector<double> effort,
                                                     std::string reference_frame_name = "world");
#endif  // GRIP_CORE_MESSAGE_UTILS_H
