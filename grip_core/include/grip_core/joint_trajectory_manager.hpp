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

#ifndef GRIP_CORE_JOINT_TRAJECTORY_MANAGER_H
#define GRIP_CORE_JOINT_TRAJECTORY_MANAGER_H

#include <ros/ros.h>
#include <grip_core/AddJointTrajectory.h>
#include <grip_core/GetJointTrajectory.h>
#include <grip_core/GetJointState.h>
#include <yaml-cpp/yaml.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <map>
#include <vector>
#include <string>
#include <algorithm>

/**
 Class containing all methods and attributes required to create a joint trajectory manager
 */
class JointTrajectoryManager
{
  public:
    // Constructor
    JointTrajectoryManager(ros::NodeHandle* nodehandle, std::string file_path);

    // Method loading the joint trajectories already defined in a provided YAML file
    void load_trajectories_from_file(std::string filename);

  private:
    // ROS node handler allowing to create services
    ros::NodeHandle node_handler_;
    // Map gathering all named JointTrajectory messages
    std::map<std::string, trajectory_msgs::JointTrajectory> trajectories_map_;
    // Vector gathering all nameless JointTrajectory messages
    std::vector<trajectory_msgs::JointTrajectory> anonymous_trajectories_;
    // Service server for adding new joint trajectory
    ros::ServiceServer add_trajectory_service_;
    // Service server for retrieving a joint trajectory
    ros::ServiceServer retrieve_trajectory_service_;
    // Indices used to keep track of the anonymous joint trajectories registered
    int anonymous_stored_index_;
    int anonymous_requested_index_;

    // Internal functions allowing to add and retrieve joint trajectories, that are going to be linked to the services
    bool _add_trajectory(grip_core::AddJointTrajectoryRequest& request,
                         grip_core::AddJointTrajectoryResponse& response);
    bool _get_trajectory(grip_core::GetJointTrajectoryRequest& request,
                         grip_core::GetJointTrajectoryResponse& response);
};

#endif  // GRIP_CORE_JOINT_TRAJECTORY_MANAGER_H
