/*
* Copyright 2019, 2023 Shadow Robot Company Ltd.
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

#ifndef GRIP_CORE_POSE_STAMPED_MANAGER_H
#define GRIP_CORE_POSE_STAMPED_MANAGER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <grip_core/AddPoseStamped.h>
#include <grip_core/GetPoseStamped.h>
#include <grip_core/ReinitManager.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <map>
#include <sys/stat.h>

/**
 Class containing all methods and attributes required to create a pose stamped manager
 */
class PoseStampedManager
{
  public:
    // Constructor
    PoseStampedManager(ros::NodeHandle* nodehandle, std::string file_path);

    // Method loading the poses already defined in the corresponding YAML file
    void load_poses_from_file(std::string filename);

  private:
    // ROS node handler allowing to create services
    ros::NodeHandle node_handler_;
    // Map gathering all named poses
    std::map<std::string, geometry_msgs::PoseStamped> poses_map_;
    // Vector gathering all nameless poses
    std::vector<geometry_msgs::PoseStamped> anonymous_poses_;
    // Service server for adding new pose
    ros::ServiceServer add_pose_service_;
    // Service server for retrieving a pose
    ros::ServiceServer retrieve_pose_service_;
    // Service server for reinitialising the manager
    ros::ServiceServer reinitialise_service_;
    // Indices used to keep track of the anonymous poses registered
    int anonymous_stored_index_;
    int anonymous_requested_index_;

    // Internal functions allowing to add and retrieve poses, that are going to be linked to the services
    bool _add_pose(grip_core::AddPoseStampedRequest& request,
                   grip_core::AddPoseStampedResponse& response);
    bool _get_pose(grip_core::GetPoseStampedRequest& request,
                   grip_core::GetPoseStampedResponse& response);
    bool _reinitialise(grip_core::ReinitManagerRequest& request,
                      grip_core::ReinitManagerResponse& response);
};

#endif  // GRIP_CORE_POSE_STAMPED_MANAGER_H
