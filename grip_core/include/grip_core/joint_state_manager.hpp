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

#ifndef GRIP_CORE_JOINT_STATE_MANAGER_H
#define GRIP_CORE_JOINT_STATE_MANAGER_H

#include <ros/ros.h>
#include <grip_core/AddJointState.h>
#include <grip_core/GetJointState.h>
#include <grip_core/ReinitManager.h>
#include <yaml-cpp/yaml.h>
#include <sensor_msgs/JointState.h>
#include <map>
#include <vector>
#include <string>
#include <sys/stat.h>

/**
 Class containing all methods and attributes required to create a joint state manager
 */
class JointStateManager
{
  public:
    // Constructor
    JointStateManager(ros::NodeHandle* nodehandle, std::string file_path);

    // Method loading the joint states already defined in a provided YAML file
    void load_joint_states_from_file(std::string filename);

  private:
    // ROS node handler allowing to create services
    ros::NodeHandle node_handler_;
    // Map gathering all named JointStates messages
    std::map<std::string, sensor_msgs::JointState> joint_states_map_;
    // Vector gathering all nameless JointStates
    std::vector<sensor_msgs::JointState> anonymous_joint_states_;
    // Service server for adding new joint state
    ros::ServiceServer add_joint_state_service_;
    // Service server for retrieving a joint state
    ros::ServiceServer retrieve_joint_state_service_;
    // Service server for reinitialising the manager
    ros::ServiceServer reinitialise_service_;
    // Indices used to keep track of the anonymous joint_state registered
    int anonymous_stored_index_;
    int anonymous_requested_index_;

    // Internal functions allowing to add and retrieve joint_states, that are going to be linked to the services
    bool _add_joint_state(grip_core::AddJointStateRequest& request,
                          grip_core::AddJointStateResponse& response);
    bool _get_joint_state(grip_core::GetJointStateRequest& request,
                          grip_core::GetJointStateResponse& response);
    bool _reinitialise(grip_core::ReinitManagerRequest& request,
                       grip_core::ReinitManagerResponse& response);
};

#endif  // GRIP_CORE_JOINT_STATE_MANAGER_H
