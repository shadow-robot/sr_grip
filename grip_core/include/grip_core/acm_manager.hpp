/*
* Copyright 2019, 2021 Shadow Robot Company Ltd.
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

#ifndef GRIP_CORE_ACM_MANAGER_H
#define GRIP_CORE_ACM_MANAGER_H

#include <ros/ros.h>
#include <grip_core/SetInitACM.h>
#include <grip_core/UpdateACMEntry.h>
#include <grip_core/ModifyACM.h>
#include <moveit_msgs/AllowedCollisionMatrix.h>
#include <moveit_msgs/AllowedCollisionEntry.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/PlanningSceneComponents.h>
#include <moveit_msgs/PlanningScene.h>
#include <map>
#include <vector>
#include <string>

/**
 Class containing all methods and attributes required to deal with Allowed Collision Matrices (ACM)
 */
class ACMManager
{
  public:
    // Constructor
    explicit ACMManager(ros::NodeHandle* nodehandle);

    // Function that initializes the manager's ACM from the current scene
    bool initialize_acm_from_scene();

  private:
    // ROS node handler allowing to create services
    ros::NodeHandle node_handler_;
    // Moveit message containing the initial ACM
    moveit_msgs::AllowedCollisionMatrix initial_acm_;
    // Moveit message containing the current ACM
    moveit_msgs::AllowedCollisionMatrix current_acm_;
    // Boolean stating whether an initial ACM has been stored
    bool is_updated_;
    // Map containing the indices of the robot's links
    std::map<std::string, int> robot_links_map_;
    // Map containing the indices of the links corresponding to objects
    std::map<std::string, int> object_links_map_;
    // Number of entries composing the current ACM
    int number_entries_;
    // Service server for setting a new ACM matrix as the initial
    ros::ServiceServer set_init_acm_service_;
    // Service server for updating the entries of the ACM
    ros::ServiceServer update_acm_entry_service_;
    // Service server for changing the collisions checks between different elements of the scene
    ros::ServiceServer modify_acm_service_;
    // Publisher allowing us to update the ACM that MoveIt! uses
    ros::Publisher publish_current_acm_;

    // Internal function that stores the indices of the input links
    void _set_robot_links_mapping(std::vector<std::string> entry_names);

    // Internal functions performing the different operations corresponding to the services
    bool _set_init_acm(grip_core::SetInitACMRequest& request,
                       grip_core::SetInitACMResponse& response);
    bool _update_acm_entry(grip_core::UpdateACMEntryRequest& request,
                           grip_core::UpdateACMEntryResponse& response);
    bool _modify_acm(grip_core::ModifyACMRequest& request,
                     grip_core::ModifyACMResponse& response);
};

#endif  // GRIP_CORE_ACM_MANAGER_H
