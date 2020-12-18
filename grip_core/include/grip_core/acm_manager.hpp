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

#ifndef GRIP_CORE_ACM_MANAGER_H
#define GRIP_CORE_ACM_MANAGER_H

#include <ros/ros.h>
#include <grip_core/SetInitACM.h>
#include <grip_core/UpdateACMEntry.h>
#include <grip_core/GetModifiedACM.h>
#include <moveit_msgs/AllowedCollisionMatrix.h>
#include <moveit_msgs/AllowedCollisionEntry.h>
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

  private:
    // ROS node handler allowing to create services
    ros::NodeHandle node_handler_;
    // Moveit message containing the initial ACM
    moveit_msgs::AllowedCollisionMatrix initial_acm_;
    // Moveit message containing an updated ACM based on the actions performed so far
    moveit_msgs::AllowedCollisionMatrix updated_acm_;
    // Boolean stating whether an initial ACM has been stored
    bool is_updated_;
    // Vector containing the indices of the manipulator's links
    std::vector<int> manipulator_links_indices_;
    // Number of entries in the initial ACM
    int initial_number_entries_;
    // Number of entries composing the ACM
    int number_entries_;
    // Service server for adding a new ACM matrix
    ros::ServiceServer set_init_acm_service_;
    // Service server for updating the ACM
    ros::ServiceServer update_acm_entry_service_;
    // Service server for optionally updating and getting the ACM to correspond to a certain collision state
    ros::ServiceServer get_modified_acm_service_;

    // Internal functions allowing to perform the different operations corresponding to the services
    bool _set_init_acm(grip_core::SetInitACMRequest& request,
                       grip_core::SetInitACMResponse& response);
    bool _update_acm_entry(grip_core::UpdateACMEntryRequest& request,
                           grip_core::UpdateACMEntryResponse& response);
    bool _get_modified_acm(grip_core::GetModifiedACMRequest& request,
                           grip_core::GetModifiedACMResponse& response);
};

#endif  // GRIP_CORE_ACM_MANAGER_H
