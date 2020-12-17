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

#ifndef GRIP_CORE_STANDARDISED_GRASP_MANAGER_H
#define GRIP_CORE_STANDARDISED_GRASP_MANAGER_H

#include <ros/ros.h>
#include <grip_core/StandardisedGrasp.h>
#include <grip_core/AddStandardisedGrasp.h>
#include <grip_core/GetStandardisedGrasp.h>
#include <string>
#include <vector>
#include <map>

/**
 Class containing all methods and attributes required to create a grasp message manager
 */
class StandardisedGraspManager
{
  public:
    // Constructor
    explicit StandardisedGraspManager(ros::NodeHandle* nodehandle);

  private:
    // ROS node handler allowing to create services
    ros::NodeHandle node_handler_;
    // Map gathering all named grasps
    std::map<std::string, grip_core::StandardisedGrasp> grasps_map_;
    // Vector gathering all nameless grasps
    std::vector<grip_core::StandardisedGrasp> anonymous_grasps_;
    // Service server for adding new grasp
    ros::ServiceServer add_grasp_service_;
    // Service server for retrieving grasp
    ros::ServiceServer retrieve_grasp_service_;
    // Indices used to keep track of the anonymous grasps registered
    int anonymous_stored_index_;
    int anonymous_requested_index_;

    // Internal functions allowing to add and retrieve grasps, that are going to be linked to the services
    bool _add_grasp(grip_core::AddStandardisedGraspRequest& request,
                    grip_core::AddStandardisedGraspResponse& response);
    bool _get_grasp(grip_core::GetStandardisedGraspRequest& request,
                    grip_core::GetStandardisedGraspResponse& response);
};

#endif  // GRIP_CORE_STANDARDISED_GRASP_MANAGER_H
