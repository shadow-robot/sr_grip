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

#ifndef GRIP_CORE_MOVEIt_PLAN_MANAGER_H
#define GRIP_CORE_MOVEIt_PLAN_MANAGER_H

#include <ros/ros.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <grip_core/AddMoveitPlan.h>
#include <grip_core/GetMoveitPlan.h>
#include <grip_core/ReinitManager.h>
#include <string>
#include <vector>
#include <map>
#include <sys/stat.h>

/**
 Class containing all methods and attributes required to create a pose stamped manager
 */
class MoveitPlanManager
{
  public:
    // Constructor
    explicit MoveitPlanManager(ros::NodeHandle* nodehandle);

  private:
    // ROS node handler allowing to create services
    ros::NodeHandle node_handler_;
    // Map gathering all named moveit plans
    std::map<std::string, moveit_msgs::RobotTrajectory> plans_map_;
    // Vector gathering all nameless moveit plans
    std::vector<moveit_msgs::RobotTrajectory> anonymous_plans_;
    // Service server for adding new moveit plan
    ros::ServiceServer add_plan_service_;
    // Service server for retrieving a moveit plan
    ros::ServiceServer retrieve_plan_service_;
    // Service server for reinitialising the manager
    ros::ServiceServer reinitialise_service_;
    // Indices used to keep track of the anonymous plans registered
    int anonymous_stored_index_;
    int anonymous_requested_index_;

    // Internal functions allowing to add and retrieve poses, that are going to be linked to the services
    bool _add_plan(grip_core::AddMoveitPlanRequest& request,
                   grip_core::AddMoveitPlanResponse& response);
    bool _get_plan(grip_core::GetMoveitPlanRequest& request,
                   grip_core::GetMoveitPlanResponse& response);
    bool _reinitialise(grip_core::ReinitManagerRequest& request,
                       grip_core::ReinitManagerResponse& response);
};

#endif  // GRIP_CORE_MOVEIt_PLAN_MANAGER_H
