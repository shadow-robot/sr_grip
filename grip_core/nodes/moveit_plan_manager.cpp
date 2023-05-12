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

#include <grip_core/moveit_plan_manager.hpp>
#include <string>

/**
 Constructor of the class
 * @param nodehandler reference to a ros NodeHandle object
 */
MoveitPlanManager::MoveitPlanManager(ros::NodeHandle* nodehandler) : node_handler_(*nodehandler)
{
    // Define the values corresponding to the number of anonymous (nameless) moveit plans stored and requested
    anonymous_stored_index_ = 0;
    anonymous_requested_index_ = 0;
    // Initialize services
    add_plan_service_ = node_handler_.advertiseService("add_plan", &MoveitPlanManager::_add_plan, this);
    retrieve_plan_service_ = node_handler_.advertiseService("get_plan", &MoveitPlanManager::_get_plan, this);
    reinitialise_service_ =
        node_handler_.advertiseService("reinitialise_plan_manager", &MoveitPlanManager::_reinitialise, this);
    // Display a message stating that initialisation was a success
    ROS_INFO_STREAM("The moveit plan manager is ready");
}

/**
 Store a RobotTrajectory message (moveit plan)
 * @param  request  Object containing a field "plan_name" (string) that can be empty
                    and "moveit_plan" (moveit_msgs::RobotTrajectory)
 * @param  response Object containing a field "success" (boolean) stating whether the storing was successfull
 * @return          Boolean that should be always true in order to avoid having runtime errors on the client side
 */
bool MoveitPlanManager::_add_plan(grip_core::AddMoveitPlanRequest& request,
                                  grip_core::AddMoveitPlanResponse& response)
{
    // Extract the objects from the request
    std::string plan_name = request.plan_name;
    moveit_msgs::RobotTrajectory moveit_plan = request.moveit_plan;
    // If a grasp with the same name already exists, then display an info message stating that it will be overwritten
    if (plans_map_.count(plan_name) >= 1)
    {
        ROS_INFO_STREAM("The plan named " << plan_name << " already exists and will be overwritten");
    }
    // If the name is empty, then store the message in a list in order to keep the insertion order
    if (plan_name.empty())
    {
        anonymous_plans_.push_back(moveit_plan);
        // Increment the number of stored plans
        anonymous_stored_index_++;
    }
    // Otherwise just store it in the map
    else
    {
        plans_map_[plan_name] = moveit_plan;
    }
    // Fill the success field of the response to true
    response.success = true;
    return true;
}

/**
 Give access to an already stored RobotTrajectory message
 * @param  request  Object containing a field "plan_name" (string) depicting the name of the plan to retrieve.
                    If the field is empty then returns the oldest element of the anonymous plans
 * @param  response Object containing a field "moveit_plan" (moveit_msgs::RobotTrajectory) containing the requested plan
                    and a field "success" (boolean) stating whether the plan can be found
 * @return          Boolean that should be always true in order to avoid having runtime errors on the client side
 */
bool MoveitPlanManager::_get_plan(grip_core::GetMoveitPlanRequest& request,
                                  grip_core::GetMoveitPlanResponse& response)
{
    // Extract the information from the request
    std::string plan_name = request.plan_name;
    // If the name of the plan is empty, and the anonymous plan vector still has elements then return the oldest
    // element
    if ((plan_name.empty()) && (anonymous_requested_index_ < anonymous_stored_index_))
    {
        // Set the plan to the oldest element and delete it in order to save memory
        response.moveit_plan = anonymous_plans_[anonymous_requested_index_];
        anonymous_plans_.erase(anonymous_plans_.begin());
        // Increment the number of elements retrieved from the anonymous list
        anonymous_requested_index_++;
        // Success field of the response is set to true
        response.success = true;
        return true;
    }
    // If a greater number of request has been made than the number of stored anonymous plans then display an error
    // and set the success field to false
    else if ((anonymous_requested_index_ > anonymous_stored_index_) && (anonymous_stored_index_ != 0))
    {
        ROS_ERROR_STREAM("The number of requests has exceeded the number of plans saved!");
        response.success = false;
        return true;
    }
    // If a plan message with the requested name is in the map then access it and set it to the response
    else if (plans_map_.count(plan_name) == 1)
    {
        response.moveit_plan = plans_map_[plan_name];
        response.success = true;
        return true;
    }
    // Otherwise it means that the requested name does not exist, so display error message and set success to false
    else
    {
        ROS_ERROR_STREAM("The plan named " << plan_name << " does not seem to exist!");
        response.success = false;
        return true;
    }
}

/**
 Reinitialise the manager
 * @param  request  Object containing a field argument (string) that won't impact the behaviour of this function
 * @param  response Object containing a field "success" (boolean) stating whether the operation was successfull or not
 * @return          Boolean that should be always true in order to avoid having runtime errors on the client side
 */
bool MoveitPlanManager::_reinitialise(grip_core::ReinitManagerRequest& request,
                                      grip_core::ReinitManagerResponse& response)
{
    // Clear all the maps gathering the previously stored plans
    plans_map_.clear();
    anonymous_plans_.clear();
    // Reset the indices
    anonymous_stored_index_ = 0;
    anonymous_requested_index_ = 0;

    // Fill the success field of the response to true
    response.success = true;
    return true;
}


// If this file is called as a main, then create a node and launches the server
int main(int argc, char** argv)
{
    ros::init(argc, argv, "moveit_plan_manager_server");
    ros::NodeHandle node_handle;
    MoveitPlanManager moveit_plan_manager(&node_handle);
    ros::spin();
    return 0;
}
