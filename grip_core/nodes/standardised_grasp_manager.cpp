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

#include <grip_core/standardised_grasp_manager.hpp>
#include <string>

/**
 Constructor of the class
 * @param nodehandler reference to a ros NodeHandle object
 */
StandardisedGraspManager::StandardisedGraspManager(ros::NodeHandle* nodehandler) : node_handler_(*nodehandler)
{
    // Define the values corresponding to the number of anonymous (nameless) grasps stored and requested
    anonymous_stored_index_ = 0;
    anonymous_requested_index_ = 0;
    // Initialize services
    add_grasp_service_ = node_handler_.advertiseService("add_grasp", &StandardisedGraspManager::_add_grasp, this);
    retrieve_grasp_service_ = node_handler_.advertiseService("get_grasp", &StandardisedGraspManager::_get_grasp, this);
    // Display a message stating that initialisation was a success
    ROS_INFO_STREAM("The standardised grasp manager is ready");
}

/**
 Store a StandardisedGrasp
 * @param  request  Object containing a field "grasp_name" (string) that can be empty
                    and "grasp_message" (StandardisedGrasp)
 * @param  response Object containing a field "success" (boolean) stating whether the storing was successfull
 * @return          Boolean that should be always true in order to avoid having runtime errors on the client side
 */
bool StandardisedGraspManager::_add_grasp(grip_core::AddStandardisedGraspRequest& request,
                                          grip_core::AddStandardisedGraspResponse& response)
{
    // Extract the objects from the request
    std::string grasp_name = request.grasp_name;
    grip_core::StandardisedGrasp grasp_message = request.grasp_message;
    // If a grasp with the same name already exists, then display an info message stating that it will be overwritten
    if (grasps_map_.count(grasp_name) >= 1)
    {
        ROS_INFO_STREAM("The grasp named " << grasp_name << " already exists and will be overwritten");
    }
    // If the name is empty, then store the message in a list in order to keep the insertion order
    if (grasp_name.empty())
    {
        anonymous_grasps_.push_back(grasp_message);
        // Increment the number of stored grasps
        anonymous_stored_index_++;
    }
    // If the field grasp_id of the message to store is empty and grasp_name is not empty, set the name of the message
    // and store it to a map with the name as key in order to look up for it
    else if (grasp_message.grasp_id.empty())
    {
        grasp_message.grasp_id = grasp_name;
        grasps_map_[grasp_name] = grasp_message;
    }
    // Otherwise just store it in the map
    else
    {
        grasps_map_[grasp_name] = grasp_message;
    }
    // Fill the success field of the response to true
    response.success = true;
    return true;
}

/**
 Give access to an already stored StandardisedGrasp message
 * @param  request  Object containing a field "grasp_name" (string) depicting the name of the grasp to retrieve.
                    If the field is empty then returns the oldest element of the anonymous grasps
 * @param  response Object containing a field "grasp_message" (StandardisedGrasp) containing the requested grasp
                    and a field "success" (boolean) stating whether the grasp can be found
 * @return          Boolean that should be always true in order to avoid having runtime errors on the client side
 */
bool StandardisedGraspManager::_get_grasp(grip_core::GetStandardisedGraspRequest& request,
                                          grip_core::GetStandardisedGraspResponse& response)
{
    // Extract the information from the anonymous_requested_index_
    std::string grasp_name = request.grasp_name;
    // If the name of the grasp is empty, and the anonymous grasp vector still has elements then return the oldest
    // element
    if ((grasp_name.empty()) && (anonymous_requested_index_ < anonymous_stored_index_))
    {
        // Set the grasp_message to the oldest element and delete it in order to save memory
        response.grasp_message = anonymous_grasps_[anonymous_requested_index_];
        anonymous_grasps_.erase(anonymous_grasps_.begin());
        // Increment the number of elements retrieved from the anonymous list
        anonymous_requested_index_++;
        // Success field of the response is set to true
        response.success = true;
        return true;
    }
    // If a greater number of request has been made than the number of stored anonymous grasps then display an error
    // and set the success field to false
    else if ((anonymous_requested_index_ > anonymous_stored_index_) && (anonymous_stored_index_ != 0))
    {
        ROS_ERROR_STREAM("The number of requests has exceeded the number of grasps saved!");
        response.success = false;
        return true;
    }
    // If a grasp message with the requested name is in the map then access it and set it to the response
    else if (grasps_map_.count(grasp_name) == 1)
    {
        response.grasp_message = grasps_map_[grasp_name];
        response.success = true;
        return true;
    }
    // Otherwise it means that the requested name does not exist, so display error message and set success to false
    else
    {
        ROS_ERROR_STREAM("The grasp named " << grasp_name << " does not seem to exist!");
        response.success = false;
        return true;
    }
}

// If this file is called as a main, then create a node and launches the server
int main(int argc, char** argv)
{
    ros::init(argc, argv, "standardised_grasp_manager_server");
    ros::NodeHandle node_handle;
    StandardisedGraspManager standardised_grasp_manager(&node_handle);
    ros::spin();
    return 0;
}
