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

#include <grip_core/joint_state_manager.hpp>
#include <string>

/**
 Constructor of the class from an optional YAML file containing predefined joint states
 * @param nodehandler reference to a ros NodeHandle object
 * @param file_path   path (can be empty) to a YAML file containing predefined joint states
 */
JointStateManager::JointStateManager(ros::NodeHandle* nodehandler, std::string file_path) : node_handler_(*nodehandler)
{
    // If the provided file path is not empty then load from the linked file
    if (!file_path.empty())
    {
        // Load the already defined joint states defined in the provided YAML file
        load_joint_states_from_file(file_path);
    }
    // Define the values corresponding to the number of anonymous (nameless) joint states stored and requested
    anonymous_stored_index_ = 0;
    anonymous_requested_index_ = 0;
    // Initialize services
    add_joint_state_service_ =
        node_handler_.advertiseService("add_joint_state", &JointStateManager::_add_joint_state, this);
    retrieve_joint_state_service_ =
        node_handler_.advertiseService("get_joint_state", &JointStateManager::_get_joint_state, this);
    ROS_INFO_STREAM("The joint state manager is ready");
}

/**
 Fill the joint_states_map_ attribute from the content of a YAML file
 * @param file_path Path of the file to parse and load (string)
 */
void JointStateManager::load_joint_states_from_file(std::string file_path)
{
    // Load the file in a YAML node
    YAML::Node robot_waypoints = YAML::LoadFile(file_path);
    // Using an iterator to go through all the potential maps (representing a joint state)
    for (YAML::const_iterator robot_waypoint = robot_waypoints.begin(); robot_waypoint != robot_waypoints.end();
         ++robot_waypoint)
    {
        // Define a JointState message
        sensor_msgs::JointState joint_state;
        // For each value of the map (which is also a map)
        for (YAML::const_iterator waypoint = robot_waypoint->second.begin(); waypoint != robot_waypoint->second.end();
             ++waypoint)
        {
            // Fill the JointState message fields with the name and value of the joint
            joint_state.name.push_back(waypoint->first.as<std::string>());
            joint_state.position.push_back(waypoint->second.as<float>());
        }
        // Add the JointState with the corresponding name to the attribute of the class
        joint_states_map_[robot_waypoint->first.as<std::string>()] = joint_state;
        ROS_INFO_STREAM("Joint state named " << robot_waypoint->first.as<std::string>() << " successfully added!");
    }
}

/**
 Store a JointState
 * @param  request  Object containing a field "joint_state_name" (string) that can be empty
                    and "joint_state" (sensor_msgs::JointState)
 * @param  response Object containing a field "success" (boolean) stating whether the storing was successfull
 * @return          Boolean that should be always true in order to avoid having runtime errors on the client side
 */
bool JointStateManager::_add_joint_state(grip_core::AddJointStateRequest& request,
                                         grip_core::AddJointStateResponse& response)
{
    // Extract the objects from the request
    std::string joint_state_name = request.joint_state_name;
    sensor_msgs::JointState joint_state = request.joint_state;
    // If a joint state with the same name already exists, then display an info message stating that it will be
    // overwritten
    if (joint_states_map_.count(joint_state_name) >= 1)
    {
        ROS_INFO_STREAM("The joint state named " << joint_state_name << " already exists and will be overwritten");
    }
    // If the name is empty, then store the message in a list in order to keep the insertion order
    if (joint_state_name.empty())
    {
        anonymous_joint_states_.push_back(joint_state);
        // Increment the number of stored joint states
        anonymous_stored_index_++;
    }
    // Otherwise just store it in the map
    else
    {
        joint_states_map_[joint_state_name] = joint_state;
    }
    // Fill the success field of the response to true
    response.success = true;
    return true;
}

/**
 Give access to an already stored sensor_msgs::JointState message
 * @param  request  Object containing a field "joint_state_name" (string) depicting the name of the JointState to
                    retrieve. If the field is empty then returns the oldest element of the anonymous joint states
 * @param  response Object containing a field "joint_state" (sensor_msgs::JointState) containing the requested
                    JointState and a field "success" (boolean) stating whether the state can be found
 * @return          Boolean that should be always true in order to avoid having runtime errors on the client side
 */
bool JointStateManager::_get_joint_state(grip_core::GetJointStateRequest& request,
                                         grip_core::GetJointStateResponse& response)
{
    // Extract the information from the anonymous_requested_index_
    std::string joint_state_name = request.joint_state_name;
    // If the name of the joint state is empty, and the anonymous joint state vector still has elements then return the
    // oldest element
    if ((joint_state_name.empty()) && (anonymous_requested_index_ < anonymous_stored_index_))
    {
        // Set the joint_state field to the oldest element and delete it in order to save memory
        response.joint_state = anonymous_joint_states_[anonymous_requested_index_];
        anonymous_joint_states_.erase(anonymous_joint_states_.begin());
        // Increment the number of elements retrieved from the anonymous list
        anonymous_requested_index_++;
        // Success field of the response is set to true
        response.success = true;
        return true;
    }
    // If a greater number of request has been made than the number of stored anonymous joint state then display an
    // error and set the success field to false
    else if ((anonymous_requested_index_ > anonymous_stored_index_) && (anonymous_stored_index_ != 0))
    {
        ROS_ERROR_STREAM("The number of requests has exceeded the number of joint states saved!");
        response.success = false;
        return true;
    }
    // If a JointState message with the requested name is in the map then access it and set it to the response
    else if (joint_states_map_.count(joint_state_name) == 1)
    {
        response.joint_state = joint_states_map_[joint_state_name];
        response.success = true;
        return true;
    }
    // Otherwise it means that the requested name does not exist, so display error message and set success to false
    else
    {
        ROS_ERROR_STREAM("The joint state named " << joint_state_name << " does not seem to exist!");
        response.success = false;
        return true;
    }
}

// If this file is called as a main, then create a node and launches the server
int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_state_manager_server");
    ros::NodeHandle node_handle;
    std::string constructor_argument = "";
    // Get the potential YAML file path passed as argument
    if (argc >= 2)
    {
        constructor_argument = argv[1];
    }
    JointStateManager joint_state_manager(&node_handle, constructor_argument);
    ros::spin();
    return 0;
}
