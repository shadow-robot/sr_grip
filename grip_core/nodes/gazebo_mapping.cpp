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

#include <grip_core/gazebo_mapping.hpp>
#include <string>

/**
 Constructor of the class
 * @param nodehandler reference to a ros NodeHandle object
 */
GazeboMappingManager::GazeboMappingManager(ros::NodeHandle* nodehandler) : node_handler_(*nodehandler)
{
    // Initialize services
    add_gazebo_service_ =
        node_handler_.advertiseService("add_gazebo_mapping", &GazeboMappingManager::_add_gazebo_mapping, this);
    get_gazebo_object_type_service_ =
        node_handler_.advertiseService("get_object_type", &GazeboMappingManager::_get_object_type, this);
    delete_gazebo_mapping_service_ =
        node_handler_.advertiseService("delete_gazebo_mapping", &GazeboMappingManager::_delete_gazebo_mapping, this);
}

/**
 Store a mapping between an object name and its type
 * @param  request  Object containing a field "object_name" (string) and "object_type" (string)
 * @param  response Object containing a field "success" (boolean) stating whether the storing was successfull
 * @return          Boolean that should be always true in order to avoid having runtime errors on the client side
 */
bool GazeboMappingManager::_add_gazebo_mapping(grip_core::AddGazeboMappingRequest& request,
                                               grip_core::AddGazeboMappingResponse& response)
{
    // Extract the objects from the request
    std::string object_name = request.object_name;
    std::string object_type = request.object_type;
    // If an object is already stored in the map object with the same key then display a warning message and set success
    // to false
    if (gazebo_name_map_.count(object_name) >= 1)
    {
        ROS_WARN_STREAM("The object named " << object_name << " already exists and will not be overwritten");
        response.success = false;
        return true;
    }
    // Otherwise store the information inside the map
    gazebo_name_map_[object_name] = object_type;
    // Fill the success field of the response to true
    response.success = true;
    return true;
}

/**
 Give the type of a requested named object
 * @param  request  Object containing a field "object_name" (string) depicting the name of the object for which we want
                    to know its type.
 * @param  response Object containing a field "object_type" (string) containing the type of the requested named object
                    and a field "success" (boolean) stating whether the object type can be found
 * @return          Boolean that should be always true in order to avoid having runtime errors on the client side
 */
bool GazeboMappingManager::_get_object_type(grip_core::GetGazeboTypeRequest& request,
                                            grip_core::GetGazeboTypeResponse& response)
{
    // Extract the information from the request
    std::string object_name = request.object_name;
    // If we try to retrieve the type of an object not stored in the map object display a debug message and set success
    // to false
    if (gazebo_name_map_.count(object_name) < 1)
    {
        ROS_DEBUG_STREAM("The gazebo object named " << object_name << " does not seem to exist");
        response.success = false;
        return true;
    }
    // Otherwise set the response to the mapped object type
    response.object_type = gazebo_name_map_[object_name];
    response.success = true;
    return true;
}

/**
 Delete a gazebo mapping (called when deleting an object)
 * @param  request  Object containing a field "object_name" (string) depicting the name of the object that has been
                    deleted
 * @param  response Object containing a field "success" (boolean) stating whether the mapping has been deleted
 * @return          Boolean that should be always true in order to avoid having runtime errors on the client side
 */
bool GazeboMappingManager::_delete_gazebo_mapping(
    grip_core::DeleteGazeboMappingRequest& request,
    grip_core::DeleteGazeboMappingResponse& response)
{
    // Extract the information from the request
    std::string object_name = request.object_name;
    // If we try to delete something that has not been stored here display a debug message and set the success to false
    if (gazebo_name_map_.count(object_name) < 1)
    {
        ROS_WARN_STREAM("The gazebo object named " << object_name << " does not seem to exist");
        response.success = false;
        return true;
    }
    // Otherwise remove the object from the map and set success to true
    gazebo_name_map_.erase(object_name);
    response.success = true;
    return true;
}

// If this file is called as a main, then create a node and launches the server
int main(int argc, char** argv)
{
    ros::init(argc, argv, "gazebo_mapping_server");
    ros::NodeHandle node_handle;
    GazeboMappingManager gazebo_mapping_manager(&node_handle);
    ros::spin();
    return 0;
}
