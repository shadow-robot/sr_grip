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

#include <grip_core/acm_manager.hpp>
#include <vector>
#include <string>
#include <map>

/**
 Constructor of the class
 * @param nodehandler reference to a ros NodeHandle object
 */
ACMManager::ACMManager(ros::NodeHandle* nodehandler) : node_handler_(*nodehandler)
{
    // Set the attributes of the class
    is_updated_ = false;
    number_entries_ = 0;
    // Initialize services
    set_init_acm_service_ = node_handler_.advertiseService("set_init_acm", &ACMManager::_set_init_acm, this);
    update_acm_entry_service_ =
        node_handler_.advertiseService("update_acm_entry", &ACMManager::_update_acm_entry, this);
    modify_acm_service_ =
        node_handler_.advertiseService("modify_acm", &ACMManager::_modify_acm, this);
    // Initialize the publisher
    publish_current_acm_ = node_handler_.advertise<moveit_msgs::PlanningScene>("planning_scene", 5);
    // Display a message stating that initialisation was a success
    ROS_INFO_STREAM("The ACM manager is ready");
}

/**
 Set the initial ACM, that can be used to reinitialize the system at its original state
 * @param  request  Object containing a field "acm" (moveit_msgs::AllowedCollisionMatrix) that must correspond to the
                    initial state
 * @param  response Object containing a field "success" (boolean) stating whether the storing was successfull
 * @return          Boolean that should be always true in order to avoid having runtime errors on the client side
 */
bool ACMManager::_set_init_acm(grip_core::SetInitACMRequest& request, grip_core::SetInitACMResponse& response)
{
    // If an initial ACM has already been set, display a warning message
    if (!initial_acm_.entry_names.empty())
    {
        ROS_INFO_STREAM("An initial ACM used as reference already exists and is going to be overwritten");
    }
    // Store the ACM provided in the request and set the boolean specifying whether we should set the updated ACM to the
    // initial one
    try
    {
        initial_acm_ = request.acm;
        // Make sure we know which robot's links correspond to which entry in the ACM
        _set_robot_links_mapping(initial_acm_.entry_names);
        // We assume that no objects are part of the ACM at first (i.e. collisions will be checked)
        object_links_map_.clear();
        is_updated_ = false;
        response.success = true;
    }
    // If anything goes wrong, catch the exception and set success to false
    catch (const ros::Exception& exception)
    {
        ROS_ERROR_STREAM("An error occurred when storing the provided ACM: " << exception.what());
        response.success = false;
    }
    return true;
}

/**
 Update the entries of the current ACM
 * @param  request  Object containing a field "action" (uint8) respresenting the kind of modification we want to bring
                    to the ACM entries. It can be adding or deleting an entry, or reinitialise the current ACM.
                    The field "entry_name" provides the name of the entry to modify.
 * @param  response Object containing a field "success" (boolean) stating whether the modification has been successfull
 * @return          Boolean that should be always true in order to avoid having runtime errors on the client side
 */
bool ACMManager::_update_acm_entry(grip_core::UpdateACMEntryRequest& request,
                                   grip_core::UpdateACMEntryResponse& response)
{
    // Set the response success to False by default
    response.success = false;
    // Sanity check, to make sure an initial ACM has been set
    if (initial_acm_.entry_names.empty())
    {
        ROS_INFO_STREAM("No initial ACM has been set using the 'set_init_acm' service. Creating the initial one from "
                        "the current scene.");
        // Initialize the ACM from the current state of the scene (before adding/removing any entry)
        bool res = initialize_acm_from_scene();
        object_links_map_.clear();
        // If the initialization failed, stop here
        if (!res)
        {
            return true;
        }
    }
    // Extract the action to perform
    uint8_t update_action = request.action;
    // If the current_acm_ attribute needs to be reinitialised or set to the initial ACM
    if (!is_updated_ || update_action == request.REINIT)
    {
        current_acm_ = initial_acm_;
        // Update the number of entries
        number_entries_ = current_acm_.entry_names.size();
        is_updated_ = true;
        if (update_action == request.REINIT)
        {
            response.success = true;
            object_links_map_.clear();
        }
    }

    // If the action requires the field entry_name field and the latter is empty display an error message and set
    // success to false
    if (request.entry_name.empty())
    {
        ROS_ERROR_STREAM("No target link(s) provided for updating the ACM, cannot proceed");
        response.success = false;
        return true;
    }

    // Rename the object_name to map the content of the collision objects stored in the world field of the planning
    // scene
    std::string object_name = request.entry_name + "__link";
    // When an entry must be added
    if (update_action == request.ADD_ENTRY)
    {
        // For each existing entry, add at the end a boolean setting whether we should skip the collision checking
        // Since we set it to false, the new object can't be in collision with any links of the robot
        for (int entry_index = 0; entry_index < current_acm_.entry_values.size(); entry_index++)
        {
            current_acm_.entry_values[entry_index].enabled.push_back(false);
        }
        // Create a new collision entry for the object to add
        moveit_msgs::AllowedCollisionEntry allowed_collision_entry;
        // Set the vector to make the collision checking work with all the other links
        for (uint16_t index = 0; index < number_entries_; index++)
        {
            allowed_collision_entry.enabled.push_back(false);
        }
        // Disallow the collision checking between the object and itself
        allowed_collision_entry.enabled.push_back(true);
        // Add the collision entry to the updated ACM
        current_acm_.entry_values.push_back(allowed_collision_entry);
        // Add the name of the object to the entry names
        current_acm_.entry_names.push_back(object_name);
        object_links_map_[request.entry_name] = number_entries_;
        // Update the number of entries
        number_entries_ += 1;

        response.success = true;
    }

    // When an entry must be deleted
    if (update_action == request.DELETE_ENTRY)
    {
        // Get an iterator containing the index of the name of the object to delete in the entry names
        std::vector<std::string>::iterator query_name_position =
            std::find(current_acm_.entry_names.begin(), current_acm_.entry_names.end(), object_name);
        // If the object cannot be found then display a warning message and set success to true since technically the
        // object is not in the ACM anymore
        if (query_name_position == current_acm_.entry_names.end())
        {
            ROS_WARN_STREAM("The entry name " << object_name << " cannot be found in the ACM");
            response.success = true;
            // return true;
        }
        // Get index of element from the iterator
        int index = std::distance(current_acm_.entry_names.begin(), query_name_position);
        // Remove the entry corresponding to the object
        current_acm_.entry_values.erase(current_acm_.entry_values.begin() + index);
        // For all the other entries, remove the boolean corresponding to the deleted obejct
        for (int entry_index = 0; entry_index < current_acm_.entry_values.size(); entry_index++)
        {
            current_acm_.entry_values[entry_index].enabled.erase(
                current_acm_.entry_values[entry_index].enabled.begin() + index);
        }
        // Remove the name of the object from the entries
        current_acm_.entry_names.erase(current_acm_.entry_names.begin() + index);
        object_links_map_.erase(request.entry_name);
        // Update the number of entries
        number_entries_ -= 1;

        response.success = true;
        return true;
    }

    // If the ACM is successfully updated and that the users wishes to publish it to MoveIt!
    if (request.publish && response.success)
    {
        moveit_msgs::PlanningScene new_scene;
        new_scene.is_diff = true;
        new_scene.allowed_collision_matrix = current_acm_;
        publish_current_acm_.publish(new_scene);
    }
    // If everything worked well then just return true
    else if (!response.success)
    {
      ROS_ERROR_STREAM("The action provided in the request is not supported");
    }

    // In any case, return True
    return true;
}

/**
 Update the allowed collisions with respect to the modification requested
 * @param  request  Object containing a field "modification" (uint8) respresenting the kind of modification we want to
                    bring to the ACM. It can either allow self collision for a given set of robot links or
                    allow collision between a list of objects and a set of robot links.
 * @param  response Object containing a field "success" (boolean) stating if the modification has been successfull
 * @return          Boolean that should always be true in order to avoid having runtime errors on the client side
 */
bool ACMManager::_modify_acm(grip_core::ModifyACMRequest& request, grip_core::ModifyACMResponse& response)
{
    // Set the response success field as False by default
    response.success = false;
    // Sanity check, to make sure an initial ACM has been set
    if (initial_acm_.entry_names.empty())
    {
      ROS_INFO_STREAM("No initial ACM has been set using the 'set_init_acm' service. Creating the initial one from "
                      "the current scene.");

      bool res = initialize_acm_from_scene();
      // If the initialization fails, then stops here
      if (!res)
      {
          return true;
      }
    }
    // Extract the boolean to set in the ACM to allow or disallow collisions
    bool is_allowed = request.allow_collision;
    // Extract the kind of modification to bring to the ACM
    uint8_t modification = request.modification;
    // Extract the links of the robot for which we should allow/disallow collisions
    std::vector<std::string> robot_links = request.robot_links;
    // Vector of indices that are going to be used to modify the ACM
    std::vector<int> indices_to_modify;
    // Iterator allowing to find elements in configured maps
    std::map<std::string, int>::iterator it;
    // Out of all the links part of the initial ACM, extract only hte one part of the request
    for (size_t index_link_request = 0; index_link_request < robot_links.size(); index_link_request++)
    {
      it = robot_links_map_.find(robot_links[index_link_request]);
      // If a given link is not found, display an error message and stops here
      if (it == robot_links_map_.end())
      {
        ROS_ERROR_STREAM("Could not find the robot's link named " << robot_links[index_link_request]);
        return true;
      }
      // Otherwise get its index
      else
      {
        indices_to_modify.push_back(it->second);
      }
    }

    // If the ACM must be modified to allow collision between a given list of objects and the configured links
    if (modification == request.ROBOT_OBJECT_COLLISION)
    {
        // Extract the objects for which we should allow collision
        std::vector<std::string> objects = request.objects;
        // If nothing is specified then allow the collision with all the objects added since the initial ACM
        if (objects.empty())
        {
            ROS_DEBUG_STREAM("No argument provided, hence allowing collisions between the robot and all the objects "
                             "added to the scene since the initial ACM");

            uint16_t number_robot_links = robot_links_map_.size();
            // Get the number of objects added interactively
            uint16_t number_objects_added = object_links_map_.size();
            // Add the indices corresponding to these objects
            for (it = object_links_map_.begin(); it != object_links_map_.end(); ++it)
            {
                indices_to_modify.push_back(it->second);
            }
        }
        // If the field objects is specified
        else
        {
          // Get the indices of the objects specified by the user
          for (size_t index_link_request = 0; index_link_request < objects.size(); index_link_request++)
          {
            it = object_links_map_.find(objects[index_link_request]);
            // If a given object cannot be found then returns an error
            if (it == object_links_map_.end())
            {
              ROS_ERROR_STREAM("Could not find the object named " << objects[index_link_request]);
              response.success = false;
              return true;
            }
            else
            {
              indices_to_modify.push_back(it->second);
            }
          }
        }
    }

    // Now that the manipulator_links_indices_ have been updated with respect to what was requested, change the values
    // of the ACM
    if (modification == request.ROBOT_SELF_COLLISION || modification == request.ROBOT_OBJECT_COLLISION)
    {
        // Nested for loops allowing to change the collision checking between all links stored in indices_to_modify
        for (int index = 0; index < indices_to_modify.size(); index++)
        {
            int row = indices_to_modify[index];
            for (int manip_index = 0; manip_index < indices_to_modify.size(); manip_index++)
            {
                int col = indices_to_modify[manip_index];
                // Avoid to check collision between the object and itself
                if (index != manip_index)
                {
                    current_acm_.entry_values[row].enabled[col] = is_allowed;
                }
            }
        }
        response.success = true;
        if (request.publish && response.success)
        {
            moveit_msgs::PlanningScene new_scene;
            new_scene.is_diff = true;
            new_scene.allowed_collision_matrix = current_acm_;
            publish_current_acm_.publish(new_scene);
        }
        return true;
    }

    // If we are here it means that the modification set in the request in not valid
    ROS_ERROR_STREAM("The modification provided in the request is not supported");
    return true;
}

/**
 Set the initial acm attribute of the class from the current scene
 * @return  Boolean stating whether the initialization worked properly or not
 */
bool ACMManager::initialize_acm_from_scene()
{
    ros::ServiceClient get_planning_scene_service =
        node_handler_.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
    // Make sure we can access the service
    get_planning_scene_service.waitForExistence();
    // Create a request for the service that contains the ACM
    moveit_msgs::GetPlanningScene service;
    service.request.components.components = moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX |
                                            moveit_msgs::PlanningSceneComponents::SCENE_SETTINGS;

    // If the current ACM can't be retrieved return false
    if (!get_planning_scene_service.call(service))
    {
      ROS_ERROR_STREAM("Failed to retrieve the current ACM");
      return false;
    }

    // Store the ACM returned by the service and set the boolean specifying whether we should set the updated ACM to the
    // initial one
    try
    {
        initial_acm_ = service.response.scene.allowed_collision_matrix;
        _set_robot_links_mapping(initial_acm_.entry_names);
        is_updated_ = false;
    }
    // If anything goes wrong, catch the exception and return false
    catch (const ros::Exception& exception)
    {
        ROS_ERROR_STREAM("An error occurred when storing the retrieved ACM: " << exception.what());
        return false;
    }
    return true;
}

/**
 Update the robot_links_map_ attribute from a vector of strings
 */
void ACMManager::_set_robot_links_mapping(std::vector<std::string> entry_names)
{
  robot_links_map_.clear();
  for (int entry_index = 0; entry_index < entry_names.size(); entry_index++)
  {
    robot_links_map_[entry_names[entry_index]] = entry_index;
  }
}

// If this file is called as a main, then create a node and launches the server
int main(int argc, char** argv)
{
    ros::init(argc, argv, "acm_manager_server");
    ros::NodeHandle node_handle;
    ACMManager acm_manager(&node_handle);
    ros::spin();
    return 0;
}
