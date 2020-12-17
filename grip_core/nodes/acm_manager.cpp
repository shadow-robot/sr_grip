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

#include <grip_core/acm_manager.hpp>
#include <vector>
#include <string>

/**
 Constructor of the class
 * @param nodehandler reference to a ros NodeHandle object
 */
ACMManager::ACMManager(ros::NodeHandle* nodehandler) : node_handler_(*nodehandler)
{
    // Set the attributes of the class
    is_updated_ = false;
    number_entries_ = 0;
    initial_number_entries_ = 0;
    // Initialize services
    set_init_acm_service_ = node_handler_.advertiseService("set_init_acm", &ACMManager::_set_init_acm, this);
    update_acm_entry_service_ =
        node_handler_.advertiseService("update_acm_entry", &ACMManager::_update_acm_entry, this);
    get_modified_acm_service_ =
        node_handler_.advertiseService("get_modified_acm", &ACMManager::_get_modified_acm, this);
    // Display a message stating that initialisation was a success
    ROS_INFO_STREAM("The ACM plan manager is ready");
}

/**
 Set the initial acm attribute of the class
 * @param  request  Object containing a field "acm" (moveit_msgs::AllowedCollisionMatrix) that must correspond to the
                    initial state
 * @param  response Object containing a field "success" (boolean) stating whether the storing was successfull
 * @return          Boolean that should be always true in order to avoid having runtime errors on the client side
 */
bool ACMManager::_set_init_acm(grip_core::SetInitACMRequest& request,
                               grip_core::SetInitACMResponse& response)
{
    // Get the indices of the manipulator's links from the ROS parameter server and store it in the class attribute
    if (manipulator_links_indices_.empty())
    {
        node_handler_.getParam("manipulator_links_indices", manipulator_links_indices_);
    }
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
        initial_number_entries_ = initial_acm_.entry_names.size();
        is_updated_ = false;
        response.success = true;
    }
    // if anything goes wrong, catch the exception and set success to false
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
                    to the ACM entries. It can be adding, deleting an entry or reinitialise the current ACM. The other
                    field "entry_name" provided the name of the entry to modify.
 * @param  response Object containing a field "success" (boolean) stating whether the modification has been successfull
 * @return          Boolean that should be always true in order to avoid having runtime errors on the client side
 */
bool ACMManager::_update_acm_entry(grip_core::UpdateACMEntryRequest& request,
                                   grip_core::UpdateACMEntryResponse& response)
{
    // Sanity check, to make sure an initial ACM has bee nset
    if (initial_acm_.entry_names.empty())
    {
        ROS_ERROR_STREAM("An initial ACM needs to be set using the 'set_init_acm' service before using "
                         "this one");
        response.success = false;
        return true;
    }
    // Extract the action to perform
    uint8_t update_action = request.action;
    // If the updated_acm_ attribute needs to be reinitialised or set to the initial ACM
    if (!is_updated_ || update_action == request.REINIT)
    {
        updated_acm_ = initial_acm_;
        // Update the number of entries
        number_entries_ = updated_acm_.entry_names.size();
        is_updated_ = true;
        if (update_action == request.REINIT)
        {
            response.success = true;
            return true;
        }
    }

    // If the action requires the field entry_name field and the latter is empty display an error message and set
    // success to false
    if (request.entry_name.empty())
    {
        ROS_ERROR_STREAM("No target link provided for updating the ACM, cannot proceed");
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
        for (int entry_index = 0; entry_index < updated_acm_.entry_values.size(); entry_index++)
        {
            updated_acm_.entry_values[entry_index].enabled.push_back(false);
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
        updated_acm_.entry_values.push_back(allowed_collision_entry);
        // Add the name of the object to the entry names
        updated_acm_.entry_names.push_back(object_name);
        // Update the number of entries
        number_entries_ += 1;

        response.success = true;
        return true;
    }

    // When an entry must be deleted
    if (update_action == request.DELETE_ENTRY)
    {
        // Get an iterator containing the index of the name of the object to delete in the entry names
        std::vector<std::string>::iterator query_name_position =
            std::find(updated_acm_.entry_names.begin(), updated_acm_.entry_names.end(), object_name);
        // If the object cannot be found then display a warning message and set success to true since technically the
        // object is not in the ACM anymore
        if (query_name_position == updated_acm_.entry_names.end())
        {
            ROS_WARN_STREAM("The entry name " << object_name << " cannot be found in the ACM");
            response.success = true;
            return true;
        }
        // Get index of element from the iterator
        int index = std::distance(updated_acm_.entry_names.begin(), query_name_position);
        // Remove the entry corresponding to the object
        updated_acm_.entry_values.erase(updated_acm_.entry_values.begin() + index);
        // For all the other entries, remove the boolean corresponding to the deleted obejct
        for (int entry_index = 0; entry_index < updated_acm_.entry_values.size(); entry_index++)
        {
            updated_acm_.entry_values[entry_index].enabled.erase(
                updated_acm_.entry_values[entry_index].enabled.begin() + index);
        }
        // Remove the name of the object from the entries
        updated_acm_.entry_names.erase(updated_acm_.entry_names.begin() + index);
        // Update the number of entries
        number_entries_ -= 1;

        response.success = true;
        return true;
    }

    // If we are here it means that the action set in the request in not valid
    ROS_ERROR_STREAM("The action provided in the request is not supported");
    response.success = false;
    return true;
}

/**
 Update the allowed collision of updated_acm_ with respect to the modification requested
 * @param  request  Object containing a field "modification" (uint8) respresenting the kind of modification we want to
                    bring to the ACM. It can be nothing (simple getter), allowing self collision for the manipulator or
                    allowing collision between a list of objects and the manipulator.
 * @param  response Object containing a field "acm" (moveit_msgs::AllowedCollisionMatrix) containing the updated ACM
                    after modification and a field "success" (boolean) stating whether the modification has been
                    successfull
 * @return          Boolean that should be always true in order to avoid having runtime errors on the client side
 */
bool ACMManager::_get_modified_acm(grip_core::GetModifiedACMRequest& request,
                                   grip_core::GetModifiedACMResponse& response)
{
    // Sanity check, to make sure an initial ACM has bee nset
    if (initial_acm_.entry_names.empty())
    {
        ROS_ERROR_STREAM("An initial ACM needs to be set using the 'set_init_acm' service before using this one");
        response.success = false;
        return true;
    }
    // Extract the boolean to set in the ACM to allow or disallow collisions
    bool is_allowed = request.allow_collision;
    // Extract the kind of modification to bring to the ACM
    uint8_t modification = request.modification;

    // If the ACM must be modified to allow collision between an object and
    if (modification == request.MANIPULATOR_OBJECT_COLLISION)
    {
        // Extract the objects for which we should allow collision
        std::vector<std::string> objects = request.objects;
        // If nothing is specified then allow the collision with all the objects added from the initial ACM
        if (objects.empty())
        {
            ROS_DEBUG_STREAM("No argument provided, will allow collision between the manipulator and all the external "
                             "objects added after the initial ACM");

            // Get the number of objects added interactively
            uint16_t number_objects_added = number_entries_ - initial_number_entries_;
            // Add the indices corresponding to these objects
            for (size_t index_added_object = 0; index_added_object < number_objects_added; index_added_object++)
            {
                manipulator_links_indices_.push_back(initial_number_entries_ + index_added_object);
            }
        }
        // If objects has been specified
        else
        {
            // vector containing the indices of the specific objects
            std::vector<int> index_objects;
            // Iterator providing the position of the name of the object in the entries
            std::vector<std::string>::iterator position;
            // Name of the object mapping the content of the ACM
            std::string acm_object_name;
            for (int index_object = 0; index_object < objects.size(); index_object++)
            {
                acm_object_name = objects[index_object] + "__link";
                position = std::find(updated_acm_.entry_names.begin(), updated_acm_.entry_names.end(), acm_object_name);
                // Check whether the object has been found. If not, display a warning message
                if (position == updated_acm_.entry_names.end())
                {
                    ROS_WARN_STREAM("The object " << acm_object_name << " cannot be found in the entries of the ACM");
                }
                // Otherwise add the index of the object
                else
                {
                    index_objects.push_back(std::distance(updated_acm_.entry_names.begin(), position));
                }
            }
            // Extend the manipulator_links_indices_ attribute in order to allow collisions
            manipulator_links_indices_.insert(manipulator_links_indices_.end(), index_objects.begin(),
                                              index_objects.end());
        }
    }

    // Now that the manipulator_links_indices_ have been updated with respect to what was requested, change the entry
    // values of the ACM
    if (modification == request.MANIPULATOR_SELF_COLLISION || modification == request.MANIPULATOR_OBJECT_COLLISION)
    {
        // Nested for loops allowing to change the collision checking between all links stored in
        // manipulator_links_indices_
        for (int index = 0; index < manipulator_links_indices_.size(); index++)
        {
            int row = manipulator_links_indices_[index];
            for (int manip_index = 0; manip_index < manipulator_links_indices_.size(); manip_index++)
            {
                int col = manipulator_links_indices_[manip_index];
                // Avoid to check collision between the object and itself
                if (index != manip_index)
                {
                    updated_acm_.entry_values[row].enabled[col] = is_allowed;
                }
            }
        }
        response.acm = updated_acm_;
        response.success = true;
        return true;
    }
    // If the request is about just getting the current ACM
    if (modification == request.NOTHING)
    {
        if (!is_updated_)
        {
            updated_acm_ = initial_acm_;
        }
        response.acm = updated_acm_;
        response.success = true;
        return true;
    }

    // If we are here it means that the modification set in the request in not valid
    ROS_ERROR_STREAM("The modification provided in the request is not supported");
    response.success = false;
    return true;
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
