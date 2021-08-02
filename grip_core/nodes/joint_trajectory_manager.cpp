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

#include <grip_core/joint_trajectory_manager.hpp>
#include <string>
#include <vector>
#include <algorithm>

/**
 Constructor of the class from an optional YAML file containing predefined named trajectories
 * @param nodehandler reference to a ros NodeHandle object
 * @param file_path   path (can be empty) to a YAML file containing predefined trajectories
 */
JointTrajectoryManager::JointTrajectoryManager(ros::NodeHandle* nodehandler, std::string file_path)
    : node_handler_(*nodehandler)
{
    // If the provided file path is not empty then load from the linked file
    if (!file_path.empty())
    {
        // Load the already defined joint trajectories defined in the provided YAML file
        load_trajectories_from_file(file_path);
    }
    // Define the values corresponding to the number of anonymous (nameless) joint trajectories stored and requested
    anonymous_stored_index_ = 0;
    anonymous_requested_index_ = 0;
    // Initialize services
    add_trajectory_service_ =
        node_handler_.advertiseService("add_trajectory", &JointTrajectoryManager::_add_trajectory, this);
    retrieve_trajectory_service_ =
        node_handler_.advertiseService("get_trajectory", &JointTrajectoryManager::_get_trajectory, this);
    reinitialise_service_ =
        node_handler_.advertiseService("reinitialise_trajectory_manager", &JointTrajectoryManager::_reinitialise, this);
    ROS_INFO_STREAM("The joint trajectory manager is ready");
}

/**
 Fill the trajectories_map_ attribute from the content of a YAML file
 * @param file_path Path of the file to parse and load (string)
 */
void JointTrajectoryManager::load_trajectories_from_file(std::string file_path)
{
    // Load the file in a YAML node
    YAML::Node named_trajectories = YAML::LoadFile(file_path);
    // Initialize a service client to access the already loaded named joint states
    ros::ServiceClient get_joint_state =
        node_handler_.serviceClient<grip_core::GetJointState>("get_joint_state");
    // Initialise the interface
    grip_core::GetJointState service;

    // Using an iterator to go through all the potential maps (representing trajectories)
    for (YAML::const_iterator named_trajectory = named_trajectories.begin();
         named_trajectory != named_trajectories.end(); ++named_trajectory)
    {
        // Define a JointTrajectory message
        trajectory_msgs::JointTrajectory joint_trajectory;
        // Float stating after how many seconds should be executed from the beginning of the trajectory when reaching a
        // given waypoint
        float time_from_start = 0.;
        // Get the number of waypoints composing the named trajectory
        int number_waypoints = named_trajectory->second.size();
        // Count the number of valid waypoints
        int number_valid_waypoints = 0;
        // Go through all the waypoints
        for (std::size_t waypoint_index = 0; waypoint_index < number_waypoints; waypoint_index++)
        {
            // Create a JointTrajectoryPoint message that will contain all inforamtions defined in the waypoints
            trajectory_msgs::JointTrajectoryPoint trajectory_point;
            // Extract the name of the waypoint we want to retrieve using the joint state manager
            service.request.joint_state_name = named_trajectory->second[waypoint_index]["name"].as<std::string>();
            get_joint_state.waitForExistence();
            // Call the service
            get_joint_state.call(service);
            // If any error happened while retrieving the joint state, display an error message and break from the loop
            if (!service.response.success)
            {
                ROS_ERROR_STREAM("The required joint state named "
                                 << named_trajectory->second[waypoint_index]["name"].as<std::string>()
                                 << " can't be retrieved. Cannot proceed!");
                break;
            }
            // Otherwise fill the JointTrajectoryPoint message
            else
            {
                // The order of insertion of the joint names can be different between the waypoints (loaded as maps)
                // But all specified values in the point must be in same order as the joint names in JointTrajectory.msg
                // Hence we compute a vector containing the indices required to sort the vector containing the joint
                // names.
                // Extract the joint names (unordered)
                std::vector<std::string> joint_names = service.response.joint_state.name;
                // Create the vector the will contain the sorting indices
                std::vector<int> sorted_indices(joint_names.size());
                // Index used to generate the original vector of indices
                std::size_t index(0);
                // Create a vector of ordered indices ([0,1,2,...])
                std::generate(std::begin(sorted_indices), std::end(sorted_indices), [&] { return index++; });  // NOLINT
                // Use C++ lambda function to change the indices of the index vector in order to have what we want
                std::sort(std::begin(sorted_indices), std::end(sorted_indices),
                          [&](int lambda1, int lambda2) { return joint_names[lambda1] < joint_names[lambda2]; });  // NOLINT
                // Once everything is done, sort the joint names to be able to check whether the joint names correspond
                // between the waypoints
                std::sort(joint_names.begin(), joint_names.end());
                // If the field "joint_names" is empty, then fill it with the sorted joint names of the first waypoint
                if (joint_trajectory.joint_names.empty())
                {
                    joint_trajectory.joint_names = joint_names;
                }
                // But if it is different (and then not empty) display an error message and break from the loop
                else if (joint_trajectory.joint_names != joint_names)
                {
                    ROS_ERROR_STREAM("One of the waypoint of the trajectory does not have the sames joints");
                    break;
                }
                // Keep filling the JointTrajectoryPoint
                number_valid_waypoints += 1;
                // Fill the "positions" field in the correct order
                for (auto sorted_index : sorted_indices)
                {
                    trajectory_point.positions.push_back(service.response.joint_state.position[sorted_index]);
                }
                // Compute the time from start using the interpolation time information from the YAML file
                time_from_start += named_trajectory->second[waypoint_index]["interpolate_time"].as<float>();
                // The field must be a ROS duration so convert it
                ros::Duration duration_from_start(time_from_start);
                trajectory_point.time_from_start = duration_from_start;
                // Add the JointTrajectoryPoint to the JointTrajectory message
                joint_trajectory.points.push_back(trajectory_point);

                // Trick to mimic that the robot stops at a waypoint and consists in creating another similar point
                // with the proper interpolation time
                if ((named_trajectory->second[waypoint_index]["pause_time"]) &&
                    (named_trajectory->second[waypoint_index]["pause_time"].as<float>() > 0.))
                {
                    // Initialise another JointTrajectoryPoint
                    trajectory_msgs::JointTrajectoryPoint extra_trajectory_point;
                    // Copy the positions of the named joint state
                    extra_trajectory_point.positions = trajectory_point.positions;
                    // Use the pause time as interpolation time
                    time_from_start += named_trajectory->second[waypoint_index]["pause_time"].as<float>();
                    // Convert it to a duration and set it to the messages
                    ros::Duration duration_from_start(time_from_start);
                    extra_trajectory_point.time_from_start = duration_from_start;
                    // Add the point to the trajectory
                    joint_trajectory.points.push_back(extra_trajectory_point);
                }
            }
        }
        // We want to add in the map only valid JointTrajectories, so comparing for each trajectory that the number of
        // valid waypoints correspond to the number waypoints declared in the YAML file
        if (number_valid_waypoints == number_waypoints)
        {
            trajectories_map_[named_trajectory->first.as<std::string>()] = joint_trajectory;
            ROS_INFO_STREAM("Trajectory named " << named_trajectory->first.as<std::string>() << " successfully added!");
        }
        // Otherwise display an error message
        else
        {
            ROS_ERROR_STREAM("The named trajectory " << named_trajectory->first.as<std::string>() << " could not be "
                                                                                                     "loaded!");
        }
    }
}

/**
 Store a JointTrajectory
 * @param  request  Object containing a field "trajectory_name" (string) that can be empty
                    and "joint_trajectory" (trajectory_msgs::JointTrajectory)
 * @param  response Object containing a field "success" (boolean) stating whether the storing was successfull
 * @return          Boolean that should be always true in order to avoid having runtime errors on the client side
 */
bool JointTrajectoryManager::_add_trajectory(grip_core::AddJointTrajectoryRequest& request,
                                             grip_core::AddJointTrajectoryResponse& response)
{
    // Extract the objects from the request
    std::string trajectory_name = request.trajectory_name;
    trajectory_msgs::JointTrajectory joint_trajectory = request.joint_trajectory;
    // If a joint trajectory with the same name already exists, then display an info message stating that it will be
    // overwritten
    if (trajectories_map_.count(trajectory_name) >= 1)
    {
        ROS_INFO_STREAM("The joint state named " << trajectory_name << " already exists and will be overwritten");
    }
    // If the name is empty, then store the message in a list in order to keep the insertion order
    if (trajectory_name.empty())
    {
        anonymous_trajectories_.push_back(joint_trajectory);
        // Increment the number of stored joint states
        anonymous_stored_index_++;
    }
    // Otherwise just store it in the map
    else
    {
        trajectories_map_[trajectory_name] = joint_trajectory;
    }
    // Fill the success field of the response to true
    response.success = true;
    return true;
}

/**
 Give access to an already stored trajectory_msgs::JointTrajectory message
 * @param  request  Object containing a field "trajectory_name" (string) depicting the name of the JointTrajectory to
                    retrieve. If the field is empty then returns the oldest element of the anonymous joint trajectories
 * @param  response Object containing a field "joint_trajectory" (trajectory_msgs::JointTrajectory) containing the
 *                  requested JointTrajectory and a field "success" (boolean) stating whether the state can be found
 * @return          Boolean that should be always true in order to avoid having runtime errors on the client side
 */
bool JointTrajectoryManager::_get_trajectory(grip_core::GetJointTrajectoryRequest& request,
                                             grip_core::GetJointTrajectoryResponse& response)
{
    // Extract the information from the anonymous_requested_index_
    std::string trajectory_name = request.trajectory_name;
    // If the name of the joint state is empty, and the anonymous joint state vector still has elements then return the
    // oldest element
    if ((trajectory_name.empty()) && (anonymous_requested_index_ < anonymous_stored_index_))
    {
        // Set the joint_trajectory field to the oldest element and delete it in order to save memory
        response.joint_trajectory = anonymous_trajectories_[anonymous_requested_index_];
        anonymous_trajectories_.erase(anonymous_trajectories_.begin());
        // Increment the number of elements retrieved from the anonymous list
        anonymous_requested_index_++;
        // Success field of the response is set to true
        response.success = true;
        return true;
    }
    // If a greater number of request has been made than the number of stored anonymous joint trajectory then display an
    // error and set the success field to false
    else if ((anonymous_requested_index_ > anonymous_stored_index_) && (anonymous_stored_index_ != 0))
    {
        ROS_ERROR_STREAM("The number of requests has exceeded the number of joint trajectories saved!");
        response.success = false;
        return true;
    }
    // If a JointTrajectory message with the requested name is in the map then access it and set it to the response
    else if (trajectories_map_.count(trajectory_name) == 1)
    {
        response.joint_trajectory = trajectories_map_[trajectory_name];
        response.success = true;
        return true;
    }
    // Otherwise it means that the requested name does not exist, so display error message and set success to false
    else
    {
        ROS_ERROR_STREAM("The joint trajectory named " << trajectory_name << " does not seem to exist!");
        response.success = false;
        return true;
    }
}

/**
 Reinitialise the manager
 * @param  request  Object containing a field argument (string) corresponding to the path of the file that contains
                    ROS messages to load
 * @param  response Object containing a field "success" (boolean) stating whether the operation was successfull or not
 * @return          Boolean that should be always true in order to avoid having runtime errors on the client side
 */
bool JointTrajectoryManager::_reinitialise(grip_core::ReinitManagerRequest& request,
                                           grip_core::ReinitManagerResponse& response)
{
    // Extract the objects from the request
    std::string file_path = request.argument;

    trajectories_map_.clear();
    anonymous_trajectories_.clear();
    anonymous_stored_index_ = 0;
    anonymous_requested_index_ = 0;

    if (!file_path.empty())
    {
      // Check that the file does exist
      struct stat buf;
      if (stat(file_path.c_str(), &buf) != -1)
      {
        load_trajectories_from_file(file_path);
      }
      else
      {
        ROS_WARN_STREAM("The file " + file_path + " does not seem to exist!");
        response.success = false;
        return true;
      }
    }
    // Fill the success field of the response to true
    response.success = true;
    return true;
}

// If this file is called as a main, then create a node and launches the server
int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_trajectory_manager_server");
    ros::NodeHandle node_handle;
    std::string constructor_argument = "";
    // Get the potential YAML file path passed as argument
    if (argc >= 2)
    {
        constructor_argument = argv[1];
    }
    JointTrajectoryManager joint_trajectory_manager(&node_handle, constructor_argument);
    ros::spin();
    return 0;
}
