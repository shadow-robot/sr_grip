/*
* Copyright 2019, 2020, 2023 Shadow Robot Company Ltd.
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

#include <grip_core/robot_pose_manager.hpp>
#include <string>

/**
 Constructor of the class
 * @param nodehandler reference to a ros NodeHandle object
 */
RobotPoseManager::RobotPoseManager(ros::NodeHandle* nodehandler, std::string file_path) : node_handler_(*nodehandler)
{
    // If the provided file path is not empty then load from the linked file
    if (!file_path.empty())
    {
        // Load the already defined poses defined in the provided YAML file
        load_robot_poses_from_file(file_path);
    }
    // Define the values corresponding to the number of anonymous (nameless) robot poses stored and requested
    anonymous_stored_index_ = 0;
    anonymous_requested_index_ = 0;
    // Initialize services
    add_robot_pose_service_ =
        node_handler_.advertiseService("add_robot_pose", &RobotPoseManager::_add_robot_pose, this);
    retrieve_robot_pose_service_ =
        node_handler_.advertiseService("get_robot_pose", &RobotPoseManager::_get_robot_pose, this);
    reinitialise_service_ =
        node_handler_.advertiseService("reinitialise_robot_pose_manager", &RobotPoseManager::_reinitialise, this);
    // Display a message stating that initialisation was a success
    ROS_INFO_STREAM("The robot pose manager is ready");
}

/**
 Fill the robot_poses_map_ attribute from the content of a YAML file
 * @param file_path Path of the file to parse and load (string)
 */
void RobotPoseManager::load_robot_poses_from_file(std::string file_path)
{
    // Load the file in a YAML node
    YAML::Node robot_poses = YAML::LoadFile(file_path);
    // Using an iterator to go through all the potential maps (representing a pose)
    for (YAML::const_iterator robot_pose = robot_poses.begin(); robot_pose != robot_poses.end(); ++robot_pose)
    {
        // Make sure the current pose is meant to be a RobotPose and not a PoseStamped
        if (!robot_pose->second["position"])
        {
            // Define a RobotPose message
            grip_core::RobotPose robot_pose_msg;
            // Define a PoseArray message
            geometry_msgs::PoseArray pose_array_msg;
            // For each component of the robot pose
            for (YAML::const_iterator component = robot_pose->second.begin(); component != robot_pose->second.end();
                 ++component)
            {
                geometry_msgs::Pose pose_msg;
                if (component->first.as<std::string>() == "reference_frame")
                {
                    pose_array_msg.header.frame_id = component->second.as<std::string>();
                }
                else
                {
                    robot_pose_msg.end_effector_link_names.push_back(component->first.as<std::string>());
                    pose_msg.position.x = component->second["position"]["x"].as<float>();
                    pose_msg.position.y = component->second["position"]["y"].as<float>();
                    pose_msg.position.z = component->second["position"]["z"].as<float>();
                    if (component->second["orientation"]["x"])
                    {
                        pose_msg.orientation.x = component->second["orientation"]["x"].as<float>();
                        pose_msg.orientation.y = component->second["orientation"]["y"].as<float>();
                        pose_msg.orientation.z = component->second["orientation"]["z"].as<float>();
                        pose_msg.orientation.w = component->second["orientation"]["w"].as<float>();
                    }
                    else
                    {
                        tf2::Quaternion quaternion;
                        quaternion.setRPY(component->second["orientation"]["r"].as<float>(),
                                          component->second["orientation"]["p"].as<float>(),
                                          component->second["orientation"]["y"].as<float>());
                        pose_msg.orientation.x = quaternion.x();
                        pose_msg.orientation.y = quaternion.y();
                        pose_msg.orientation.z = quaternion.z();
                        pose_msg.orientation.w = quaternion.w();
                    }
                    pose_array_msg.poses.push_back(pose_msg);
                }
            }
            robot_pose_msg.pose_array = pose_array_msg;

            robot_poses_map_[robot_pose->first.as<std::string>()] = robot_pose_msg;
            ROS_INFO_STREAM("Robot pose named " << robot_pose->first.as<std::string>() << " successfully added!");
        }
    }
}

/**
 Store a RobotPose
 * @param  request  Object containing a field "pose_name" (string) that can be empty
                    and "robot_pose" (RobotPose)
 * @param  response Object containing a field "success" (boolean) stating whether the storing was successfull
 * @return          Boolean that should be always true in order to avoid having runtime errors on the client side
 */
bool RobotPoseManager::_add_robot_pose(grip_core::AddRobotPoseRequest& request,
                                       grip_core::AddRobotPoseResponse& response)
{
    // Extract the objects from the request
    std::string robot_pose_name = request.robot_pose_name;
    grip_core::RobotPose robot_pose = request.robot_pose;
    // If a pose with the same name already exists, then display an info message stating that it will be overwritten
    if (robot_poses_map_.count(robot_pose_name) >= 1)
    {
        ROS_INFO_STREAM("The robot pose named " << robot_pose_name << " already exists and will be overwritten");
    }
    // If the name is empty, then store the message in a list in order to keep the insertion order
    if (robot_pose_name.empty())
    {
        anonymous_robot_poses_.push_back(robot_pose);
        // Increment the number of stored robot poses
        anonymous_stored_index_++;
    }
    // Otherwise just store it in the map
    else
    {
        robot_poses_map_[robot_pose_name] = robot_pose;
    }
    // Fill the success field of the response to true
    response.success = true;
    return true;
}

/**
 Give access to an already stored RobotPose message
 * @param  request  Object containing a field "pose_name" (string) depicting the name of the robot pose to retrieve.
                    If the field is empty then returns the oldest element of the anonymous robot poses
 * @param  response Object containing a field "robot_pose" (RobotPose) containing the requested robot pose
                    and a field "success" (boolean) stating whether the robot pose can be found
 * @return          Boolean that should be always true in order to avoid having runtime errors on the client side
 */
bool RobotPoseManager::_get_robot_pose(grip_core::GetRobotPoseRequest& request,
                                       grip_core::GetRobotPoseResponse& response)
{
    // Extract the information from the request
    std::string robot_pose_name = request.robot_pose_name;
    // If the name of the carteisan pose is empty, and the anonymous robot pose vector still has elements then
    // return the oldest element
    if ((robot_pose_name.empty()) && (anonymous_requested_index_ < anonymous_stored_index_))
    {
        // Set the robot pose to the oldest element and delete it in order to save memory
        response.robot_pose = anonymous_robot_poses_[anonymous_requested_index_];
        anonymous_robot_poses_.erase(anonymous_robot_poses_.begin());
        // Increment the number of elements retrieved from the anonymous list
        anonymous_requested_index_++;
        // Success field of the response is set to true
        response.success = true;
        return true;
    }
    // If a greater number of request has been made than the number of stored anonymous robot poses then display an
    // error and set the success field to false
    else if ((anonymous_requested_index_ > anonymous_stored_index_) && (anonymous_stored_index_ != 0))
    {
        ROS_ERROR_STREAM("The number of requests has exceeded the number of robot poses saved!");
        response.success = false;
        return true;
    }
    // If a RobotPose message with the requested name is in the map then access it and set it to the response
    else if (robot_poses_map_.count(robot_pose_name) == 1)
    {
        response.robot_pose = robot_poses_map_[robot_pose_name];
        response.success = true;
        return true;
    }
    // Otherwise it means that the requested name does not exist, so display error message and set success to false
    else
    {
        ROS_ERROR_STREAM("The robot pose named " << robot_pose_name << " does not seem to exist!");
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
bool RobotPoseManager::_reinitialise(grip_core::ReinitManagerRequest& request,
                                     grip_core::ReinitManagerResponse& response)
{
    // Extract the objects from the request
    std::string file_path = request.argument;

    robot_poses_map_.clear();
    anonymous_robot_poses_.clear();
    anonymous_stored_index_ = 0;
    anonymous_requested_index_ = 0;

    if (!file_path.empty())
    {
      // Check that the file does exist
      struct stat buf;
      if (stat(file_path.c_str(), &buf) != -1)
      {
        load_robot_poses_from_file(file_path);
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
    ros::init(argc, argv, "robot_pose_manager_server");
    ros::NodeHandle node_handle;
    std::string constructor_argument = "";
    // Get the potential YAML file path passed as argument
    if (argc >= 2)
    {
        constructor_argument = argv[1];
    }
    RobotPoseManager robot_pose_manager(&node_handle, constructor_argument);
    ros::spin();
    return 0;
}
