/*
* Copyright 2019, 2020 Shadow Robot Company Ltd.
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

#include <ros/ros.h>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <grip_core/GetPoseStamped.h>
#include <yaml-cpp/yaml.h>

// Node creating the frames corresponding to sensors and add the tranform to the tf2 tree
int main(int argc, char** argv)
{
    // Initializing the node
    ros::init(argc, argv, "sensors_tf2_broadcaster");
    ros::NodeHandle node_handler;

    YAML::Node parameters = YAML::LoadFile(argv[1]);
    for (YAML::const_iterator parameter = parameters.begin(); parameter != parameters.end(); ++parameter)
    {
        // Define the tf message and broadcaster
        static tf2_ros::StaticTransformBroadcaster tf2_broadcaster;
        geometry_msgs::TransformStamped transform_message;

        // If the parsed object is a map then create the trasnform from scratch
        if (parameter->second["initial_pose"].Type() == YAML::NodeType::Map)
        {
            // Fill the message
            transform_message.header.stamp = ros::Time::now();
            // Store the information provided in the YAML file
            transform_message.header.frame_id = parameter->second["initial_pose"]["reference_frame"].as<std::string>();
            transform_message.child_frame_id = parameter->second["frame_id"].as<std::string>();
            transform_message.transform.translation.x = parameter->second["initial_pose"]["position"]["x"].as<float>();
            transform_message.transform.translation.y = parameter->second["initial_pose"]["position"]["y"].as<float>();
            transform_message.transform.translation.z = parameter->second["initial_pose"]["position"]["z"].as<float>();
            if (parameter->second["initial_pose"]["orientation"]["r"])
            {
                // Define a quaternion and set its value given RPY provided in the YAML file
                tf2::Quaternion quaternion;
                float roll_angle = parameter->second["initial_pose"]["orientation"]["r"].as<float>();
                float pitch_angle = parameter->second["initial_pose"]["orientation"]["p"].as<float>();
                float yaw_angle = parameter->second["initial_pose"]["orientation"]["y"].as<float>();
                quaternion.setRPY(roll_angle, pitch_angle, yaw_angle);
                // Fill orientation of the frame
                transform_message.transform.rotation.x = quaternion.x();
                transform_message.transform.rotation.y = quaternion.y();
                transform_message.transform.rotation.z = quaternion.z();
                transform_message.transform.rotation.w = quaternion.w();
            }
            else
            {
                transform_message.transform.rotation.x =
                    parameter->second["initial_pose"]["orientation"]["x"].as<float>();
                transform_message.transform.rotation.y =
                    parameter->second["initial_pose"]["orientation"]["y"].as<float>();
                transform_message.transform.rotation.z =
                    parameter->second["initial_pose"]["orientation"]["z"].as<float>();
                transform_message.transform.rotation.w =
                    parameter->second["initial_pose"]["orientation"]["w"].as<float>();
            }
        }
        else
        {
            // Initialize a service client to access the already loaded named pose
            ros::ServiceClient get_pose = node_handler.serviceClient<grip_core::GetPoseStamped>("get_pose");
            grip_core::GetPoseStamped service;
            service.request.pose_name = parameter->second["initial_pose"].as<std::string>();
            get_pose.waitForExistence();
            // Call the service
            get_pose.call(service);
            // If any error happened while retrieving the pose, display an error message and break from the loop
            if (!service.response.success)
            {
                ROS_ERROR_STREAM("The required pose named " << parameter->second["initial_pose"].as<std::string>()
                                                            << " can't be retrieved. Cannot proceed!");
                break;
            }
            // Otherwise fill the transform
            else
            {
                // Fill the message
                transform_message.header.stamp = ros::Time::now();
                // Store the information provided in the YAML file
                transform_message.header.frame_id = service.response.pose_stamped.header.frame_id;
                transform_message.child_frame_id = parameter->second["frame_id"].as<std::string>();
                transform_message.transform.translation.x = service.response.pose_stamped.pose.position.x;
                transform_message.transform.translation.y = service.response.pose_stamped.pose.position.y;
                transform_message.transform.translation.z = service.response.pose_stamped.pose.position.z;
                transform_message.transform.rotation.x = service.response.pose_stamped.pose.orientation.x;
                transform_message.transform.rotation.y = service.response.pose_stamped.pose.orientation.y;
                transform_message.transform.rotation.z = service.response.pose_stamped.pose.orientation.z;
                transform_message.transform.rotation.w = service.response.pose_stamped.pose.orientation.w;
            }
        }
        // Send the transform
        tf2_broadcaster.sendTransform(transform_message);
    }
    ros::spin();
    return 0;
}
