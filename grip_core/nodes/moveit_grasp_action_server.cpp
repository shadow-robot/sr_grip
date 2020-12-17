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

#include <grip_core/moveit_grasp_action_server.hpp>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>

/**
 Constructor of the class
 * @param nodehandler               reference to a ros NodeHandle object
 * @param action_server_name        name given to the action server
 * @param manipulator_group_name    name given to group corresponding to the manipulator that must be actuated
 */
GraspActionServer::GraspActionServer(ros::NodeHandle* node_handler, std::string action_server_name,
                                     std::string manipulator_group_name)
  : action_server_(*node_handler, action_server_name, false), move_group_interface_(manipulator_group_name)
{
    action_server_.registerGoalCallback(boost::bind(&GraspActionServer::goal_callback, this));
    action_server_.registerPreemptCallback(boost::bind(&GraspActionServer::preempt_callback, this));
    action_server_.start();
}

/**
 * Method called everytime a goal is received.
 * It checks that the server is not busy with another action, checks that the grasp command is valid and execute
 * the grasp defined if possible. If it is not, a log is displayed and the server sends a result equivalent
 * to ABORTED. If the grasp is executed then also publishes a feedback containing the joint values during execution
 */
void GraspActionServer::goal_callback()
{
    // If the server is already processing a goal
    if (busy_)
    {
        ROS_ERROR("The action will not be processed because the server is already busy with another action. "
                  "Please preempt the latter or wait before sending a new goal");
        return;
    }
    // Extract the GraspCommand msg from the accepted goal
    goal_grasp_command_ = action_server_.acceptNewGoal()->grasp_command;
    ROS_INFO("New goal received");
    // Set busy to true
    busy_ = true;

    // Extract the correct ManipulatorState corresponding to the grasp_state
    if (goal_grasp_command_.grasp_state == 0)
    {
        manipulator_state_ = goal_grasp_command_.grasp.pregrasp;
    }
    else if (goal_grasp_command_.grasp_state == 1)
    {
        manipulator_state_ = goal_grasp_command_.grasp.grasp;
    }
    else if (goal_grasp_command_.grasp_state == 2)
    {
        manipulator_state_ = goal_grasp_command_.grasp.postgrasp;
    }

    // Check whether the content of the ManipulatorState is consistent, i.e
    // the number of name and values corresponding to the joints match and won't lead to an error.
    // The back() function just ensure that we use the last TrajectoryJointPoint added
    if (manipulator_state_.posture.name.size() != manipulator_state_.posture.position.size())
    {
        // if the message is not consistent display a log, publish an aborted result, exit from the callback
        // and set busy to false
        ROS_ERROR("The number of joints and the desired posture provided in the goal don't match.");
        action_result_.pregrasped.data = false;
        action_result_.grasped.data = false;
        action_result_.postgrasped.data = false;
        action_server_.setAborted(action_result_);
        busy_ = false;
        return;
    }

    // Declare the key and value of the map<string, double>
    std::string joint_name;
    double joint_value;
    // Clear the map to get updates target joints
    target_joints_.clear();
    // Vector used to get the target joints in order to test if we are already in a desired state
    std::vector<double> target_joint_values;
    // Initialise the target joints (map<string, double>) composed ot the fields of the ManipulatorState
    for (int index = 0; index < manipulator_state_.posture.name.size(); index++)
    {
        joint_name = manipulator_state_.posture.name[index];
        joint_value = manipulator_state_.posture.position[index];
        target_joints_.insert(std::pair<std::string, double>(joint_name, joint_value));
        target_joint_values.push_back(joint_value);
    }
    // Initialise the start state for motion planning to the current one
    move_group_interface_.setStartStateToCurrentState();

    // If the target joints correspond to the current state then we do not need to keep going return as if it has
    // been executed
    std::vector<double> current_joint_values = move_group_interface_.getCurrentJointValues();
    // Sort the values in order to remove any doubt regarding the order in which they are returned or stored
    std::sort(target_joint_values.begin(), target_joint_values.end());
    std::sort(current_joint_values.begin(), current_joint_values.end());
    // Make sure that we have comparable values (turns stuff such as 1.e-05 into 0.)
    std::transform(current_joint_values.begin(), current_joint_values.end(), current_joint_values.begin(),
                   [](double& value) { return std::roundf(value); });  // NOLINT
    // If the two vectors correspond then send the same result as if the execution was successful
    if (current_joint_values == target_joint_values)
    {
        action_result_.pregrasped.data = (0 == goal_grasp_command_.grasp_state);
        action_result_.grasped.data = (1 == goal_grasp_command_.grasp_state);
        action_result_.postgrasped.data = (2 == goal_grasp_command_.grasp_state);
        action_server_.setSucceeded(action_result_);
        busy_ = false;
        return;
    }

    // Set the target fot the motion planner. This function returns false if the joints are out of bound.
    // In that particular case publish an aborted result, exit from the callback and set busy to false
    if (!move_group_interface_.setJointValueTarget(target_joints_))
    {
        ROS_ERROR("Joint values out of bound.");
        action_result_.pregrasped.data = false;
        action_result_.grasped.data = false;
        action_result_.postgrasped.data = false;
        action_server_.setAborted(action_result_);
        busy_ = false;
        return;
    }

    // Plan and execute asynchronously in order to publish a feedback.
    // This function returns a Code message and if it is not the success one then exit the call back,
    // publish an ABORTED message and set busy to false
    if (!(move_group_interface_.asyncMove() == moveit::planning_interface::MoveItErrorCode::SUCCESS))
    {
        ROS_ERROR("Error while executing the provided grasp.");
        action_result_.pregrasped.data = false;
        action_result_.grasped.data = false;
        action_result_.postgrasped.data = false;
        action_server_.setAborted(action_result_);
        busy_ = false;
        return;
    }

    // Listen to the feedback provided by the move group to know if the asynchronous execution is finished.
    // If that is the case then the feedback's state will be set to "IDLE".
    // For some reason can not do that properly in a subscriber's callback
    moveit_msgs::MoveGroupActionFeedbackConstPtr feedback_message;
    feedback_message = ros::topic::waitForMessage<moveit_msgs::MoveGroupActionFeedback>("/move_group/feedback");
    while ("IDLE" != feedback_message->feedback.state)
    {
        // As long as the execution is not terminated then fill a GraspFeedback message and publish it
        action_feedback_.current_joint_state.header.stamp = ros::Time::now();
        action_feedback_.current_joint_state.name = move_group_interface_.getActiveJoints();
        action_feedback_.current_joint_state.position = move_group_interface_.getCurrentJointValues();
        action_server_.publishFeedback(action_feedback_);
        // Update the feedback message to be sure to catch the stopping signal
        feedback_message = ros::topic::waitForMessage<moveit_msgs::MoveGroupActionFeedback>("/move_group/feedback");
    }

    // If we arrive here then it means that everything went well so send a success result and set busy to false
    action_result_.pregrasped.data = (0 == goal_grasp_command_.grasp_state);
    action_result_.grasped.data = (1 == goal_grasp_command_.grasp_state);
    action_result_.postgrasped.data = (2 == goal_grasp_command_.grasp_state);
    action_server_.setSucceeded(action_result_);
    busy_ = false;
}

void GraspActionServer::preempt_callback()
{
    ROS_INFO("Action preempted");
    move_group_interface_.stop();
    action_server_.setPreempted();
}

int main(int argc, char** argv)
{
    // Initalise the node
    ros::init(argc, argv, "moveit_grasp_action_server");
    // Initialise the node handler that will be given to the class' constructor
    ros::NodeHandle node_handler;
    // We need an AsyncSpinner in order to run the move_group in C++ without any error
    // Please see https://github.com/ros-planning/moveit/issues/1187 for more details
    ros::AsyncSpinner spinner(2);
    // Starts the spinner
    spinner.start();
    // Declare strings corresponding to the name of the action server and manipulator's group name
    std::string action_server_name, manipulator_group_name;
    // Initialise them regarding what have been uploaded on the rosparam server
    node_handler.getParam("manipulator_controller_action_server_name", action_server_name);
    node_handler.getParam("manipulator_group_name", manipulator_group_name);
    // Construct and launch the action server
    GraspActionServer grasp_action_server_(&node_handler, action_server_name, manipulator_group_name);
    // Wait for shutdown
    ros::waitForShutdown();
    return 0;
}
