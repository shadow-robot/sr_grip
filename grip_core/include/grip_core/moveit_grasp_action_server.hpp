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

#ifndef GRIP_CORE_MOVEIT_GRASP_ACTION_SERVER_H
#define GRIP_CORE_MOVEIT_GRASP_ACTION_SERVER_H

#include <string>
#include <vector>
#include <utility>
#include <map>
#include <algorithm>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/MoveGroupActionFeedback.h>
#include <grip_core/GraspCommand.h>
#include <grip_core/GraspAction.h>
#include <grip_core/GraspFeedback.h>
#include <grip_core/ManipulatorState.h>
#include <grip_core/GraspResult.h>

/**
 This class runs the server allowing to execute grasps using actions (as long as they follow the provided interface)
 through MoveIt!
 */
class GraspActionServer
{
  public:
    // Constructor
    GraspActionServer(ros::NodeHandle* node_handler, std::string action_server_name,
                      std::string manipulator_group_name);

  private:
    // Declare the SimpleActionServer used to execute grasps
    actionlib::SimpleActionServer<grip_core::GraspAction> action_server_;
    // Interface to control the move_group corresponding to the manipulator (should be defined in the SRDF file)
    moveit::planning_interface::MoveGroupInterface move_group_interface_;
    // Declare and initialise a GraspCommand msg used by the framework and composing the goal of the action
    grip_core::GraspCommand goal_grasp_command_;
    // Declare and initialise a ManipulatorState msg used to define the pregrasp or grasp
    grip_core::ManipulatorState manipulator_state_;
    // Declare and initialise the message containing the outcome of the action (defined in Grasp.action)
    grip_core::GraspResult action_result_;
    // Declare an object mapping joint variable names to values. Used to set the target of the move_group
    std::map<std::string, double> target_joints_;
    // Declare and initialise a GraspFeedback containing current joint_state of the manipulator and the grasp state
    grip_core::GraspFeedback action_feedback_;
    // Declare and initialise a boolean storing the state of the server
    bool busy_ = false;

    // Internal method executing all the steps required when receiving a new goal or preempting an action
    void goal_callback();
    void preempt_callback();
};

#endif  // GRIP_CORE_MOVEIT_GRASP_ACTION_SERVER_H
