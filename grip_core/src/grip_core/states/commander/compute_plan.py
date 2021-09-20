#!/usr/bin/env python3

# Copyright 2019 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

import rospy
import smach
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from grip_core.utils.manager_utils import get_add_service_from_type, get_retrieve_service_from_type


class ComputePlan(smach.State):

    """
        State performing motion planning from a starting to a target state
    """

    def __init__(self, group_name, target_type, target_name, plan_name="", starting_type="", starting_name="",
                 outcomes=["success", "failure"], input_keys=[], output_keys=[], io_keys=["commanders"]):
        """
            Initialise the attributes of the class

            @param group_name: Name of the group that should be used to plan
            @param target_type: Target state's type. Can be any of {"", "pose", "joint state"}
            @param target_name: Target state's name. If target_type is "", the target state is fetched from the userdata
                                If target_type is set to a non empty value, this parameter is used as the key for
                                retrieving it from the corresponding manager.
            @param plan_name: Name that will be given to the computed plan. If empty, then the plan is internally stored
                              by the commander. If set to an non empty string, then the plan is sent to its manager.
            @param starting_type: Starting state's type. Can be any of {"", "pose", "joint state"}
            @param starting_name: Starting state's name. If starting_type is "", the target state is fetched from the
                                  userdata. If starting_type is set to a non empty value, this parameter is used as the
                                  key fo retrieving it from the corresponding manager. If both this parameter and
                                  starting_type are empty then use the current robot's state.
            @param outcomes: Possible outcomes of the state. Default "success" and "failure"
            @param input_keys: List enumerating all the inputs that a state needs to run
            @param output_keys: List enumerating all the outputs that a state provides
            @param io_keys: List enumerating all objects to be used as input and output data
        """
        smach.State.__init__(self, outcomes=outcomes, io_keys=io_keys, input_keys=input_keys, output_keys=output_keys)
        # Get the proper commander
        self.commander_name = group_name
        # Optional name provided to the output plan
        self.plan_name = plan_name
        # Type of the target state
        self.target_type = target_type
        # Name of the target state (if any)
        self.target_name = target_name
        # Type of the starting state
        self.starting_type = starting_type
        # Name (if any) of the starting state
        self.starting_name = starting_name
        # Get the outcomes
        self.outcomes = outcomes
        # Initialise all the services that might be required to retrieve the messages
        self.get_starting_message = get_retrieve_service_from_type(starting_type) if starting_type else None
        self.get_target_message = get_retrieve_service_from_type(target_type) if target_type else None
        # Get the service to send the computed plan to the manager
        self.add_computed_plan = get_add_service_from_type("plan") if plan_name else None

    @staticmethod
    def joint_state_to_robot_state(joint_state):
        """
            Create a RobotState message from a JointState message or a dictionary mapping joint names to their values

            @param joint_state: JointState message or dictionary with the following format {'joint_name': joint_value}
            @return: RobotState message corresponding to the input value
        """
        if isinstance(joint_state, dict):
            joint_state_message = JointState()
            joint_state_message.name = joint_state.keys()
            joint_state_message.position = joint_state.values()
        else:
            joint_state_message = joint_state
        moveit_robot_state = RobotState()
        moveit_robot_state.joint_state = joint_state_message
        return moveit_robot_state

    def execute(self, userdata):
        """
            Compute the plan from the starting to the target state using MoveIt!

            @param userdata: Input and output data that can be communicated to other states

            @return: - outcomes[-1] ("fail" by default) if an error occurs when computing the plan or retrieving a state
                     - outcomes[0] ("success" by default) otherwise
        """
        # Get the commander
        commander = userdata.commanders[self.commander_name]
        # Depending on the state's initialization parameters, get the starting message
        if self.starting_type == "joint state":
            starting_message = self.get_starting_message(self.starting_name).joint_state
        elif self.starting_type == "pose":
            starting_message = self.get_starting_message(self.starting_name).pose_stamped
        # If the starting state has a name but no type, it means we need to have a look inside the userdata
        elif not self.starting_type and self.starting_name:
            # Make sure the starting state has previously been stored in the userdata
            if self.starting_name not in userdata:
                rospy.logerr("The starting state does not seem to be stored in the userdata...")
                return self.outcomes[-1]
            else:
                starting_message = userdata[self.starting_name]

        # Make sure the  starting_robot_state variable follow the proper format (i.e. RobotState msg)
        if self.starting_type == "joint state":
            starting_robot_state = self.joint_state_to_robot_state(starting_message)
        elif self.starting_type == "pose":
            starting_robot_state = self.joint_state_to_robot_state(commander.get_ik(starting_message))
        elif not self.starting_type and self.starting_name:
            # If the message is retrieved from the userdata we need to make sure it has a valid format
            if isinstance(starting_message, PoseStamped):
                starting_robot_state = self.joint_state_to_robot_state(commander.get_ik(starting_message))
            elif isinstance(starting_message, JointState):
                starting_robot_state = self.joint_state_to_robot_state(starting_message)
            elif isinstance(starting_message, RobotState):
                starting_robot_state = starting_message
            else:
                rospy.logerr("The selected starting state must be either a PoseStamped, a JointState"
                             " or a RobotState message!")
                return self.outcomes[-1]
        else:
            # In that particular case, the starting robot state is the current robot state during execution
            starting_robot_state = None

        # Depending on the state's initialization parameters, get the target message
        if self.target_type == "joint state":
            target_message = self.get_target_message(self.target_name).joint_state
        elif self.target_type == "pose":
            target_message = self.get_target_message(self.target_name).pose_stamped
        elif not self.target_type and self.target_name:
            if self.target_name not in userdata:
                rospy.logerr("The target state does not seem to be stored in the userdata...")
                return self.outcomes[-1]
            else:
                target_message = userdata[self.target_name]
                if not isinstance(target_message, PoseStamped) and not isinstance(target_message, JointState):
                    rospy.logerr("The selected target state must be either a PoseStamped or a JointState message!")
                    return self.outcomes[-1]
        else:
            rospy.logerr("Could not find the target message")
            return self.outcomes[-1]

        # Compute the plan
        if isinstance(target_message, JointState):
            plan = commander.plan_to_joint_value_target(target_message, custom_start_state=starting_robot_state)
        elif isinstance(target_message, PoseStamped):
            plan = commander.plan_to_pose_target(target_message, custom_start_state=starting_robot_state)

        # This condition checks whether a plan has been found
        if plan.joint_trajectory.joint_names == list():
            rospy.logerr("Fail to find a plan")
            return self.outcomes[-1]

        # If the message needs to be sent to its manager
        if self.plan_name:
            response = self.add_computed_plan(plan, self.plan_name)
            if not response.success:
                rospy.logerr("Cannot store the computed plan named {}".format(self.plan_name))
                return self.outcomes[-1]

        return self.outcomes[0]
