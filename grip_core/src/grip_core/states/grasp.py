#!/usr/bin/env python

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
from grip_core.msg import GraspCommand, GraspGoal
from grip_core.srv import GetStandardisedGrasp


class Grasp(smach.State):

    """
        State actuating the manipulator according to a predefined message
    """

    def __init__(self, grasp_type, grasp_name="", outcomes=["success", "failure"], input_keys=[], output_keys=[],
                 io_keys=["manipulator_controller_client", "max_torque"]):
        """
            Initialise the attributes of the class

            @param grasp_type: Specify the manipulator state to be executed, can be {"pregrasp", "grasp" or "postgrasp"}
            @param grasp_name: Specify the name of the grasp that should be retrieved from the manager
            @param outcomes: Possible outcomes of the state. Default "success" and "fail"
            @param input_keys: List enumerating all the inputs that a state needs to run
            @param output_keys: List enumerating all the outputs that a state provides
            @param io_keys: List enumarting all objects to be used as input and output data
        """
        smach.State.__init__(self, outcomes=outcomes, io_keys=io_keys, input_keys=input_keys, output_keys=output_keys)
        # Store the manipulator state to be executed
        self.grasp_type = grasp_type
        # Store the grasp name if we want to retrieve it from the manager
        self.grasp_name = grasp_name
        # Initialise the attribute storing the service to retrieve grasp if we need to
        self.get_grasp = None if "selected_grasp" in self._input_keys else rospy.ServiceProxy(
            "get_grasp", GetStandardisedGrasp)
        # Initialise the objects linked to the action
        self.grasp_goal = GraspGoal()
        self.grasp_cmd = GraspCommand()
        # Store the outcomes
        self.outcomes = outcomes

    def execute(self, userdata):
        """
            Execute a manipulator state defined in the selected grasp contained in the userdata

            @param userdata: Input and output data that can be communicated to other states

            @return: - outcomes[-1] ("fail" by default) if no grasp can be selected/retrieved or
                                                        if an error occured during the execution
                     - outcomes[0] otherwise
        """
        # Additional sanity check
        if "selected_grasp" not in self._input_keys and self.get_grasp is None:
            rospy.logerr("No grasp has been provided to the state machine or can be found by the manager.")
            return self.outcomes[-1]

        # If "selected_grasp" is in the input keys, set the grasp command grasp to the selected one
        if "selected_grasp" in self._input_keys:
            self.grasp_cmd.grasp = userdata.selected_grasp
        # Otherwise try to retrieve it from the manager and set the grasp command to the message
        else:
            requested_grasp = self.get_grasp(self.grasp_name)
            if not requested_grasp.success:
                rospy.logerr("Can not retrieve the required grasp from the manager.")
                return self.outcomes[-1]
            self.grasp_cmd.grasp = requested_grasp.grasp_message

        # Get the grasp client from the userdata
        self.grasp_client = userdata.manipulator_controller_client
        # Get the max torque from the userdata and store the information in the grasp command
        self.grasp_cmd.max_torque = userdata.max_torque

        # Execute the selected type of grasp
        if self.grasp_type == "pregrasp":
            if not self.execute_pregrasp():
                rospy.logerr("Couldn't execute pregrasp")
                return self.outcomes[-1]
        elif self.grasp_type == "grasp":
            if not self.execute_grasp():
                rospy.logerr("Couldn't execute grasp")
                return self.outcomes[-1]
        elif self.grasp_type == "postgrasp":
            if not self.execute_postgrasp():
                rospy.logerr("Couldn't execute postgrasp")
                return self.outcomes[-1]
        return self.outcomes[0]

    def execute_pregrasp(self):
        """
            Modify the grasp command to execute a pregrasp

            @return: - True if the grasp has been successfully applied
                     - False if an error occured in the process
        """
        try:
            # Change the field of the grasp command specifying the type of grasp
            self.grasp_cmd.grasp_state = self.grasp_cmd.PRE_GRASP_STATE
            # Fill the grasp goal command field with the grasp command
            self.grasp_goal.grasp_command = self.grasp_cmd
            # Send the goal and wait for result
            self.grasp_client.send_goal(self.grasp_goal)
            self.grasp_client.wait_for_result()
            # Get a result if and only if the goal is in the correct format
            result = self.grasp_client.get_result()
            # Make sure that the execution has been a success
            if not result.pregrasped:
                rospy.logerr("Pregrasp action server reports failure.")
                return False
        except rospy.ROSException:
            rospy.logerr("There was a problem executing the pre_grasp")
            return False
        return True

    def execute_grasp(self):
        """
            Modify the grasp command to execute a grasp

            @return: - True if the grasp has been successfully applied
                     - False if an error occured in the process
        """
        try:
            # Change the field of the grareleasesp command specifying the type of grasp
            self.grasp_cmd.grasp_state = self.grasp_cmd.GRASP_STATE
            # Fill the grasp goal command field with the grasp command
            self.grasp_goal.grasp_command = self.grasp_cmd
            # Send the goal and wait for result
            self.grasp_client.send_goal(self.grasp_goal)
            self.grasp_client.wait_for_result()
            # Get a result if and only if the goal is in the correct format
            result = self.grasp_client.get_result()
            # Make sure that the execution has been a success
            if not result.grasped:
                rospy.logerr("Grasp action server reports failure.")
                return False
        except rospy.ROSException:
            rospy.logerr("There was a problem executing the grasp")
            return False
        return True

    def execute_postgrasp(self):
        """
            Modify the grasp command to execute a postgrasp

            @return: - True if the grasp has been successfully applied
                     - False if an error occured in the process
        """
        try:
            # Change the field of the grasp command specifying the type of grasp
            self.grasp_cmd.grasp_state = self.grasp_cmd.POST_GRASP_STATE
            # Fill the grasp goal command field with the grasp command
            self.grasp_goal.grasp_command = self.grasp_cmd
            # Send the goal and wait for result
            self.grasp_client.send_goal(self.grasp_goal)
            self.grasp_client.wait_for_result()
            # Get a result if and only if the goal is in the correct format
            result = self.grasp_client.get_result()
            # Make sure that the execution has been a success
            if not result.postgrasped:
                rospy.logerr("Grasp action server reports failure.")
                return False
        except rospy.ROSException:
            rospy.logerr("There was a problem executing the postgrasp")
            return False
        return True
