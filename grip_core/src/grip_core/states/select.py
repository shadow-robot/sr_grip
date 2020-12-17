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

import smach
import rospy
from grip_core.srv import (GetStandardisedGrasp, GetJointState, GetPoseStamped,
                                        GetMoveitPlan, GetJointTrajectory)


class Select(smach.State):

    """
        State selecting an object from the corresponding manager
    """

    def __init__(self, message_type, message_names=None, outcomes=["success", "failure"], input_keys=[], output_keys=[],
                 io_keys=[]):
        """
            Initialise the attributes of the class

            @param message_type: Type of message to select. Legal values are {"joint_state", "trajectory", "plan",
                                                                              "pose", "grasp", "pregrasp", "postgrasp"}
            @param message_names: Optional names of the messages you want to retrieve and select one after the other
            @param outcomes: Possible outcomes of the state. Default "success" and "fail"
            @param input_keys: List enumerating all the inputs that a state needs to run
            @param output_keys: List enumerating all the outputs that a state provides
            @param io_keys: List enumarting all objects to be used as input and output data
        """
        # Initialise the state
        smach.State.__init__(self, outcomes=outcomes, io_keys=io_keys, input_keys=input_keys, output_keys=output_keys)
        # Store the type of message to retrieve from the manager
        self.message_type = message_type
        # Get the names of the messages to extract
        self.message_names = message_names
        # Extract from the previous attribute the maximum number of the requests
        # If no name is provided, we are going to retrieve anonymous messages (and then we don't know the upper limit)
        self.number_max_request = float("inf") if self.message_names is None else len(self.message_names)
        # Number of times the state is executed (will allow to know which message to return amon the names provided)
        self.times_called = 0
        # Set the service to retrieve the proper message to None
        self.get_object_service = None
        # Initialise the above attribute with the proper service
        self._initialise_correct_service()
        # Outcomes of the state
        self.outcomes = outcomes
        # Sanity check
        if self.get_object_service is None:
            rospy.logerr("Can not find a manager for type {}. Legal values are {}".format(
                message_type, {"joint_state", "trajectory", "plan", "pose", "grasp", "pregrasp", "postgrasp"}))

    def _initialise_correct_service(self):
        """
            Set the attribute responsible for retrieving messages from a manager to the corresponding service
        """
        # Depending on the value of message_type set the proper service
        if self.message_type == "joint_state":
            self.get_object_service = rospy.ServiceProxy("get_joint_state", GetJointState)
        elif self.message_type == "pose":
            self.get_object_service = rospy.ServiceProxy("get_pose", GetPoseStamped)
        elif "grasp" in self.message_type:
            self.get_object_service = rospy.ServiceProxy("get_grasp", GetStandardisedGrasp)
        elif self.message_type == "plan":
            self.get_object_service = rospy.ServiceProxy("get_plan", GetMoveitPlan)
        elif self.message_type == "trajectory":
            self.get_object_service = rospy.ServiceProxy("get_trajectory", GetJointTrajectory)

    def execute(self, userdata):
        """
            Selects a message and stores it to the userdata to be used in following states

            @param userdata: Input and output data that can be communicated to other states

            @return: - outcomes[-1] ("fail" by default) if all the requested messages can not be used by the states
                                   following this one, if a message cannot be accessed or if the manager is not found
                     - outcomes[0] otherwise
        """
        # If the first message has not been noticed by the user, during the state machine initialisation, kill the state
        # during execution
        if self.get_object_service is None:
            rospy.logerr("Cannot find a manager for the input message type.")
            return self.outcomes[-1]

        # The default message name is empty (anonymous grasp)
        message_name = ""
        # If names are provided and we haven't already gone through all of them, set the object name
        if self.message_names is not None and self.times_called < self.number_max_request:
            message_name = self.message_names[self.times_called]
        # But if we have already accessed all of them then means that nothing is working
        elif self.times_called >= self.number_max_request:
            rospy.logerr("All the requested {} have been selected!".format(self.message_type))
            return self.outcomes[-1]
        # Call the service to get the proper message
        requested_message = self.get_object_service(message_name)
        # Increment the number of times the state is executed
        self.times_called += 1
        # If we can not access the requested message then display an error message
        if not requested_message.success:
            rospy.logerr("Cannot retrieve the specified {}! Aborting the selection...".format(self.message_type))
            return self.outcomes[-1]
        # If the state is supposed to output a "selected_*message_type*" and the message type corresponds, then set
        # the proper field of the userdata
        if "selected_grasp" in self._output_keys and "grasp" in self.message_type:
            userdata.selected_grasp = requested_message.grasp_message
        elif "selected_pose" in self._output_keys and self.message_type == "pose":
            userdata.selected_pose = requested_message.pose_stamped
        elif "selected_trajectory" in self._output_keys and self.message_type == "trajectory":
            userdata.selected_trajectory = requested_message.joint_trajectory
        elif "selected_joint_state" in self._output_keys and self.message_type == "joint_state":
            userdata.selected_joint_state = requested_message.joint_state
        elif "selected_plan" in self._output_keys and self.message_type == "plan":
            userdata.selected_plan = requested_message.moveit_plan
        else:
            rospy.logwarn("The selected message will not be stored in the userdata of the state machine")
        return self.outcomes[0]
