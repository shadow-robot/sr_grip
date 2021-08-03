#!/usr/bin/env python

# Copyright 2019, 2021 Shadow Robot Company Ltd.
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
from grip_core.utils.manager_utils import get_retrieve_service_from_type


class Select(smach.State):

    """
        State selecting an object from the corresponding manager
    """

    def __init__(self, input, input_type, output, outcomes=["success", "failure"], input_keys=[], output_keys=[],
                 io_keys=[]):
        """
            Initialise the attributes of the class

            @param input: Name of the message to select
            @param input_type: Name of the manager to get a message from. Valid values are: joint state, plan, pose or
                               trajectory.
            @param output: Name under which the returned message will be stored in the userdata
            @param outcomes: Possible outcomes of the state. Default "success" and "failure"
            @param input_keys: List enumerating all the inputs that a state needs to run
            @param output_keys: List enumerating all the outputs that a state provides
            @param io_keys: List enumerating all objects to be used as input and output data
        """
        # Initialise the state
        smach.State.__init__(self, outcomes=outcomes, io_keys=io_keys, input_keys=input_keys, output_keys=output_keys)
        self.input = input
        self.output = output
        # If needed, initialize the services required to retrieve and send messages to the managers
        self.message_getter = get_retrieve_service_from_type(input_type) if input_type else None
        # Outcomes of the state
        self.outcomes = outcomes
        # Sanity check
        if self.message_getter is None:
            rospy.logerr("Can not find a manager for type {}.".format(input_type))

    def execute(self, userdata):
        """
            Selects a message and stores it to the userdata to be used in the following states

            @param userdata: Input and output data that can be communicated to other states

            @return: - outcomes[-1] ("fail" by default) if the requested message can not be found in the manager
                     - outcomes[0] otherwise
        """
        # If the first error log has not been seen by the user, return the last outcome and stop the execution
        if self.message_getter is None:
            rospy.logerr("Cannot find a manager for the input message type.")
            return self.outcomes[-1]
        # Retrieve the response of the service
        message_response = self.message_getter(self.input)
        # Make sure all is good
        if not message_response.success:
            return self.outcomes[-1]
        # Make sure to only extract the proper field out of the response of the manager
        retrieved_msg = message_response.__getattribute__(message_response.__slots__[0])
        # Store it into the userdata
        userdata[self.output] = retrieved_msg
        return self.outcomes[0]
