#!/usr/bin/env python

# Copyright 2021 Shadow Robot Company Ltd.
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
import timeit


class WaitFor(smach.State):

    """
        Blocking state allowing you to control the flow of a state machine until a condition is met
    """

    def __init__(self, topic_name, timeout=None, outcomes=["success", "failure"], input_keys=[], output_keys=[],
                 io_keys=[]):
        """
            Initialise the attributes of the class

            @param topic_name: String stating the name of the topic containing a boolean message. If an empty string is
                               provided, only wait for a given amount of time set by timeout
            @param timeout: Optional parameter stating how long the state should block (in seconds)
            @param outcomes: Possible outcomes of the state. Default "success" and "failure"
            @param input_keys: List enumerating all the inputs that a state needs to run
            @param output_keys: List enumerating all the outputs that a state provides
            @param io_keys: List enumerating all objects to be used as input and output data
        """
        smach.State.__init__(self, outcomes=outcomes, output_keys=output_keys, input_keys=input_keys, io_keys=io_keys)
        self.topic_name = None if not topic_name else topic_name if topic_name.startswith("/") else "/" + topic_name
        self.timeout = timeout if timeout != "None" else None
        self.outcomes = outcomes

    def execute(self, userdata):
        """
            Wait for a message sent by a specific topic or simply wait for a specific amount of time

            @param userdata: Input and output data that can be communicated to other states

            @return: - outcomes[-1] ("fail" by default) if no message has been received within the time limit
                     - outcomes[0] ("success" by default) otherwise
        """
        # If a topic name has been provided
        if self.topic_name is not None:
            # Display an informative message
            rospy.loginfo("Waiting for a message coming from the topic {}...".format(self.topic_name))
            # Wait for the message for the given amount of time
            try:
                message = rospy.wait_for_message(self.topic_name, rospy.AnyMsg, timeout=self.timeout)
            # If timing out the then display a message and quit
            except rospy.ROSException:
                rospy.logerr("Could not receive any message within {} seconds!".format(self.timeout))
                return self.outcomes[-1]
            # If a message is received, then return the first outcome otherwise return the last one
            return self.outcomes[0] if message else self.outcomes[-1]
        # In case no topic name is provided and a value of timeout is provided
        elif self.timeout is not None:
            start = timeit.default_timer()
            # Most trustworth way to really have the amount of time expected
            while timeit.default_timer() - start < self.timeout:
                continue

        # If everything is good then return the first outcome
        return self.outcomes[0]
