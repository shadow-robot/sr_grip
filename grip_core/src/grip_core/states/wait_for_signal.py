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
from std_msgs.msg import Bool


class WaitForSignal(smach.State):

    """
        Blocking state allowing to control the flow of a state machine to some events.
    """

    def __init__(self, topic_name, timeout=None, outcomes=["success", "failure"], input_keys=[], output_keys=[],
                 io_keys=[]):
        """
            Initialise the attributes of the class

            @param topic_name: String stating the name of the topic containing a boolean message
            @param timeout: Optional parameter stating how long the state should block (in seconds)
            @param outcomes: Possible outcomes of the state. Default "success" and "fail"
            @param input_keys: List enumerating all the inputs that a state needs to run
            @param output_keys: List enumerating all the outputs that a state provides
            @param io_keys: List enumarting all objects to be used as input and output data
        """
        smach.State.__init__(self, outcomes=outcomes, output_keys=output_keys, input_keys=input_keys, io_keys=io_keys)
        self.topic_name = topic_name
        self.timeout = timeout
        self.outcomes = outcomes

    def execute(self, userdata):
        """
            Wait for a Bool message stating whether the state should keep blocking.

            @param userdata: Input and output data that can be communicated to other states

            @return: - outcomes[-1] ("fail" by default) if no message has been received within the time limit or
                                                       if the content of the message is "false"
                     - outcomes[0] ("success" by default) otherwise
        """
        # Wait for a message coming from a method or another state
        try:
            rospy.loginfo("Waiting for a signal coming from the topic {}...".format(self.topic_name))
            boolean_message = rospy.wait_for_message(self.topic_name, Bool, timeout=self.timeout)
        except rospy.ROSException:
            rospy.logerr("Could not receive any message within {} seconds!".format(self.timeout))
            return self.outcomes[-1]
        # If the message is receives, check the message's content to ensure that we can proceed to other states
        if not boolean_message.data:
            rospy.logerr("The content of the message indicates that something may be wrong! Aborting...")
            return self.outcomes[-1]

        # If everything is good then return the first outcome
        return self.outcomes[0]
