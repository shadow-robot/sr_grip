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

import smach


class Counter(smach.State):

    """
        State creating or incrementing/decrementing a counter
    """

    def __init__(self, initial_value=0, end_value=None, output="counter", outcomes=["success", "finished"],
                 input_keys=[], output_keys=[], io_keys=[]):
        """
            Initialise the attributes of the class

            @param initial_value: Initial value (integer) of the counter. Default is 0
            @param end_value: Value (integer) for which the state is going to return the outcome "finished"
            @param output: String, specifying the name to set to the counter. Default is "counter"
            @param outcomes: Possible outcomes of the state. Default "success" and "finished"
            @param input_keys: List enumerating all the inputs that a state needs to run
            @param output_keys: List enumerating all the outputs that a state provides
            @param io_keys: List enumerating all objects to be used as input and output data
        """
        # Initialise the state
        smach.State.__init__(self, outcomes=outcomes, io_keys=io_keys, input_keys=input_keys, output_keys=output_keys)
        self.current_value = initial_value
        self.end_value = end_value
        self.counter_name = output
        # Outcomes of the state
        self.outcomes = outcomes

    def execute(self, userdata):
        """
            Increment/decrement the internal variable named current_value until it reaches the value of end_value

            @param userdata: Input and output data that can be communicated to other states

            @return: - outcomes[-1] ("finished" by default) if the current internal value is equal to end_value
                     - outcomes[0] otherwise
        """
        if self.current_value == self.end_value:
            return self.outcomes[-1]

        userdata[self.counter_name] = self.current_value
        # Increment/decrement the current value
        if self.current_value < self.end_value:
            self.current_value += 1
        else:
            self.current_value -= 1
        return self.outcomes[0]
