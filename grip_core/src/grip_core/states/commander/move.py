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
from grip_core.srv import GetMoveitPlan


class Move(smach.State):

    """
        State allowing to move the robot according to a plan already computed
    """

    def __init__(self, plan_name="", outcomes=["success", "failure"], input_keys=[], output_keys=[],
                 io_keys=["arm_commander"]):
        """
            Initialise the attributes of the class

            @param plan_name: Specify which plan should be executed. Used if it should be retrieved from the manager.
            @param outcomes: Possible outcomes of the state. Default "success" and "fail"
            @param input_keys: List enumerating all the inputs that a state needs to run
            @param output_keys: List enumerating all the outputs that a state provides
            @param io_keys: List enumerating all objects to be used as input and output data
        """
        smach.State.__init__(self, outcomes=outcomes, io_keys=io_keys, input_keys=input_keys, output_keys=output_keys)
        # Store the name of the plan we want to execute (empty for an anonymous one)
        self.plan_name = plan_name
        # Initialise the attribute storing the service to retrieve a plan if "selected_plan" is not in input_keys
        self.get_computed_plan = rospy.ServiceProxy(
            "get_plan", GetMoveitPlan) if "selected_plan" not in self._input_keys else None
        # Store the possible outcomes
        self.outcomes = outcomes

    def execute(self, userdata):
        """
            Execute a plan already computed by a motion planner

            @param userdata: Input and output data that can be communicated to other states

            @return: - outcomes[-1] ("fail" by default) if an error occurs when requesting the plan or executing it
                     - outcomes[0] ("success" by default) otherwise
        """
        # Check whether we can retreive a plan either through the userdata or through the manager
        if self.get_computed_plan is None:
            plan_to_execute = userdata.selected_plan
        else:
            requested_plan = self.get_computed_plan(self.plan_name)
            if not requested_plan.success:
                rospy.logerr("No plan can be retrieved through the plan manager")
                return self.outcomes[-1]
            plan_to_execute = requested_plan.moveit_plan

        # Use the arm commander to execute the already computed plan. Instruction in a try except in order to catch
        # any errors about execution
        try:
            userdata.arm_commander.execute_plan(plan_to_execute)
        except Exception as exception:
            rospy.logerr("Cannot properly execute the plan: {}".format(exception))
            return self.outcomes[-1]
        # Successfull execution
        return self.outcomes[0]
