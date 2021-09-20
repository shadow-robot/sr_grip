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

import smach
import rospy
from grip_core.utils.manager_utils import get_retrieve_service_from_type


class Move(smach.State):

    """
        State allowing to move the robot according to a plan already computed
    """

    def __init__(self, group_name, plan_name="", outcomes=["success", "failure"], input_keys=[], output_keys=[],
                 io_keys=["commanders"]):
        """
            Initialise the attributes of the class

            @param group_name: Name of the group that should be moved
            @param plan_name: Specify which plan should be executed. Used if it should be retrieved from the manager or
                              from the userdata.
            @param outcomes: Possible outcomes of the state. Default "success" and "failure"
            @param input_keys: List enumerating all the inputs that a state needs to run
            @param output_keys: List enumerating all the outputs that a state provides
            @param io_keys: List enumerating all objects to be used as input and output data
        """
        smach.State.__init__(self, outcomes=outcomes, io_keys=io_keys, input_keys=input_keys, output_keys=output_keys)
        # Get the proper commander
        self.commander_name = group_name
        # Store the name of the plan we want to execute (empty for an anonymous one)
        self.plan_name = plan_name
        # Initialise the attribute storing the service to retrieve a plan if "selected_plan" is not in input_keys
        self.get_computed_plan = get_retrieve_service_from_type("plan") if plan_name else None
        # Store the possible outcomes
        self.outcomes = outcomes

    def execute(self, userdata):
        """
            Execute a plan already computed by a motion planner

            @param userdata: Input and output data that can be communicated to other states

            @return: - outcomes[-1] ("fail" by default) if an error occurs when requesting the plan or executing it
                     - outcomes[0] ("success" by default) otherwise
        """
        # Get the commander
        commander = userdata.commanders[self.commander_name]
        # Check whether we can retrieve a plan either through the commander, through the manager or through userdata
        # If plan_name is empty, then execute the latest plan computed by the commander
        if self.get_computed_plan is None:
            commander.execute()
            return self.outcomes[0]
        # If plan_name is part of userdata then get it
        elif self.plan_name in userdata:
            plan_to_execute = userdata[self.plan_name]
        # Otherwise get it from the manager
        else:
            requested_plan = self.get_computed_plan(self.plan_name)
            if not requested_plan.success:
                rospy.logerr("Plan {} can't be retrieved from the plan manager...". format(self.plan_name))
                return self.outcomes[-1]
            # Get the plan from the service's response
            plan_to_execute = requested_plan.moveit_plan

        # Use the commander to execute the already computed plan. Instruction in a try except in order to catch
        # any errors about execution
        try:
            commander.execute_plan(plan_to_execute)
        except Exception as exception:
            rospy.logerr("Cannot properly execute the plan: {}".format(exception))
            return self.outcomes[-1]
        # Successfull execution
        return self.outcomes[0]
