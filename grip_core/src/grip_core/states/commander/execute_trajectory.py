#!/usr/bin/env python3

# Copyright 2021, 2023 Shadow Robot Company Ltd.
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
from grip_core.utils.manager_utils import get_retrieve_service_from_type


class ExecuteTrajectory(smach.State):

    """
        Execute a joint trajectory
    """

    def __init__(self, group_name, trajectory_name="", outcomes=["success", "failure"], input_keys=[], output_keys=[],
                 io_keys=["commanders"]):
        """
            Initialise the attributes of the class

            @param group_name: Name of the group that should execute the trajectory
            @param target_type: Target state's type. Can be any of {"", "pose", "joint state"}
            @param trajectory_name: Name of the trajectory to execute. The name will be looked up into the userdata and
                                    the corresponding manager.
            @param outcomes: Possible outcomes of the state. Default "success" and "failure"
            @param input_keys: List enumerating all the inputs that a state needs to run
            @param output_keys: List enumerating all the outputs that a state provides
            @param io_keys: List enumerating all objects to be used as input and output data
        """
        smach.State.__init__(self, outcomes=outcomes, output_keys=output_keys, input_keys=input_keys, io_keys=io_keys)
        # Get the proper commander
        self.commander_name = group_name
        self.trajectory_name = trajectory_name
        # Get the service to get the trajectory
        self.get_trajectory = get_retrieve_service_from_type("trajectory")
        self.outcomes = outcomes

    def execute(self, userdata):
        """
            Fetch and execute the trajectory

            @param userdata: Input and output data that can be communicated to other states

            @return: - outcomes[-1] ("fail" by default) if the trajectory cannot be found or executed
                     - outcomes[0] ("success" by default) otherwise
        """
        # Get the commander
        commander = userdata.commanders[self.commander_name]

        # If trajectory_name is part of userdata then get it
        if self.trajectory_name in userdata:
            trajectory_to_execute = userdata[self.trajectory_name]
        # Otherwise get it from the manager
        else:
            requested_trajectory = self.get_trajectory(self.trajectory_name)
            if not requested_trajectory.success:
                rospy.logerr("Trajectory {} can't be retrieved from its manager...". format(self.trajectory_name))
                return self.outcomes[-1]
            # Get the trajectory from the service's response
            trajectory_to_execute = requested_trajectory.joint_trajectory

        # Use the commander to execute the atrajectory. Instruction in a try except in order to catch
        # any errors about execution
        try:
            commander.run_joint_trajectory(trajectory_to_execute)
        except Exception as exception:
            rospy.logerr("Cannot properly execute the trajectory: {}".format(exception))
            return self.outcomes[-1]
        # Successfull execution
        return self.outcomes[0]
