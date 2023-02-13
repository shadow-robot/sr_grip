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
from grip_core.utils.manager_utils import reinitialise_managers


class ReinitialiseManagers(smach.State):

    """
        Reset the managers, i.e. clear them and fill them with messages set in the corresponding editors
    """

    def __init__(self, js_file, pose_file, traj_file, outcomes=["success", "failure"], input_keys=[], output_keys=[],
                 io_keys=[]):
        """
            Initialise the attributes of the class

            @param js_file: Path to a YAML file in which joint states are defined
            @param pose_file: Path to a YAML file in which poses and/or robot poses are defined
            @param traj_file: Path to a YAML file in which trajectories are defined
            @param outcomes: Possible outcomes of the state. Default "success" and "failure"
            @param input_keys: List enumerating all the inputs that a state needs to run
            @param output_keys: List enumerating all the outputs that a state provides
            @param io_keys: List enumerating all objects to be used as input and output data
        """
        smach.State.__init__(self, outcomes=outcomes, output_keys=output_keys, input_keys=input_keys, io_keys=io_keys)
        # Get the path of the files
        self.defined_joint_states_file = js_file if js_file is not None else ""
        self.defined_poses_file = pose_file if pose_file is not None else ""
        self.defined_trajectories_file = traj_file if traj_file is not None else ""
        self.outcomes = outcomes

    def execute(self, userdata):
        """
            Reinitialise all the managers

            @param userdata: Input and output data that can be communicated to other states

            @return: - outcomes[-1] ("fail" by default) if any manager has not been reinitialised successfully
                     - outcomes[0] ("success" by default) otherwise
        """
        js_file = self.defined_joint_states_file
        pose_file = self.defined_poses_file
        traj_file = self.defined_trajectories_file
        try:
            res = reinitialise_managers(js_file, pose_file, traj_file)
        except rospy.ROSException as exception:
            rospy.logerr("Error while reinitialising the managers: {}".format(exception))
            return self.outcome[-1]

        # If everything is good then return the first outcome
        return self.outcomes[0] if res else self.outcomes[-1]
