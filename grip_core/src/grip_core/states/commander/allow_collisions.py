#!/usr/bin/env python3

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

from grip_core.srv import ModifyACM
import rospy
import smach


class AllowCollisions(smach.State):

    """
        State allowing/disallowing collisions involving the robot with its environment
    """

    def __init__(self, group_name, allow, collision, objects=[], outcomes=["success", "failure"], input_keys=[],
                 output_keys=[], io_keys=["commanders"]):
        """
            Initialise the attributes of the class
            @param group_name: Name of the move group for which we want to modify collision checks. If set to "", the
                               ACM will be modified for all the links that compose each configured commander
            @param allow: Specify whether the state should allow or disallow collision check for the group
            @param collision_type: Kind of modification to be brought to the ACM (self collision or object collision)
            @param objects: Optional list of objects we want to allow the manipulator to collide with.
                            If left empty all added objects will be considered
            @param outcomes: Possible outcomes of the state. Default "success" and "fail"
            @param input_keys: List enumerating all the inputs that a state needs to run
            @param output_keys: List enumerating all the outputs that a state provides
            @param io_keys: List enumerating all objects to be used as input and output data
        """
        smach.State.__init__(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys, io_keys=io_keys)
        # Make sure the service is ready to be used
        rospy.wait_for_service("/modify_acm")
        # Proxy to the ACM manager to change the ACM
        self.modify_acm = rospy.ServiceProxy("modify_acm", ModifyACM)
        # Get the modification (equivalent to a switch statement)
        self.modification = {"self collision": 0, "object collision": 1}.get(collision, None)
        # If collision is not supported display an error message
        if self.modification is None:
            rospy.logerr("The parameter collision_type must be either \"self collision\" or \"object-collision\"")
        # Store the objects to be modified
        self.objects = objects
        # Store the option
        self.allow_collision = True if allow == "True" else False
        # Store the group for which the ACM must be modified
        self.group_name = group_name
        # Store the outcomes
        self.outcomes = outcomes

    def execute(self, userdata):
        """
            Modify the ACM in order to allow or disallow collisions when using MoveIt!
            @param userdata: Input and output data that can be communicated to other states
            @return: - outcomes[-1] ("fail" by default) if an error occurs when modifying the ACM
                     - outcomes[0] otherwise
        """
        # Sanity check during execution
        if self.modification is None:
            rospy.logerr("The parameter collision_type must be either \"self collision\" or \"object-collision\"")
            return self.outcomes[-1]
        # List that is going to get all the robot links for which the change should apply
        robot_links = list()
        # Groups for which the change should be applied
        group_names = list()
        # If no group is specified
        if not self.group_name:
            # Go over all the configured commanders
            for commander in userdata.commanders.values():
                # Get their name
                group_names.append(commander._name)
        # Otherwise store the group name
        else:
            group_names.append(self.group_name)
        # For each group, get the associated links
        for group in group_names:
            robot_links += userdata.commanders.values()[0]._robot_commander.get_link_names(group)

        # If nothing has been retrieved dispaly and error message and sttop here
        if not robot_links:
            rospy.logerr("Could not retrieve the robot links from the commanders...")
            return self.outcomes[-1]

        # Call the service that modifies the current ACM
        response = self.modify_acm(self.modification, robot_links, self.objects, self.allow_collision, True)
        # If anything went wrong on the service side display an error message and fail the state
        if not response.success:
            rospy.logerr("An error ocurred while modifying the allowed collisions")
            return self.outcomes[-1]
        # If everything works fine, return the corresponding outcome
        return self.outcomes[0]
