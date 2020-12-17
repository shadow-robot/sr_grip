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

from moveit_msgs.msg import PlanningScene
from grip_core.srv import GetModifiedACM
import rospy
import smach


class AllowManipulatorCollisions(smach.State):

    """
        State changing the ACM to allow collision between the manipulator and other elements
    """

    def __init__(self, allow, collision_type="", objects=[], outcomes=["success", "failure"], input_keys=[],
                 output_keys=[], io_keys=[]):
        """
            Initialise the attributes of the class
            @param allow: States whether the state should allow or disallow collision check for the hand
            @param collision_type: Kind of modification to be brought to the ACM (must be either '' i.e simple getter,
                                                                                 'self-collision' or 'object-collision')
            @param objects: Optional list of objects we want to allow the manipulator to collide with.
                            If left empty all added objects will be considered
            @param outcomes: Possible outcomes of the state. Default "success" and "fail"
            @param input_keys: List enumerating all the inputs that a state needs to run
            @param output_keys: List enumerating all the outputs that a state provides
            @param io_keys: List enumerating all objects to be used as input and output data
        """
        smach.State.__init__(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys, io_keys=io_keys)
        # Initialise the PlanningScene publisher
        self.planning_scene_publisher = rospy.Publisher("planning_scene", PlanningScene, queue_size=5, latch=True)
        # Make sure the service is ready to be use
        rospy.wait_for_service("/get_modified_acm")
        # Proxy to the ACm manager to get modified ACMs
        self.get_modified_acm = rospy.ServiceProxy("get_modified_acm", GetModifiedACM)
        # Get the modification type (equivalent to a switch statement)
        self.modification_type = {"": 0, "self-collision": 1, "object-collision": 2}.get(collision_type, None)
        # If collision_type is not supported display an error message
        if self.modification_type is None:
            rospy.logerr("The parameter collision_type can only be {'', 'self-collision', 'object-collision'}")
        # Store the objects to be modified
        self.objects = objects
        # Store the option
        self.allow_collision = allow
        # Store the outcomes
        self.outcomes = outcomes

    def execute(self, userdata):
        """
            Get the modified ACM and publish it in order to update the allowed collisions when moveit plans
            @param userdata: Input and output data that can be communicated to other states
            @return: - outcomes[-1] ("fail" by default) if an error occurs when modifying the ACM
                     - outcomes[0] otherwise
        """
        # Sanity check during execution
        if self.modification_type is None:
            rospy.logerr("The parameter collision_type can only be {'', 'self-collision', 'object-collision'}")
            return self.outcomes[-1]
        # Call the service allowing to modify an updated ACM with all the interactively added objects
        response = self.get_modified_acm(self.modification_type, self.objects, self.allow_collision)
        # If anything went wrong on the service side display an error message and fail the state
        if not response.success:
            rospy.logerr("Could not allow collisions with the provided parameters")
            return self.outcomes[-1]
        # If everything works fine, we just set the output ACM in a PlanningScene message and send it to Moveit
        planning_scene_diff = PlanningScene()
        planning_scene_diff.is_diff = True
        try:
            planning_scene_diff.allowed_collision_matrix = response.acm
            self.planning_scene_publisher.publish(planning_scene_diff)
            return self.outcomes[0]
        except rospy.ROSException as exception:
            rospy.logerr("Can not change collision state of the scene: {}".format(exception))
            return self.outcomes[-1]
