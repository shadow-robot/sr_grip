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
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import rospy
from grip_core.srv import GetJointState, GetPoseStamped, AddMoveitPlan


class Plan(smach.State):

    """
        State performing motion planning from a starting to a target state
    """

    def __init__(self, group_name, target_state_type, target_state_name="", plan_name="", starting_state_type="",
                 starting_state_name="", outcomes=["success", "failure"], input_keys=[], output_keys=[],
                 io_keys=["commanders"]):
        """
            Initialise the attributes of the class

            @param target_state_type: Target state's type. Can be any of {"pose", "joint_state"}
            @param target_state_name: Optional target state's name. Used to retrieve it using the corresponding manager.
                                      If set to "userdata" then try to load the field "selected_<target_state_type>"
                                      from the userdata. Default is "", meaning that it will retrieve
                                      the latest corresponding anonymous message.
            @param plan_name: Name that will be given to the computed plan. Can be empty to make it anonymous.
            @param starting_state_type: Starting state's type. Can be any of {"pose", "joint_state", "grasp",
                                                                              "pregrasp", "postgrasp"}
            @param starting_state_name: Optional starting state's name. Used to retrieve it using the
                                        corresponding manager. If set to "userdata" then try to load the field
                                        "selected_<starting_state_type>" from the userdata. Default is "", meaning that
                                        it will retrieve the latest corresponding anonymous message.
            @param outcomes: Possible outcomes of the state. Default "success" and "fail"
            @param input_keys: List enumerating all the inputs that a state needs to run
            @param output_keys: List enumerating all the outputs that a state provides
            @param io_keys: List enumerating all objects to be used as input and output data
        """
        smach.State.__init__(self, outcomes=outcomes, io_keys=io_keys, input_keys=input_keys, output_keys=output_keys)
        # Get the proper commander
        self.commander_name = group_name
        # Optional name provided to the output plan
        self.plan_name = plan_name
        # Type of the target state
        self.target_state_type = target_state_type
        # Name of the target state (if any)
        self.target_state_name = target_state_name
        # Type of the starting state
        self.starting_state_type = starting_state_type
        # Name (if any) of the starting state
        self.starting_state_name = starting_state_name
        # Get the outcomes
        self.outcomes = outcomes
        # Initialise all the services that might be required to None
        self.get_starting_state_message = None
        self.get_target_state_message = None
        self.add_computed_plan = None

        if target_state_type or starting_state_type:
            # Initialise the proper services
            self._initialise_correct_services()

        # Get the service to send the computed plan to the manager
        if plan_name:
            self.add_computed_plan = rospy.ServiceProxy("add_plan", AddMoveitPlan)

    def _initialise_correct_services(self):
        """
            Initialise the proper services to get the target and starting state required to plan
        """
        # If we need to lookup a joint state in the manager (meaning that it is not in userdata), initialise the service
        if self.target_state_type == "joint_state":
            self.get_target_state_message = rospy.ServiceProxy("get_joint_state", GetJointState)
        # Same thing for the pose
        elif self.target_state_type == "pose":
            self.get_target_state_message = rospy.ServiceProxy("get_pose", GetPoseStamped)

        # Same idea for the starting state
        if self.starting_state_type == "joint_state":
            self.get_starting_state_message = rospy.ServiceProxy("get_joint_state", GetJointState)
        elif self.starting_state_type == "pose" and self.starting_state_name != "userdata":
            self.get_starting_state_message = rospy.ServiceProxy("get_pose", GetPoseStamped)

    @staticmethod
    def joint_state_to_robot_state(joint_state):
        """
            Create a RobotState message from a JointState message or a dictionary mapping joint names to their values

            @param joint_state: JointState message or dictionary with the following format {'joint_name': joint_value}
        """
        if isinstance(joint_state, dict):
            joint_state_message = JointState()
            joint_state_message.name = joint_state.keys()
            joint_state_message.position = joint_state.values()
        else:
            joint_state_message = joint_state
        moveit_robot_state = RobotState()
        moveit_robot_state.joint_state = joint_state_message
        return moveit_robot_state

    def execute(self, userdata):
        """
            Compute the motion planning from the starting to the target state

            @param userdata: Input and output data that can be communicated to other states

            @return: - outcomes[-1] ("fail" by default) if an error occurs when computing the plan or retrieving a state
                     - outcomes[0] ("success" by default) otherwise
        """
        print(userdata)
        print(userdata["commanders"])
        commander = userdata.commanders[self.commander_name]
        # Depending on the starting state type and the value of the attribute storing the client of the proper service,
        # determines the starting message (either to be retrieved from the manager or from the userdata)
        if self.get_starting_state_message is None and "grasp" in self.starting_state_type:
            starting_message = userdata.selected_grasp
        elif "grasp" in self.starting_state_type:
            starting_message = self.get_starting_state_message(self.starting_state_name).grasp_message
        elif self.get_starting_state_message is None and self.starting_state_type == "joint_state":
            starting_message = userdata.selected_joint_state
        elif self.starting_state_type == "joint_state":
            starting_message = self.get_starting_state_message(self.starting_state_name).joint_state
        elif self.get_starting_state_message is None and self.starting_state_type == "pose":
            starting_message = userdata.selected_pose
        elif self.starting_state_type == "pose":
            starting_message = self.get_starting_state_message(self.starting_state_name).pose_stamped
        # If all the aforementioned conditions do not work and the starting state type is not empty, then it means that
        # something is wrong
        elif self.starting_state_type != "":
            rospy.logerr("Impossible to find the required message defining the starting state!")
            return self.outcomes[-1]

        # Store the starting robot state with respect to the starting state meaning that we get the joint state and
        # transform it to a RobotState message
        if self.starting_state_type == "pregrasp":
            starting_robot_state = self.joint_state_to_robot_state(commander.get_ik(starting_message.pregrasp.pose))
        elif self.starting_state_type == "grasp":
            starting_robot_state = self.joint_state_to_robot_state(commander.get_ik(starting_message.grasp.pose))
        elif self.starting_state_type == "postgrasp":
            starting_robot_state = self.joint_state_to_robot_state(commander.get_ik(starting_message.postgrasp.pose))
        elif self.starting_state_type == "joint_state":
            starting_robot_state = self.joint_state_to_robot_state(starting_message)

        elif self.starting_state_type == "pose":
            starting_robot_state = self.joint_state_to_robot_state(commander.get_ik(starting_message))
        else:
            # In that particular case, the starting robot state is the current robot state during execution
            starting_robot_state = None

        # Depending on the target state type and the value of the attribute storing the client of the proper service,
        # determines the target message (either to be retrieved from the manager or from the userdata)
        if self.get_target_state_message is None and "grasp" in self.target_state_type:
            target_message = userdata.selected_grasp
        elif "grasp" in self.target_state_type:
            target_message = self.get_target_state_message(self.target_state_name).grasp_message
        elif self.get_target_state_message is None and self.target_state_type == "joint_state":
            target_message = userdata.selected_joint_state
        elif self.target_state_type == "joint_state":
            target_message = self.get_target_state_message(self.target_state_name).joint_state
        elif self.get_target_state_message is None and self.target_state_type == "pose":
            target_message = userdata.selected_pose
        elif self.target_state_type == "pose":
            target_message = self.get_target_state_message(self.target_state_name).pose_stamped
        else:
            rospy.logerr("Impossible to find the required message defining the target state!")
            return self.outcomes[-1]

        # Compute the plan
        if self.target_state_type == "pregrasp":
            plan = commander.plan_to_pose_target(target_message.pregrasp.pose, custom_start_state=starting_robot_state)
        elif self.target_state_type == "grasp":
            plan = commander.plan_to_pose_target(target_message.grasp.pose, custom_start_state=starting_robot_state)
        elif self.target_state_type == "postgrasp":
            plan = commander.plan_to_pose_target(target_message.postgrasp.pose, custom_start_state=starting_robot_state)
        elif self.target_state_type == "joint_state":
            plan = commander.plan_to_joint_value_target(target_message, custom_start_state=starting_robot_state)
        elif self.target_state_type == "pose":
            plan = commander.plan_to_joint_value_target(target_message, custom_start_state=starting_robot_state)

        # This condition checks whether a plan has been found
        if plan.joint_trajectory.joint_names == list():
            rospy.logerr("Fail to find a plan for {}".format(self.plan_name))
            return self.outcomes[-1]

        response = self.add_computed_plan(plan, self.plan_name)
        if not response.success:
            rospy.logerr("Cannot store the computed plan")
            return self.outcomes[-1]

        return self.outcomes[0]
