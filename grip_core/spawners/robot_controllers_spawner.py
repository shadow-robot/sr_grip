#!/usr/bin/env python3

# Copyright 2019, 2020, 2023 Shadow Robot Company Ltd.
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
import os
import yaml
import sys
import rosparam
from controller_manager_msgs.srv import SwitchController, LoadController, ListControllers
from std_msgs.msg import Bool


class ControllerSpawner(object):

    """
        Class gathering functions to spawn controllers without knowing their names
    """

    def __init__(self, controller_yaml_file_path):
        """
            Initialize the attribute of the class and upload the yaml file on the server

            @param controller_yaml_file_path: Path of the YAML file containing the controllers to spawn
        """
        # Make sure that the YAML file containing the controllers to load exists, otherwise exit the node
        if not os.path.exists(controller_yaml_file_path):
            rospy.logerr("The path provided does not seem to point to a valid file. "
                         "Provided path: {}".format(controller_yaml_file_path))
            return
        # Load the yaml's content file
        with open(controller_yaml_file_path, "r") as yaml_file:
            yaml_file_content = yaml.load(yaml_file)
        # Get the name of the controllers defined in the provided file
        self.controller_to_spawn_list = yaml_file_content.keys()
        # Upload controllers parameters of the param servers
        rosparam.upload_params('/', yaml_file_content)

    def spawn_controllers(self):
        """
            Set and spawn the controllers defined in the file given during the initialization
        """
        # Boolean to control the displayed log
        success = True
        # Gathers all controllers to load
        controllers_to_start = []
        # Get potential already running controllers
        try:
            rospy.wait_for_service('controller_manager/list_controllers', 5.0)
            list_controllers = rospy.ServiceProxy('controller_manager/list_controllers', ListControllers)
            running_controllers = list_controllers()
        except rospy.ServiceException:
            success = False

        already_running = False
        # Check whether controllers we want to spawn are already running
        for controller_state in running_controllers.controller:
            if controller_state.name in self.controller_to_spawn_list:
                already_running = True
        # If it's not the case then add controllers
        if not already_running:
            controllers_to_start += self.controller_to_spawn_list

        # Use the service to load the controllers that were filtered in the above steps
        for load_control in controllers_to_start:
            try:
                rospy.wait_for_service('controller_manager/load_controller', 5.0)
                load_controllers = rospy.ServiceProxy('controller_manager/load_controller', LoadController)
                loaded_controllers = load_controllers(load_control)
            except rospy.ServiceException:
                success = False
            if not loaded_controllers.ok:
                success = False
        # Try to switch controller
        try:
            rospy.wait_for_service('controller_manager/switch_controller', 5.0)
            switch_controllers = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)
            switched_controllers = switch_controllers(controllers_to_start, list(),
                                                      SwitchController._request_class.BEST_EFFORT, False, 0)
        except rospy.ServiceException:
            success = False
        if not switched_controllers.ok:
            success = False

        # If spawning failed during the process, output an error log, otherwise display info log
        if not success:
            rospy.logerr("Failed to launch controller(s) {}!".format(self.controller_to_spawn_list))
        else:
            rospy.loginfo("Controllers successfully spawned!")

    @staticmethod
    def wait_for_topic(topic_name, timeout):
        """
            Blocking function forcing the spawner to wait for a specific topic
            @param topic_name: Name of the topic the spawner has to wait for
            @param timeout: Time after which the blocking state will stop
        """
        if not topic_name:
            return True

        # This has to be a list since Python has a peculiar mechanism to determine
        # whether a variable is local to a function or not:
        # if the variable is assigned in the body of the function, then it is
        # assumed to be local. Modifying a mutable object (like a list)
        # works around this debatable "design choice".
        wait_for_topic_result = [None]

        def wait_for_topic_cb(msg):
            wait_for_topic_result[0] = msg
            rospy.logdebug("Heard from wait-for topic: %s" % str(msg.data))
        rospy.Subscriber(topic_name, Bool, wait_for_topic_cb)
        started_waiting = rospy.Time.now().to_sec()

        # We might not have received any time messages yet
        warned_about_not_hearing_anything = False
        while not wait_for_topic_result[0]:
            rospy.sleep(0.01)
            if rospy.is_shutdown():
                return False
            if not warned_about_not_hearing_anything:
                if rospy.Time.now().to_sec() - started_waiting > timeout:
                    warned_about_not_hearing_anything = True
                    rospy.logwarn("Controller Spawner hasn't heard anything from its \"wait for\" topic (%s)" %
                                  topic_name)
        while not wait_for_topic_result[0].data:
            rospy.sleep(0.01)
            if rospy.is_shutdown():
                return False
        return True


if __name__ == "__main__":
    rospy.init_node("custom_controller_spawner")
    topic_to_wait_for = ""
    robot_hardwares = rospy.get_param("robot_hardware", "")
    if robot_hardwares:
        for robot_hardware in robot_hardwares:
            hardware_options = rospy.get_param(robot_hardware)
            if "topic_to_wait_for" in hardware_options:
                topic_to_wait_for = hardware_options["topic_to_wait_for"]
                break
    timeout = rospy.get_param("~timeout", 10.0)

    controller_file_path = sys.argv[1]

    custom_controller_spawner = ControllerSpawner(controller_file_path)

    if custom_controller_spawner.wait_for_topic(topic_to_wait_for, timeout):
        custom_controller_spawner.spawn_controllers()
