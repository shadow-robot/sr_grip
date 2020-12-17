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

import roslaunch
import rospy

# When this node is called, use the roslaunch API to run a node given some parameters saved on the rosparam server
if __name__ == '__main__':
    # Initialise node
    rospy.init_node('external_controllers_spawner', anonymous=True)
    # Get information to initialise the node (from rosparam server)
    package_name = rospy.get_param("package_manipulator_action_server")
    node_name = rospy.get_param("manipulator_controller_node_name")
    node_type = rospy.get_param("manipulator_controller_action_server_type")
    # If package name is empty it means the server is loaded from a plugin so this node can be shutdown
    if not package_name:
        rospy.signal_shutdown("Option package_manipulator_action_server not provided, this node will therefore assume "
                              "that controller has been provided by a plugin. Terminating.")
    # Create the node
    node = roslaunch.core.Node(package_name, node_type, name=node_name, output="screen")
    # Initalise and start the Launch API
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    # Start the process
    process = launch.launch(node)
    # Spin
    rospy.spin()
