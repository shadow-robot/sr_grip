#!/usr/bin/env python3

# Copyright 2020, 2023 Shadow Robot Company Ltd.
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

import rospkg
import rostopic
import os
from grip_core.utils.common_paths import GENERATED_STATES_FOLDER, STATE_TEMPLATES_FOLDER
from .templater import StateTemplater


def generate_state(state_info, target_folder_path=GENERATED_STATES_FOLDER):
    """
        Generate a python file corresponding to a state compatible with GRIPS's state machines

        @param state_info: Dictionary containing some of the parameters required to describe the state to generate
        @param target_folder_path: Path to the folder the generated state should be saved to
    """
    # Will contain all the required information
    state_descriptor = dict()
    # If the state to generate corresponds to a sensor
    if "data_topics" in state_info:
        # Get the type of message
        message_type = rostopic.get_topic_type(state_info["parameters"]["sensor_topic"])[0]
        # The above function returns package_name/msg_name, so split by / and get the import statement and message name
        split_msg_type = message_type.split("/")
        state_descriptor["msg_import_statement"] = split_msg_type[0] + ".msg"
        state_descriptor["msg_name"] = split_msg_type[1]
    # Otherwise it must be a state to run external components
    else:
        # Get the path pointing to the .srv or .action file
        server_file = state_info["action/service"]
        # Get the name of the package containing the .action or .srv file
        pkg_name = rospkg.get_package_name(server_file)
        # Get the corresponding import statement depending on whether it's an action or service
        state_descriptor["server_statement"] = pkg_name + ".srv" if server_file.endswith(".srv") else pkg_name + ".msg"

        # Get the name of the file to import
        if server_file.endswith(".srv"):
            def_file = os.path.basename(server_file).replace(".srv", "")
        else:
            def_file = os.path.basename(server_file).replace(".action", "Action")
        state_descriptor["def_file"] = def_file
        # Get the name of the server from state_info
        state_descriptor["server_name"] = state_info["server_name"]

    # Fill in the remaining parameters already part of state_info
    state_descriptor["name"] = state_info["name"]
    state_descriptor["template"] = state_info["template"]
    state_descriptor["filename"] = state_info["filename"]

    # Initialize the state templater and generate the state
    state_templater = StateTemplater(STATE_TEMPLATES_FOLDER, target_folder_path)
    state_templater.generate_state(state_descriptor)
