#!/usr/bin/env python3

# Copyright 2019, 2020 Shadow Robot Company Ltd.
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

from jinja2 import Environment, FileSystemLoader
import os
import stat


class StateMachineTemplater(object):

    """
        Class gathering all required steps to generate state machines from templates
    """

    def __init__(self, templates_directories, target_directory_path, commanders_descriptor):
        """
            Initialise the attributes of the class

            @param templates_directories: List of path pointing to folders containing state machine templates
            @param target_directory_path: Path to the directory in which generated state machines should be stored
            @param commanders_descriptor: Dictionary containing the parameters of the configured commanders
        """
        # Create a Jinja2 environment from the templates_directory_path with the proper rendering options
        self.environment = Environment(loader=FileSystemLoader(templates_directories),
                                       trim_blocks=True, lstrip_blocks=True)
        # Path to the directory in which the generated state machines will be saved
        self.target_directory_path = os.path.abspath(target_directory_path)
        # Commanders configuration
        self.commanders = commanders_descriptor

    def generate_state_machine(self, state_machine_descriptor):
        """
            Generate the state machine corresponding to the input descriptor, save it into a file and make it executable

            @param state_machine_descriptor: Object instanciated from the class StateMachineDescriptor containing
                                             the required information to generate the state machine
        """
        # Store the name of the template file
        template_file = state_machine_descriptor.template_file
        # Get the base name of the file
        file_base_name = os.path.basename(template_file)
        # Get the template
        template = self.environment.get_template("{}".format(file_base_name))
        # Render it
        if file_base_name == "base_state_machine.template":
            rendered_template = template.render(state_machine=state_machine_descriptor, commanders=self.commanders)
        else:
            rendered_template = template.render(state_machine=state_machine_descriptor)
        # Build the path of the created python file
        state_machine_filename = self.target_directory_path + "/{}.py".format(state_machine_descriptor.source_name)
        # Save it
        with open(state_machine_filename, "w") as python_file:
            python_file.write(rendered_template)
        # Make it executable to use it
        file_status = os.stat(state_machine_filename)
        os.chmod(state_machine_filename, file_status.st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)


class StateTemplater(object):

    """
        Class gathering all required steps to generate a state from a template
    """

    def __init__(self, template_directory, target_directory_path):
        """
            Initialise the attributes of the class

            @param template_directory: Path pointing to the folder containing the state templates
            @param target_directory_path: Path to the directory in which generated states should be stored
        """
        # Create a Jinja2 environment from the templates_directory_path with the proper rendering options
        self.environment = Environment(loader=FileSystemLoader(template_directory), trim_blocks=True,
                                       lstrip_blocks=True)
        # Path to the directory in which the generated states will be saved
        self.target_directory_path = os.path.abspath(target_directory_path)

    def generate_state(self, state_info):
        """
            Generate the state corresponding to the input dictionary and save it into a file

            @param state_machine_descriptor: Dictionary containing the required information to generate the state
        """
        # Store the name of the template file
        template_file = state_info["template"]
        # Get the base name of the file
        file_base_name = os.path.basename(template_file)
        # Get the template
        template = self.environment.get_template("{}".format(file_base_name))
        # Render it
        if file_base_name == "external_component.template":
            rendered_template = template.render(component=state_info)
        elif file_base_name == "sensor.template":
            rendered_template = template.render(sensor=state_info)
        # Build the path of the created python file
        state_filename = self.target_directory_path + "/{}.py".format(state_info["filename"])
        # Save it
        with open(state_filename, "w") as python_file:
            python_file.write(rendered_template)
