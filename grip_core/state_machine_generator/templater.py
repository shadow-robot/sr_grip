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

from jinja2 import Environment, FileSystemLoader
import os
import stat


class StateMachineTemplater(object):

    """
        Class gathering all required steps to generate the state machines from templates
    """

    def __init__(self, templates_directory_path, target_directory_path):
        """
            Initialise the attributes of the class

            @param templates_directory_path: Path to the directory containing the state machine templates
            @param target_directory_path: Path to the directory in which generated state machines should be stored
        """
        # Create a Jinja2 environment from the templates_directory_path with the proper rendering options
        self.environment = Environment(loader=FileSystemLoader(os.path.abspath(templates_directory_path)),
                                       trim_blocks=True, lstrip_blocks=True)
        # Path to the directory in which the generted state machiens will be saved
        self.target_directory_path = os.path.abspath(target_directory_path)

    def generate_state_machine(self, state_machine_descriptor):
        """
            Generate the state machine corresponding to the input descriptor, save it into a file and make it executable

            @param state_machine_descriptor: Object instanciated from the class StateMachineDescriptor containing
                                             the required information to generate the state machine
        """
        # Store the name of the template file
        template_file = state_machine_descriptor.template_file
        # Get the template
        template = self.environment.get_template("{}.template".format(template_file))
        # Render it
        rendered_template = template.render(state_machine=state_machine_descriptor)
        # Build the path of the created python file
        state_machine_filename = self.target_directory_path + "/{}.py".format(state_machine_descriptor.source_name)
        # Save it
        with open(state_machine_filename, "w") as python_file:
            python_file.write(rendered_template)
        # Make it executable to use it
        file_status = os.stat(state_machine_filename)
        os.chmod(state_machine_filename, file_status.st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
