#!/usr/bin/env python3

# Copyright 2020 Shadow Robot Company Ltd.
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
from grip_core.utils.common_paths import FOLDER_TEMPLATE_LAUNCH_FILE, LAUNCH_CORE_FOLDER


class GenericTemplater(object):

    """
        Generic class gathering all required steps to generate files from templates
    """

    def __init__(self, templates_directory_path, target_directory_path):
        """
            Initialise the attributes of the class

            @param templates_directory_path: Path to the templates directory that can be used to generate a file
            @param target_directory_path: Path to the directory in which generated files should be stored
        """
        # Create a Jinja2 environment from the templates_directory_path with the proper rendering options
        self.environment = Environment(loader=FileSystemLoader(os.path.abspath(templates_directory_path)),
                                       trim_blocks=True, lstrip_blocks=True)
        # Add the python add and any filters
        self.environment.filters['any'] = any
        self.environment.filters['all'] = all
        # Path to the directory in which the generated files will be saved
        self.target_directory_path = os.path.abspath(target_directory_path)


class LaunchFileTemplater(GenericTemplater):

    """
        Class allowing to create a launch file corresponding to a set of configurations
    """

    def __init__(self, templates_directory=FOLDER_TEMPLATE_LAUNCH_FILE, target_directory_path=LAUNCH_CORE_FOLDER):
        """
            Initialize the class with the appropriate arguments to create a launch file for the framework

            @param templates_directory_path: Path to the templates directory that can be used to generate a file
            @param target_directory_path: Path to the directory in which generated files should be stored
        """
        super(LaunchFileTemplater, self).__init__(templates_directory, target_directory_path)

    def generate_launch_file(self, launch_parameters, template_name="framework_launch.template",
                             filename="generated_framework_launch"):
        """
            Generate a launch file from a dictionary containing parameters that will be used to fill the
            provided template. The file is then saved where specified.

            @param launch_parameters: Dictionary containing all the information required to generate the launch file
            @param template_name: Name of the template to use
            @param filename: Name given to the generated launch file
        """
        # Get the template from the environment
        template = self.environment.get_template(template_name)
        # Render it
        rendered_template = template.render(launch_parameters=launch_parameters)
        # Makes sure the file has an appropriate extension
        if not filename.endswith(".launch"):
            filename += ".launch"
        generated_file_path = os.path.join(self.target_directory_path, filename)
        # Save it
        with open(generated_file_path, "w") as generated_file:
            generated_file.write(rendered_template)
