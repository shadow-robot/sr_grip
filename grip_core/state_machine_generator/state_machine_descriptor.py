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

from collections import OrderedDict
from state_descriptor import StateDescriptor
import rospy
import os


class StateMachineDescriptor(object):

    """
        Class used to store all required information to describe a state machine
    """

    def __init__(self, state_machine_description, states_directory_path):
        """
            Initialise the class' attributes, get the list of states and loads the state machine parameters

            @param state_machine_description: Dictionary containing parameters name and values of the state machine
            @param states_directory_path: Path to the directory containing the states used to generate state machines
        """
        # Contain the parameters of the state machine
        self.parameters = {}
        # Name of the file in which the state machine will be stored
        self.source_name = None
        # Type of the state machine (StateMachine, ConcurrentStateMachine, custom-made...)
        self.type = None
        # Name of the file in which the template is stored
        self.template_file = None
        # Dictionary that will contain all the parameters that can be propagated to children states
        self.params_to_propagate = {}
        # List that will gather the source of the states used to create the state machine
        self.states_source = []
        # Ordered Dictionary containing all the states composing the state machine
        self.components = OrderedDict()
        # Will contain the list of available states
        self.list_states = []
        # Will contain the python statement to import the states (e.g from "my_package.something.foo")
        self.state_import_statement = ""
        # Fill the list of states and get the corresponding import statement
        self._get_states_information(states_directory_path)
        # Load the parameters of the state machine
        self._load_params(state_machine_description)

    def _get_states_information(self, state_directory_path):
        """
            Extract from the path containing the states the python import statement and the lsit of all files that
            can containing states

            @param state_directory_path: Path to the directory containing the states used to generate state machines
        """
        # Get the absolute path to avoid any possible mistake
        absolute_path = os.path.abspath(state_directory_path)
        # Retrieve the correct python import statement by finding the correct package in which the states are stored
        # List gathering all the folders. Discarding the first element because it is empty (absolute path starts with /)
        split_path = absolute_path.split("/")[1:]
        # Variable that will contain the progressive paths
        tested_path = ""
        # For each root of the path tree, try to find if it contains an __init__.py (i.e the diretory is a package)
        for root_level, directory_name in enumerate(split_path):
            tested_path += "/{}".format(directory_name)
            # If we find the package name, we take everything after this to create the proper import path
            if os.path.exists(tested_path + "/__init__.py"):
                self.state_import_statement = ".".join(split_path[root_level:])
                break
        # If no python package as been found, then output a warning message will be displayed
        if not self.state_import_statement:
            rospy.loginfo("WARNING: The path directory containing the states does not seem to be a python package! "
                          "(python 2 only)")

        # Get a list of all the states
        self.list_states = [filename.replace(".py", "")
                            for filename in os.listdir(absolute_path) if filename.endswith(".py")]

    def _load_params(self, state_machine_description):
        """
            Load the parameters of the state machine

            @param state_machine_description: Dictionary containing parameters name and values of the state machine
        """
        # The field "params" can contain all parameters required for the state machine
        # and are accessible to children states
        if "params" in state_machine_description:
            self.parameters = state_machine_description["params"]
            self.params_to_propagate = self.parameters.copy()
        # We still iterate through all the parameters since they are state-dependant
        for parameter_name, parameter in state_machine_description.items():
            # The field "source" provides the name of the template file without the .template
            if parameter_name == "source":
                self.template_file = parameter
            # Fitlering the parameters that we store in the s
            elif parameter_name not in ["params", "transitions", "states"]:
                self.parameters[parameter_name] = parameter
        # If a name is specified, change the source and type accordingly
        if "name" in self.parameters:
            self.set_source(self.parameters["name"])
        # Otherwise, set these inforamtion according to the template
        else:
            self.set_source(self.template_file)

    def register_new_state(self, state_name, state_description):
        """
            Add a new state to the components of the state machine

            @param state_name: Name of the state to add
            @param state_description: Dictionary containing all the parameters fo the state
        """
        # Add a state to the dictionary with its name as key and a StateDescriptor object as value
        self.components[state_name] = StateDescriptor(state_description, self.params_to_propagate)
        # Extract the source of the state
        state_source_name = self.components[state_name].source_name
        # If it is in the list of states that can be imported the import statement is the same as the computed one
        # Otherwise we assume that it is an already generated state machine
        if state_source_name in self.list_states:
            import_statement = self.state_import_statement
        else:
            import_statement = "grip_core.state_machines"
        # From the filename infer the class name of the state to import
        # (except for classes which are supposed to have a "name" field)
        if "name" in self.components[state_name].parameters:
            # Trick to get the proper class name to import another state machine
            self.components[state_name].set_source(self.components[state_name].parameters["name"])
        state_type = self.components[state_name].type
        state_source_name = self.components[state_name].source_name
        # If it is the first occurence of the state type in the state machine we record the tuple
        if (import_statement, state_source_name, state_type) not in self.states_source:
            self.states_source.append((import_statement, state_source_name, state_type))

    def set_source(self, source):
        """
            Set the source name and the type of the state machine

            @param source: String containing the source of the state_machine (name of the python file without .py)
        """
        self.source_name = source
        # Get the type from the source following PEP-8 naming rules
        self.type = "".join([word.capitalize() for word in source.split("_")])

    def __str__(self):
        """
            Return the string representation of class' content

            @return: String containing some of the information stored in the class
        """
        first_part = "params: {}\ntype: {}\ncomponents:\n".format(self.parameters, self.type)
        second_part = ""
        for component_name, component in self.components.items():
            second_part += "\t* {}: {}\n".format(component_name, component)
        return first_part + second_part
