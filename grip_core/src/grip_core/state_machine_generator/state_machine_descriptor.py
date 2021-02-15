#!/usr/bin/env python

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

from collections import OrderedDict
from state_descriptor import StateDescriptor


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
        # List that will gather the source of the states used to create the state machine
        self.states_source = list()
        # Ordered dictionary containing all the states composing the state machine
        self.components = OrderedDict()
        # Load the parameters of the state machine
        self._load_params(state_machine_description)

    def _load_params(self, state_machine_description):
        """
            Load the parameters of the state machine

            @param state_machine_description: Dictionary containing parameters name and values of the state machine
        """
        # We iterate through all the parameters since they are state-dependant
        for parameter_name, parameter in state_machine_description.items():
            # The field "source" provides the name of the template file without the .template
            if parameter_name == "source":
                self.template_file = parameter
            # Filtering the parameters that we store in the state machine
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
            @param state_description: Dictionary containing all the parameters of a state
        """
        # If the element has a key named import_statement, then extract it and remove it from the dictionary
        if "import_statement" in state_description:
            import_statement = state_description["import_statement"]
            del state_description["import_statement"]
        # Otherwise we assume it comes from the generated state machines
        else:
            import_statement = "grip_core.generated_state_machines"

        # Add a state to the dictionary with its name as key and a StateDescriptor object as value
        self.components[state_name] = StateDescriptor(state_description)
        # Extract the source of the state
        state_source_name = self.components[state_name].source_name
        # From the filename infer the class name of the state to import, except for classes which are supposed to have
        # a "name" field
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
            Return the string representation of the class' content

            @return: String containing some of the information stored in the class
        """
        first_part = "params: {}\ntype: {}\ncomponents:\n".format(self.parameters, self.type)
        second_part = ""
        for component_name, component in self.components.items():
            second_part += "\t* {}: {}\n".format(component_name, component)
        return first_part + second_part
