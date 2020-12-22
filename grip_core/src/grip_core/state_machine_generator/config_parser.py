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

# import ruamel.yaml as yaml
from state_machine_descriptor import StateMachineDescriptor


class StateMachineConfigParser(object):

    """
        Class gathering all required operations to parse a dictionary describing a state machine
    """

    def __init__(self, state_machine_dictionary, states_directory_path):
        """
            Initialise class' attributes and parses the dictionary

            @param state_machine_dictionary: Dictionary describing the state machine to be built
            @param states_directory_path: Path to the directory containing the states used to generate state machines
        """
        # Store the state machine dictionary
        self.state_machine_content = state_machine_dictionary
        # Path of the directory containing all states
        self.states_directory_path = states_directory_path
        # Will contain all (nested) state machines to generate
        self.state_machines = []
        # Dictionary allowing to map the recursion level to the description of the state machines to generate
        self.nested_state_machine = {0: []}
        # Recursion level (= how many times the _parse method has been called)
        self.recursion_level = 0
        # Parse the YAML file and fill the different attributes
        self._parse()
        self._simplify_state_machines()

    def _parse(self):
        """
            Fill the different attributes by iterating through all the state machines to generate
        """
        # Increment the recursion level
        self.recursion_level += 1
        # Initialise an empty list that contains the description of the potential nested state machine
        self.nested_state_machine[self.recursion_level] = []
        # Initialise a StateMachineDescriptor object from the content of the state_machine
        state_machine_descriptor = StateMachineDescriptor(self.state_machine_content, self.states_directory_path)
        # Iterate through the states and add them to the state machine descriptor
        # The "states" key is a list of dictionaries. So each state is a dictionary with an unique key
        for state in self.state_machine_content["states"]:
            # Get the name provided to the state
            state_name = state.keys()[0]
            # Get the associated description
            state_description = state[state_name]
            # Register a new state to the current state machine
            state_machine_descriptor.register_new_state(state_name, state_description)
            # Only state machines should have a field named "states"
            # So if such a field is found, we add the state description for this recursion level
            if "states" in state_description:
                self.nested_state_machine[self.recursion_level].append(state_description)
        # Once all the states of the current state machine have been visited, add the descriptor to the state_machines
        self.state_machines.append(state_machine_descriptor)

        # If any nested state machines have been detected at this level, call recursively this function on them
        for state_machine in self.nested_state_machine[self.recursion_level]:
            self.state_machine_content = state_machine.copy()
            self._parse()

    def _simplify_state_machines(self):
        """
            Go through all state machines and simplify the states corresponding to state machines
        """
        # List gathering the type of all the state machines
        state_machines_type = [state_machine.type for state_machine in self.state_machines]
        # Going through all state machines that have a nested state machine
        for state_machine_index in range(len(self.state_machines) - 1):
            # Extract the current state machine
            root_state_machine = self.state_machines[state_machine_index]
            # Iterate through all the states
            for state_name in root_state_machine.components.keys():
                # Get the StateDescriptor object
                state = root_state_machine.components[state_name]
                # If it's part of the following state machines type
                if state.type in state_machines_type[state_machine_index + 1:]:
                    # Simplify the parameters field (the state machine being already built with the proper parameters)
                    parameters = {"userdata": state.parameters["userdata"]} if "userdata" in state.parameters else {}
                    root_state_machine.components[state_name].parameters = parameters
