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

from config_parser import StateMachineConfigParser
from templater import StateMachineTemplater


def generate_state_machines(state_machine_dictionary, state_source, state_machine_sources, target_folder_path):
    """
        Generate all the python files corresponding to the state machines required to run a described state machine

        @param state_machine_dictionary: Dictionary containing all the parameters required to describe a state machine
        @param state_source: Path to the folder containing the states
        @param state_machine_sources: List of pathes poitning to folders containing the state machine templates
        @param target_folder_path: Path to the folder the generated state machines should be saved to
    """
    # Initialise the parser and templater
    parser = StateMachineConfigParser(state_machine_dictionary, state_source)
    templater = StateMachineTemplater(state_machine_sources, target_folder_path)

    # For each state machine (starting with nested one) generate the corresponding file
    for state_machine in reversed(parser.state_machines):
        templater.generate_state_machine(state_machine)
