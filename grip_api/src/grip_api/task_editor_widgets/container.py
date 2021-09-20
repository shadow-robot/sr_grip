#!/usr/bin/env python3

# Copyright 2020, 2021 Shadow Robot Company Ltd.
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

import os
from collections import OrderedDict
from grip_core.utils.file_parsers import (extract_state_machine_parameters_from_file, AVAILABLE_STATEMACHINES)
from grip_core.utils.common_paths import TASK_EDITOR_ROOT_TEMPLATE
from grip_api.task_editor_graphics.container import GraphicsContainer
from grip_api.utils.common_dialog_boxes import error_message, warning_message
from .terminal_socket import TerminalSocket
from .socket import Socket
from .connector import Connector
from .state import State
from .state_machine import StateMachine
from .container_history import ContainerHistory


class Container(object):

    """
        Object keeping record of which widgets are currently composing a state machine.
    """

    def __init__(self, editor_widget, container_type):
        """
            Initialize the object

            @param editor_widget: GraphicalEditorWidget for which the state machine is the container
            @param container_type: Type of the container (string)
        """
        # Store the graphical editor widget
        self.editor_widget = editor_widget
        # Set the type of the container
        self.type = container_type
        # Attribute recording whether the container is valid (i.e. can be executed)
        self.is_valid = False
        # Create the graphics representation of this widget
        self.graphics_container = GraphicsContainer(self)
        self.graphics_container.set_graphics_scene(64000, 64000)
        # Store the different states present in the container
        self.states = list()
        # Get specific list of states that need to be generated
        self.states_to_generate = list()
        # Store the different connectors present in the container
        self.connectors = list()
        # Store the different state machines added to this container
        self.state_machines = list()
        # Extract the default outcomes of the state machine container
        self.outcomes = self.get_outcomes()
        # If applicable, will contain the state-like representation of this container in another GraphicalEditorWidget
        self.state_machine = None
        # Attribute tracking the current "layer" height required to be sure to have a widget overposing all the others
        self.z_tracker = 0
        self.output_userdata = list()
        # History of the actions performed inside the container
        self.history = ContainerHistory(self)

    def get_outcomes(self):
        """
            Extract the outcomes from the current parameters of the state machine container

            @return: A list of strings, each element being an outcome
        """
        # Get the outcomes from the parameters
        parsed_outcomes = self.parameters["outcomes"]
        # If nothing is provided, then set "success" and "failure" as default
        outcomes = ["success", "failure"] if not parsed_outcomes else parsed_outcomes
        return outcomes

    def create_initial_terminal_sockets(self):
        """
            Create and add the terminal sockets of this state machine container
        """
        # Will contain the terminal sockets
        self.terminal_sockets = list()

        # Create a terminal socket for the beginning of the container. If the container is a concurrent one then the
        # starting socket support multi connections otherwise it does not.
        is_multi = self.type == "ConcurrentStateMachine"
        start_socket = TerminalSocket(container=self, socket_name="Start", index=0, multi_connections=is_multi)
        # Add it to the terminal sockets and add it to the graphical representation
        self.terminal_sockets.append(start_socket)
        self.graphics_container.addItem(start_socket.graphics_socket)

        # Create a terminal socket for each outcome
        for index, outcome in enumerate(self.outcomes):
            terminal_socket = TerminalSocket(container=self, socket_name=outcome, index=index)
            self.terminal_sockets.append(terminal_socket)
            # Add the graphical socket to the graphical representation so it can be rendered
            self.graphics_container.addItem(terminal_socket.graphics_socket)
        # By default, the default outcome is the last one
        self.default_socket = self.terminal_sockets[-1]
        # Since the terminal sockets have not been properly located yet, the initialisation of the container is not done
        self.is_complete = False

    def add_terminal_socket(self, outcome_name, position):
        """
            Add a terminal socket to the container, which represents a new outcome of the task

            @param outcome_name: Name given to the new terminal socket
            @param position: List containing the scene coordinates (x, y) of where the socket should be located
        """
        # Add the new name ot the outcome attribute
        self.outcomes.append(outcome_name)
        # Create the terminal socket
        new_terminal_socket = TerminalSocket(container=self, socket_name=outcome_name, index=len(self.outcomes) - 1)
        # Set its position from the the input argument
        new_terminal_socket.set_position(*position)
        # All terminal sockets added by this function are deletable
        new_terminal_socket.is_deletable = True
        # Add socket to terminal sockets
        self.terminal_sockets.append(new_terminal_socket)
        # Add the graphical socket to the graphical representation so it can be rendered
        self.graphics_container.addItem(new_terminal_socket.graphics_socket)
        # Update the positions of the sockets of the state-like representation of the container if possible
        if self.state_machine is not None:
            self.state_machine.update_sockets_position()

    def set_name(self, name):
        """
            Set the name of the container and if applicable corresponding state-like representation

            @param name: Name of the state machine container
        """
        self.name = name
        if self.state_machine is not None:
            self.state_machine.name = name

    def set_state_like(self, state_machine):
        """
            Set a state-like representation of the state machine, added into another editor widget

            @param state_machine: StateMachine object corresponding to this container in another editor widget
        """
        self.state_machine = state_machine

    def connect_to_default_socket(self):
        """
            Automatically connect all free sockets to the default terminal socket
        """
        # For each state-like object in the container
        for box in (self.states + self.state_machines):
            # If a socket is not connected, create a new connector between it and the default terminal socket
            for socket in box.output_sockets:
                if not socket.is_connected():
                    Connector(self, socket, self.default_socket)

    def get_view(self):
        """
            Return the view corresponding to the associated GraphicsScene

            @return: TaskEditorView linked to the container
        """
        return self.graphics_container.views()[0]

    def add_state(self, state):
        """
            Store a new state added to the container and potentially rename it if another one with the same name exists

            @param state: State object to be added to the container
        """
        # Get the unique name with which the state will be added
        updated_name = self.get_unique_name(state.name)
        # Update the name of the state
        state.name = updated_name
        # Register the state
        self.states.append(state)
        if state.to_generate:
            self.states_to_generate.append(state)
        # Adding a state will make the container not valid, so update it
        self.update_validity()
        self.z_tracker += 1

    def add_connector(self, connector):
        """
            Store a new connector between two states, added in the view

            @param connector: Object (Connector) to be added in the container
        """
        self.connectors.append(connector)
        # When we add a new connector, it can make the container valid, so call the function to update this attribute
        self.update_validity()

    def add_state_machine(self, state_machine):
        """
            Store a new state machine added to the container

            @param state_machine: StateMachine object added to the container
        """
        self.state_machines.append(state_machine)
        # Adding a state machine will make the container not valid, so update it
        self.update_validity()

    def remove_state(self, state):
        """
            Remove the input state from the container

            @param state: State object to be removed from the container
        """
        # Make sure the state is still part of the container
        if state in self.states:
            self.states.remove(state)
            if state in self.states_to_generate:
                self.states_to_generate.remove(state)
            # Remove the state from the graphical view as well
            self.graphics_container.removeItem(state.graphics_state)
            # Removing a state can make the container valid, so update it
            self.update_validity()
            # Update the depth tracker
            self.z_tracker -= 1

    def remove_connector(self, connector):
        """
            Remove a given connector for the connectors attribute

            @param connector: Object (Connector) to be removed from the container
        """
        if connector in self.connectors:
            self.connectors.remove(connector)
            # Remove the connector from the graphical view as well (if not already removed by a socket)
            if connector.graphics_connector is not None:
                self.graphics_container.removeItem(connector.graphics_connector)
            # Removing a connector can make the container not valid, so update it
            self.update_validity()

    def remove_terminal_socket(self, socket):
        """
            Remove a given terminal socket from the container

            @param socket: TerminalSocket to be removed from the container
        """
        # If the socket is properly registered and can be removed
        if socket in self.terminal_sockets and socket.is_deletable:
            # Remove from the list of outcomes and terminal sockets
            self.outcomes.remove(socket.name)
            self.terminal_sockets.remove(socket)
            # Remove the graphical representation
            self.graphics_container.removeItem(socket.graphics_socket)
            # Update the positions of the sockets of the state-like representation of the container if possible
            if self.state_machine is not None:
                self.state_machine.update_sockets_position()
            # Make sure the container is still valid
            self.update_validity()

    def remove_state_machine(self, state_machine):
        """
            Remove the provided state machine from the container

            @param state_machine: StateMachine object to be removed from the container
        """
        # Make sure the state machine is still part of the container
        if state_machine in self.state_machines:
            self.state_machines.remove(state_machine)
            # Remove the state from the graphical view as well
            self.graphics_container.removeItem(state_machine.graphics_state)
            # Removing a state machine can make the container valid, so update it
            self.update_validity()
            # Update the depth tracker
            self.z_tracker -= 1

    def clear(self, include_state_machines=True):
        """
            Remove all the states and connectors of the scene. If include_state_machines is True, also remove all the
            state machines.

            @param include_state_machines: Boolean that specifies whether the state machines should be deleted or not
        """
        # Remove all the states
        while self.states:
            self.states[0].remove()
        # If instructed, remove all the state machines
        if include_state_machines:
            while self.state_machines:
                self.state_machines[0].remove()
        # Remove residual connectors
        while self.connectors:
            self.connectors[0].remove()

    def get_unique_name(self, name):
        """
            Modify the input name by appending a digit at the end if needed to make sure two elements part of the
            container don't have the same name

            @param name: Candidate name (string) of the element to be add to the container
            @return: Unchanged name if it is unique, otherwise name+index, for instance name0 or name1
        """
        final_name = name
        counter = 0
        # As long as we find the given name in states or state machines, generate another one
        while any(final_name == x.name for x in (self.states + self.state_machines)):
            final_name = name + "{}".format(counter)
            counter += 1
        return final_name

    def update_validity(self):
        """
            Update the is_valid attribute according to the current container's content
        """
        # All the states should be fully connected
        are_states_valid = all(state.is_valid() for state in self.states)
        # Check that all connectors are linked to two sockets
        are_connectors_valid = all(connector.is_valid() for connector in self.connectors)
        # Run a check on the state machines
        are_state_machines_valid = all(state_machine.is_valid() for state_machine in self.state_machines)
        # Check whether the container is empty
        is_empty = (not self.states and not self.state_machines)
        # For the container to be valid we need all the above conditions to be True and empty to be False
        is_valid = are_connectors_valid and are_states_valid and are_state_machines_valid and not is_empty
        # If the validity of the container has changed, update the corresponding attribute and the validity icon
        if is_valid != self.is_valid:
            self.is_valid = is_valid
            self.editor_widget.update_validity_icon()
            # Update the related containers
            if self.state_machine is not None:
                self.state_machine.container.update_validity()

    def get_parsed_container(self):
        """
            Return the information of the container as a dictionary

            @return: Dictionary with all the information from the container
        """
        # Extract container specific information
        container_information = self.get_container_specific_information()
        # Extract the information related to the container's content
        self.parse_container_content(container_information)
        return container_information

    def get_container_specific_information(self):
        """
            Create, fill and return a dictionary with the information related to the container's type

            @return: Dictionary with all the information related to the container's type
        """
        info_dictionary = dict()
        # If it's the root container, add an extra argument to identify it to ROS when run
        if self.type == "base":
            source = os.path.basename(TASK_EDITOR_ROOT_TEMPLATE)
            info_dictionary["node_name"] = self.name + "_state_machine_node"
        else:
            source = AVAILABLE_STATEMACHINES[self.type]["source"]
        # Get the source of the container to generate the state machine
        info_dictionary["source"] = source
        # Extract the container's outcomes
        info_dictionary["outcomes"] = self.outcomes
        # Get the name of the container
        info_dictionary["name"] = self.name
        # ConcurrentStateMachine containers have extra arguments required to configure them
        if self.type == "ConcurrentStateMachine":
            info_dictionary["default_outcome"] = self.default_socket.name
            info_dictionary["outcome_map"] = self.get_mapping_from_terminal()

        if self.state_machine is not None:
            info_dictionary["userdata"] = self.state_machine.get_userdata()

        return info_dictionary

    def get_mapping_from_terminal(self):
        """
            Return a dictionary that contains the mapping from states to terminal states except for the starting and
            default one.

            @return: Dictionary with following format: {terminal1: {state1: state1_outcome, ...}, terminal2: ....}
        """
        outcome_map = dict()
        # For all terminal nodes that are neither the starting or default one, extract which transitions leads to this
        # terminal outcome
        for terminal_socket in self.terminal_sockets:
            if not terminal_socket.is_starting and terminal_socket.name != self.default_socket.name:
                socket_dict = dict()
                # For each connector set to this outcome, get the state and the outcome of the state it is linked to
                for connector in terminal_socket.connectors:
                    socket_dict[str(connector.start_socket.state.name)] = str(connector.start_socket.name)
                outcome_map[str(terminal_socket.name)] = socket_dict
        return outcome_map

    def parse_container_content(self, information_dict):
        """
            Fill a provided dictionary with the information related to the states and state machines of the containers

            @param information_dict: Dictionary to fill
        """
        # Get the states and state machines in "flowing" order (from start to terminal)
        ordered_components = self.get_ordered_components()
        container_components = list()
        # Get the information of each component that belongs to this container
        for component in ordered_components:
            state_dictionary = dict()
            state_dictionary[component.name] = component.get_config()
            container_components.append(state_dictionary)
        information_dict["states"] = container_components

    def get_ordered_components(self):
        """
            Return a list containing state and state machines objects ordered following the flow of the state machine

            @return: List of State or StateMachine objects
        """
        ordered_components = list()
        # Should only have one component except if it's a concurrent container
        starting_components = list()
        for component in (self.states + self.state_machines):
            # If one of the connector of the input socket of the component is linked to the Starting terminal socket
            # then signal this component as "initial"
            if any(x.start_socket.name == "Start" for x in component.input_socket[0].connectors):
                starting_components.append(component)
        # While we have something as a starting component
        while not not starting_components:
            # Take the oldest element registered
            oldest_component = starting_components.pop(0)
            ordered_components.append(oldest_component)
            connected_states = []
            for socket in oldest_component.output_sockets:
                if isinstance(socket.connectors[0].end_socket, Socket):
                    state = socket.connectors[0].end_socket.state
                    # Check that this state is not already registered
                    if state not in ordered_components:
                        connected_states.append(state)
            starting_components = starting_components + connected_states
        return ordered_components

    def update_current_items_availability(self):
        """
            Make sure all the states and state machines currently used in the task editor are compatible with the
            current robot configuration. If not, make them semi-transparent and displays a message
        """
        # Get all the states that can currently be used
        list_available_states = self.editor_widget.parent().parent().parent().parent().state_displayer.list_widget
        # Boolean stating whether some generated or commander states are available
        has_some_generated = "Generated" in list_available_states.states_to_display
        has_some_commanders = "Commander" in list_available_states.states_to_display
        # Get the list of the generated, constant and commander states
        generated_states = list() if not has_some_generated else list_available_states.states_to_display["Generated"]
        constant_states = list_available_states.states_to_display["Constant"]
        commander_states = list() if not has_some_commanders else list_available_states.states_to_display["Commander"]

        # Will contain all the states that can't be found
        cannot_be_found_states = list()
        # For all the states that are currently being used
        for state in self.states:
            is_available = state.type in constant_states or state.type in commander_states
            is_generated = state.type in generated_states
            # If the state is not available and not to be generated, then store it in a specific list. Otherwise, make
            # sure that the graphical representation shows a valid state (fully opaque)
            if not (is_available or is_generated):
                cannot_be_found_states.append(state)
            # If the state is now available but wasn't before, reset the opacity to 1
            elif state.get_opacity() < 1:
                state.set_opacity(1.)

        # Make all the states not found semi-transparent as a cue and display a message to inform the user about the
        # current issue
        if cannot_be_found_states:
            states_name = ", ".join(map(lambda x: x.name, cannot_be_found_states))
            warning_message(
                "Error parsing the task", "Some states aren't available anymore due to missing configurations",
                additional_text="Please add the missing configuration for the states {}".format(states_name))

            for state in cannot_be_found_states:
                state.set_opacity(0.5)
        # If there's at least of state for which there is an issue, update the opacity of the box-like representation
        # to 0.5 and 1 if everything is alright
        if self.state_machine is not None:
            self.state_machine.set_opacity(0.5 if cannot_be_found_states else 1.)

        # Update the validity of the container
        self.update_validity()

    def save(self):
        """
            Save the current properties of the object so it can be restored later on

            @return: Dictionary containing the configuration of the terminal sockets, states and connectors
        """
        # Lists containing required information to restore the current configuration of the terminal sockets, states,
        # state machines and connectors.
        terminal_sockets, states, state_machines, connectors = list(), list(), list(), list()
        # Save their configuration
        for socket in self.terminal_sockets:
            terminal_sockets.append(socket.save())

        for state in self.states:
            states.append(state.save())

        for state_machine in self.state_machines:
            state_machines.append(state_machine.save())

        for connector in self.connectors:
            connectors.append(connector.save())

        return OrderedDict([
            ('terminal_sockets', terminal_sockets),
            ('states', states),
            ('state_machines', state_machines),
            ('connectors', connectors)
        ])

    def restore_states(self, list_of_states, socket_mapping, offset=list(), new_sockets=None):
        """
            Restore the states stored in a list of saved states

            @param list_of_states: List of dictionaries obtained by calling the save() function for each state
            @param socket_mapping: Dictionary mapping the id of the sockets to the pointer of the actual object
            @param offset: List or tuple containing the translational offset to be applied to the items restored
            @param new_sockets: Dictionary that will contain the mapping between the old and new socket IDs. If set to
                                None, new socket IDs won't be generated.
        """
        # Access the states that can currently be used
        list_available_states = self.editor_widget.parent().parent().parent().parent().state_displayer.list_widget
        has_some_generated = "Generated" in list_available_states.states_to_display
        has_some_commanders = "Commander" in list_available_states.states_to_display
        # Get the list of the generated, contant and commander states
        generated_states = list() if not has_some_generated else list_available_states.states_to_display["Generated"]
        constant_states = list_available_states.states_to_display["Constant"]
        commander_states = list() if not has_some_commanders else list_available_states.states_to_display["Commander"]

        cannot_be_found_states = list()
        # Make sure that each state to be restores is compatible with the current robot configuration
        for state_data in list_of_states:
            state_type = state_data["type"]
            is_available = state_type in constant_states or state_type in commander_states
            is_generated = state_type in generated_states
            # If some states can't be used as the current robot configuration does not allow to do so, extract its name
            if not (is_available or is_generated):
                cannot_be_found_states.append(state_data["name"])
            else:
                # Make sure to have an unique name. It is important, especially when pasting states with same name
                state_data["name"] = self.get_unique_name(state_data["name"])
                # Create the state and restore its configuration
                created_state = State(self, state_data["type"])
                created_state.restore(state_data, socket_mapping, new_sockets=new_sockets)

                # If is_pasted
                if offset and (isinstance(offset, list) or isinstance(offset, tuple)):
                    created_state.translate(offset[0], offset[1])

        if cannot_be_found_states:
            states_name = ", ".join(cannot_be_found_states)
            error_message("Error while restoring items", "Some states could not be loaded due to missing configuration",
                          additional_text="The states named {} will be missing.".format(states_name))

    def get_state_machine_by_name(self, name):
        """
            Return the StateMachine object with the provided name. If none is found, return None

            @param name: String corresponding to the name of the state machine to return
            @return: StateMachine if one is found, None otherwise
        """
        return next((i for i in self.state_machines if i.name == name), None)

    def restore_terminal_sockets(self, list_of_sockets, socket_mapping):
        """
            Restore the terminal sockets stored in a list of saved outcomes

            @param list_of_states: List of dictionaries obtained by calling the save() function for each terminal state
            @param socket_mapping: Dictionary mapping the id of the sockets to the pointer of the actual object
        """
        # Get the number of sockets to restore
        number_terminal_sockets_to_restore = len(list_of_sockets)
        # For each terminal socket (already created), restore their previous configuration
        for ind_sock, socket in enumerate(self.terminal_sockets):
            # If an existing terminal socket was not present in the content to be restored
            if ind_sock >= number_terminal_sockets_to_restore:
                # Remove the socket
                socket.remove()
            # Otherwise restore it to the input configuration
            else:
                socket.restore(list_of_sockets[ind_sock], socket_mapping)
        # If some terminal sockets to be restored are not already created, create them
        if ind_sock < number_terminal_sockets_to_restore:
            for extra_socket_index in range(ind_sock + 1, number_terminal_sockets_to_restore):
                # Extract position of socket
                socket_position = [list_of_sockets[extra_socket_index]["position_x"],
                                   list_of_sockets[extra_socket_index]["position_y"]]
                # Add and restore the terminal socket according to the input parameter
                self.add_terminal_socket(list_of_sockets[extra_socket_index]["name"], socket_position)
                self.terminal_sockets[-1].restore(list_of_sockets[extra_socket_index], socket_mapping)

    def restore(self, properties):
        """
            Restore the configuration of the container according to the parameters saved in properties

            @param properties: Dictionary containing the configuration of the terminal sockets, states and connectors
        """
        # Initialize the dictionary that records the mapping between the id of the sockets and the actual objects
        socket_mapping = {}

        # Extract the different information stored in properties
        terminal_sockets_data = properties["terminal_sockets"]
        states_data = properties["states"]
        state_machines_data = properties["state_machines"]
        connectors_data = properties["connectors"]
        # Remove the different elements depending on what needs to be restored
        self.clear(not state_machines_data)
        # Make sure to remove state machines that are not supposed to be there
        if state_machines_data:
            # Get the name of the one to be restored
            to_be_restored = list([x["name"] for x in state_machines_data])
            for state_machine in self.state_machines:
                if state_machine.name not in to_be_restored:
                    state_machine.remove()

        # Restore the terminal sockets
        self.restore_terminal_sockets(terminal_sockets_data, socket_mapping)

        # Restore the states
        self.restore_states(states_data, socket_mapping)

        # Restore the state machines
        for state_machine_data in state_machines_data:
            state_machine_name = state_machine_data["name"]
            # Get the state machine if already existing
            state_machine = self.get_state_machine_by_name(state_machine_name)
            # If it does not exist
            if state_machine is None:
                # Create a new subwindow
                self.editor_widget.parent().mdiArea().add_subwindow(state_machine_name, state_machine_data["type"])
                container = self.editor_widget.parent().mdiArea().focused_subwindow.widget().container
                # Create a state like representation to be displayed in the current widget
                dropped_state_machine = StateMachine(self, container)
                # Restore the state machine
                dropped_state_machine.restore(state_machine_data, socket_mapping)
            # If it does, just restore its configuration (it allows us to avoid changing other editors, and keep each
            # subwindow independant)
            else:
                state_machine.restore(state_machine_data, socket_mapping)

        # Add the connectors
        if connectors_data:
            for connector_data in connectors_data:
                if connector_data["start"] in socket_mapping and connector_data["end"] in socket_mapping:
                    Connector(self, socket_mapping[connector_data["start"]], socket_mapping[connector_data["end"]])

    @property
    def type(self):
        """
            Return the type of this container

            @return: String corresponding to the type of the container
        """
        return self._type

    @type.setter
    def type(self, container_type):
        """
            Set the type of the container and set the corresponding attributes

            @param container_type: Type of the container (string)
        """
        # Get the proper parameters
        if container_type == "base":
            self.parameters = extract_state_machine_parameters_from_file(TASK_EDITOR_ROOT_TEMPLATE)
        else:
            self.parameters = AVAILABLE_STATEMACHINES[container_type]["parameters"]
        # Set the type attribute
        self._type = container_type
