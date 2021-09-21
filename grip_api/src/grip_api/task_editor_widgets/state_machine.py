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

from grip_api.task_editor_graphics.state_machine import GraphicsStateMachine
from .state_content_widget import StateMachineContentWidget
from .socket import Socket


class StateMachine(object):

    """
        Object that gathers all the logic necessary to handle the state-like representation of state machines
    """

    def __init__(self, parent_container, def_container):
        """
            Initialize the widget and create the corresponding graphical representation

            @param parent_container: Object (Container) to which the state machine is added
            @param def_container: Object (Container) in which this state machine is defined
        """
        # Container the state machine is nested to
        self.container = parent_container
        # Container in which this state machine is being defined
        self.def_container = def_container
        # By definition the name of the container is also the name of the state machine
        self.name = self.def_container.name
        # Create the widget displayed inside the state-like representation to configure it
        self.content = StateMachineContentWidget(self)
        # Create the graphical representation of the state machine
        self.graphics_state = GraphicsStateMachine(self)
        # Add the graphical item to the graphics container
        self.container.graphics_container.addItem(self.graphics_state)
        # Parametrize the spacing between sockets
        self.socket_spacing = 80
        # Will contain the input socket, set a list to make the update easier (see update_connectors)
        self.input_socket = list()
        # Will contain all the output sockets
        self.output_sockets = list()
        # Create the sockets
        self.init_sockets()
        # Add the state machine to the container
        self.container.add_state_machine(self)

    def set_position(self, x, y):
        """
            Set the position of the object is in graphics container

            @param x: x coordinate (float or integer) of the top left corner
            @param y: y coordinate (float or integer) of the top left corner
        """
        self.graphics_state.setPos(x, y)

    def translate(self, x, y):
        """
            Translate the graphical representation of the object in the view.

            @param x: Value to add to the x coordinate of the graphical representation of the object
            @param y: Value to add to the x coordinate of the graphical representation of the object
        """
        # Get the current position
        current_x, current_y = self.graphics_state.x(), self.graphics_state.y()
        # Set position when applying the offset
        self.set_position(current_x + x, current_y + y)

    def init_sockets(self):
        """
            Create the sockets associated to the state machine
        """
        # Create a socket for input
        self.input_socket.append(Socket(state=self, socket_name="input"))
        # Get the initial outcomes
        outcomes = self.def_container.outcomes
        # Create a socket for each outcome
        for counter, item in enumerate(outcomes):
            self.output_sockets.append(Socket(state=self, index=counter, socket_name=item, multi_connections=False,
                                              count_on_this_side=len(outcomes)))

    def update_sockets_position(self):
        """
            Update the socket's position of the state machine with respect to the current outcomes of the container used
            to define the state machine
        """
        # Get the new outcomes of the definition container
        new_outcomes = self.def_container.outcomes[:]
        new_number_outcomes = len(new_outcomes)
        # Get the current nuumber of outcomes
        current_number_outcomes = len(self.output_sockets)
        # For each new outcome
        for outcome_index, outcome_name in enumerate(new_outcomes):
            # If one outcome was not registered previously, create a new socket
            if outcome_index >= current_number_outcomes:
                self.output_sockets.append(Socket(state=self, index=outcome_index, socket_name=outcome_name,
                                                  multi_connections=False, count_on_this_side=new_number_outcomes))
            # Otherwise update the socket's position
            else:
                self.output_sockets[outcome_index].update_position(new_number_outcomes)
        # If the new outcomes contain fewer elements, delete the one that should not be there anymore
        if outcome_index != current_number_outcomes - 1:
            for index in range(outcome_index + 1, current_number_outcomes):
                self.output_sockets[index].remove()
                # Make sure the element is removed from the list
                del self.output_sockets[index]
        # Update the connectors
        self.update_connectors()

    def update_connectors(self):
        """
            Function called when the graphical representation is moved by the user so that the connectors get updated
        """
        # For all the sockets part of the state machine, update the connectors
        for socket in self.input_socket + self.output_sockets:
            for connector in socket.connectors:
                connector.update_positions()

    def remove(self, is_window_closed=False):
        """
            Remove this object from the container and potentially remove the window corresponding to its definition

            @param is_window_closed: Boolean stating whether this function is called after the user closed a window
        """
        # For each socket, remove all the linked connectors and sockets
        for socket in (self.input_socket + self.output_sockets):
            for connector in socket.connectors:
                connector.remove()
            # Remove the socket as well
            socket.remove()
        # Remove the state machine from the container
        self.container.remove_state_machine(self)
        self.graphics_state = None
        # Remove all the nested state machines
        for state_machine in self.def_container.state_machines:
            state_machine.remove()
        # If this function is not called because the definition window is closed then close it
        if not is_window_closed:
            # Remove the corresponding tab in the MDI area
            self.def_container.editor_widget.parent().remove()

    def is_valid(self):
        """
            Return a boolean checking whether the state machine is valid or not

            @return: True if the state machine is valid, False otherwise
        """
        # A state machine is valid iif it is fully connected and its definition is also valid
        is_fully_connected = all(socket.is_connected() for socket in (self.input_socket + self.output_sockets))
        return is_fully_connected and self.def_container.is_valid

    def get_userdata(self):
        """
            Return the userdata configured by the user

            @return: String to be added in the container definition
        """
        # Get the information from the content of the state-like representation
        user_specified = self.content.config_state.get_slot_config("userdata")
        # If nothing has been entered by the user, then set a default value which leads to inherit the parent's userdata
        if not user_specified:
            user_specified = "self.userdata"
        return user_specified

    def get_config(self):
        """
            Return a dictionary containing the configuration of the state machine. It contains all container related
            information and transitions

            @return: Dictionary containing the information of the state machine and its transitions
        """
        # Get the information of the container that defines this state-like representation
        container_information = self.def_container.get_parsed_container()
        # Extract the transitions
        transitions = dict()
        for socket in self.output_sockets:
            if socket.connectors[0].end_socket.is_terminal:
                transitions[socket.name] = socket.connectors[0].end_socket.name
            else:
                transitions[socket.name] = str(socket.connectors[0].end_socket.state.name)
        # Add the transitions
        container_information["transitions"] = transitions
        # Return the configuration
        return container_information

    def get_opacity(self):
        """
            Return the opacity (which represents the availability of the object) of the graphical state machine

            @return: Float value between 0 and 1
        """
        return self.graphics_state.opacity()

    def set_opacity(self, value):
        """
            Set the opacity of the graphical state machine

            @param value: Float between 0 and 1
        """
        # Make sure the opacity to set is between 0 and 1
        value = 0 if value < 0 else value if value < 1 else 1
        self.graphics_state.setOpacity(value)

    def save(self):
        """
            Save the current properties of the object so it can be restored later on

            @return: Dictionary containing the configuration of the input nad output sockets, type, name and position of
                     the state machine
        """
        # Position of the graphical representation of the state
        pos = self.graphics_state.pos()
        # Get configuration of all the sockets
        input_socket, output_sockets = list(), list()
        for socket in self.input_socket:
            input_socket.append(socket.get_id())
        for socket in self.output_sockets:
            output_sockets.append(socket.get_id())

        return dict([
            ("name", self.name),
            ("type", self.def_container.type),
            ("pos_x", pos.x()),
            ("pos_y", pos.y()),
            ("input_socket", input_socket),
            ("output_sockets", output_sockets)
        ])

    def restore(self, properties, socket_mapping={}):
        """
            Set all the parameters of the state machine to restore it to a previously saved state

            @param data: Dictionary containing the input nad output sockets, type, name and position of
                         the state machine
            @param socket_mapping: Dictionary mapping the id of the sockets to the pointer of the actual object
        """
        # Set the position
        self.set_position(properties["pos_x"], properties["pos_y"])
        # Set the id of the input socket
        self.input_socket[0].set_id(properties["input_socket"][0], socket_mapping)
        # Update all the sockets
        for ind_socket, socket in enumerate(self.output_sockets):
            socket.set_id(properties["output_sockets"][ind_socket], socket_mapping)
        # Link this object to the container in which the state machine is defined
        self.def_container.set_state_like(self)
