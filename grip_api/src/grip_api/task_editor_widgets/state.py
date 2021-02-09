#!/usr/bin/env python

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

from grip_api.task_editor_graphics.state import GraphicsState
from graphical_editor_base import Serializable
from state_content_widget import StateContentWidget
from grip_core.state_machine_generator.state_generator import generate_state
from socket import Socket
import os


class State(Serializable):

    """
        Object that gathers all the logic necessary to handle states
    """

    def __init__(self, container, type="Undefined state"):
        """
            Initialize the widget and create the corresponding graphical representation

            @param container: Object (Container) to which the state is added
            @param type: Type (string) of the state (e.g. Move, Plan, etc.)
        """
        super(State, self).__init__()
        self.container = container
        # When added, by default the name of the state is its type
        self.type = type
        self.name = type
        # By default, should be False and will potentially be set to True when filling its content
        self.to_generate = False
        # Create the widget displayed inside the state to configure it
        self.content = StateContentWidget(self)
        # Create the graphical representation of the state
        self.graphics_state = GraphicsState(self)
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
        # Add the state to the container
        self.container.add_state(self)

    def set_position(self, x, y):
        """
            Set the position of the object is in graphics container

            @param x: x coordinate (float or integer) of the top left corner
            @param y: y coordinate (float or integer) of the top left corner
        """
        self.graphics_state.setPos(x, y)

    def init_sockets(self):
        """
            Create the sockets associated to the state
        """
        # Create a socket for input
        self.input_socket.append(Socket(state=self, socket_name="input"))
        # Get the initial outcomes
        outcomes = self.content.get_outcomes()
        # Create a socket for each outcome
        for counter, item in enumerate(outcomes):
            self.output_sockets.append(Socket(state=self, index=counter, socket_name=item, multi_connections=False,
                                              count_on_this_side=len(outcomes)))

    def update_connectors(self):
        """
            Function called when the graphical representation is moved by the user so that the connectors get updated
        """
        # For all the sockets part of the state, update the connectors
        for socket in self.input_socket + self.output_sockets:
            for connector in socket.connectors:
                connector.update_positions()

    def remove(self):
        """
            Remove this object from its corresponding container
        """
        # For each socket, remove all the linked connectors and sockets
        for socket in (self.input_socket + self.output_sockets):
            for connector in socket.connectors:
                connector.remove()
            # Remove the socket as well
            socket.remove()
        # Remove the state from the container
        self.container.remove_state(self)
        self.graphics_state = None

    def is_valid(self):
        """
            Return a boolean corresponding to whether the state is fully connected or not

            @return: True if all sockets of the state are connected to other objects, False otherwise
        """
        return all(socket.is_connected() for socket in (self.input_socket + self.output_sockets))

    def get_config(self):
        """
            Return a dictionary containing the configuration of the state. It contains its source, parameters, outcomes
            and its transitions

            @return: Dictionary containing at least the keys "source", "outcomes" and "transitions"
        """
        # Generate the state if required
        if self.to_generate:
            generate_state(self.content.state_info)
        # Will contain the configuration of the state (i.e. source, outcomes, parameters and transitions)
        state_config = {}
        # Extract the different information from the state's content
        for parameter_name in self.content.state_info["parameters"].keys():
            # Make sure to only extract displayed parameters
            if parameter_name not in ["outcomes", "input_keys", "output_keys", "io_keys"]:
                # If the user has specified a value for the given parameter, store it
                user_config = self.content.config_state.get_slot_config(parameter_name)
                state_config[parameter_name] = user_config
                # If the value needs to be set as an input key of the state
                if user_config in self.container.output_userdata:
                    state_config["input_keys"] = [user_config]

        # Make sure to register all the potential output keys for external states
        if self.to_generate and state_config["output"] and not state_config["output_type"]:
            self.container.output_userdata.append(state_config["output"])
            state_config["output_keys"] = [state_config["output"]]

        # Get the source of the state
        state_config["source"] = os.path.basename(self.content.state_info["source"]).split(".")[0]
        # Get the outcomes
        state_config["outcomes"] = self.content.get_outcomes()
        # Get the import statement
        state_config["import_statement"] = self.content.state_info["import_statement"]
        # Extract the transitions
        transitions = dict()
        # For each output socket, extract the transition between this state and the following element
        for socket in self.output_sockets:
            # If it is connected to a terminal socket of the container, then refer to its name directly, otherwise get
            # the connected state's name
            if socket.connectors[0].end_socket.is_terminal:
                transitions[socket.name] = str(socket.connectors[0].end_socket.name)
            else:
                transitions[socket.name] = str(socket.connectors[0].end_socket.state.name)
        # Register the transitions
        state_config["transitions"] = transitions
        return state_config
