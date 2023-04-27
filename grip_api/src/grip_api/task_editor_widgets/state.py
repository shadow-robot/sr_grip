#!/usr/bin/env python3

# Copyright 2020, 2021, 2023 Shadow Robot Company Ltd.
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
from grip_api.task_editor_graphics.state import GraphicsState
from grip_core.state_machine_generator.state_generator import generate_state
from .state_content_widget import StateContentWidget
from grip_api.task_editor_widgets.state_socket import StateSocket


class State:

    """
        Object that gathers all the logic necessary to handle states
    """

    def __init__(self, container, type="Undefined state"):
        """
            Initialize the widget and create the corresponding graphical representation

            @param container: Object (Container) to which the state is added
            @param type: Type (string) of the state (e.g. Move, Plan, etc.)
        """
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
        # Get the initial outcomes
        self.outcomes = self.content.get_outcomes()
        # Will contain the input socket, set a list to make the update easier (see update_connectors)
        self.input_socket = []
        # Will contain all the output sockets
        self.output_sockets = []
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

    def update_name(self, new_name):
        """
            Update the name of the state, making sure that two items in one container don't have the exact same name

            @param new_name: Name to be given to the state
        """
        # Make sure the name is unique
        self.name = self.container.get_unique_name(new_name)
        # Store the current history
        self.container.history.store_current_history()

    def get_socket_position(self, socket):
        """
            Return the position of an input socket, part of the state, in the view

            @param socket: Socket (StateSocket) to get the position of
            @return: Current position of the socket following the format [x, y]
        """
        # Since we can zoom out even after the state gets to its minimal size, we need to get a compensation factor
        # to keep the distance between the sockets constant
        if self.graphics_state.zoom < self.graphics_state.zoom_threshold:
            compensation_zoom = self.graphics_state.zoom_threshold - self.graphics_state.zoom
            compensation_factor = self.graphics_state.zoom_multiplier**compensation_zoom
        else:
            compensation_factor = 1
        # If the socket is used as an input one, set it at the top center
        if socket.is_starting:
            position_x, position_y = self.graphics_state.boundingRect().width() / 2., 0
        # Otherwise, depending on how many there are, compute the position given the state socket spacing
        else:
            # Add it at the bottom
            position_y = self.graphics_state.boundingRect().height()
            node_width = self.graphics_state.boundingRect().width()
            total_number_of_spaces = len(self.outcomes) - 1
            # Make sure to adapt the value of the socket spacing so it does not go over the node width
            if total_number_of_spaces * self.socket_spacing * compensation_factor >= node_width:
                socket_space = (node_width * 1./compensation_factor - 32) / total_number_of_spaces
            else:
                socket_space = self.socket_spacing
            # Scale it
            scaled_socket_space = socket_space * compensation_factor
            # Uniformly spread the sockets on the width of the state
            position_x = (node_width / 2. + socket.index * scaled_socket_space -
                          total_number_of_spaces / 2. * scaled_socket_space)

        return [position_x, position_y]        

    def init_sockets(self):
        """
            Create the sockets associated to the state
        """
        # Create a socket for input
        self.input_socket.append(StateSocket(state=self, socket_name="input"))
        # Create a socket for each outcome
        for counter, item in enumerate(self.outcomes):
            self.output_sockets.append(StateSocket(state=self, index=counter, socket_name=item,
                                                   multi_connections=False))

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
        # Remove each socket
        for socket in (self.input_socket + self.output_sockets):
            socket.remove()
        # Remove the state from the container
        self.container.remove_state(self)
        self.graphics_state = None

    def is_valid(self):
        """
            Return a boolean representing the validity of the object, i.e. if it is fully connected and compatible
            with the current robot configuration

            @return: True if the state is valid, False otherwise
        """
        is_fully_connected = all(socket.is_connected() for socket in (self.input_socket + self.output_sockets))
        return is_fully_connected and self.get_opacity() == 1.

    def get_config(self):
        """
            Return a dictionary containing the configuration of the state. It contains its source, parameters, outcomes
            and its transitions

            @return: Dictionary containing at least the keys "source", "outcomes" and "transitions"
        """
        # Generate the state if required
        if self.to_generate:
            # When generating a state related to a sensor, we need to know the topic we are listening to
            if "data_topics" in self.content.state_info:
                # Fill in the parameters slot with the topic name provided by the user
                user_topic_name = self.content.config_state.get_slot_config("sensor_topic")
                self.content.state_info["parameters"]["sensor_topic"] = user_topic_name
            generate_state(self.content.state_info)

        # Will contain the configuration of the state (i.e. source, outcomes, parameters and transitions)
        # But for now just get the configuration of the state
        state_config = self.content.get_config()
        state_config["input_keys"] = []
        # If some values needs to be set as an input key of the state
        for _, user_config in state_config.items():
            if user_config in self.container.output_userdata:
                state_config["input_keys"].append(user_config)

        # Make sure to register all the potential output keys for all the states with the keyword "output"
        if self.to_generate:
            output_value = state_config["output"]
            if (output_value and "sensor_topic" in state_config) or (output_value and not state_config["output_type"]):
                self.container.output_userdata.append(output_value)
                state_config["output_keys"] = [output_value]
        elif "output" in state_config:
            output_value = state_config["output"]
            if output_value:
                self.container.output_userdata.append(output_value)
                state_config["output_keys"] = [output_value]

        # If the state reinitialises the managers, make sure to get the latest config file path of each editor
        if "js_file" in self.content.state_info["parameters"]:
            task_editor_area = self.container.editor_widget.parent().parent().parent().parent()
            robot_integration_area = task_editor_area.framework_gui.robot_integration_area
            settings_config = robot_integration_area.settings_config_widget
            state_config["js_file"] = settings_config.named_joint_states.file_path
            state_config["pose_file"] = settings_config.named_poses.file_path
            state_config["traj_file"] = settings_config.named_trajectories.file_path

        # Get the source of the state
        state_config["source"] = os.path.basename(self.content.state_info["source"]).split(".")[0]
        # Get the outcomes
        state_config["outcomes"] = self.content.get_outcomes()
        # Get the import statement
        state_config["import_statement"] = self.content.state_info["import_statement"]
        # Extract the transitions
        transitions = {}
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

    def get_opacity(self):
        """
            Return the opacity (which represents the availability of the object) of the graphical state

            @return: Float value between 0 and 1
        """
        return self.graphics_state.opacity()

    def set_opacity(self, value):
        """
            Set the opacity of the graphical state

            @param value: Float between 0 and 1
        """
        # Make sure the opacity to set is between 0 and 1
        value = 0 if value < 0 else value if value < 1 else 1
        self.graphics_state.setOpacity(value)

    def save(self):
        """
            Gather and return all the parameters required to recreate the exact same state

            @return: Dictionary containing the id, name, type, position, sockets and content of the state
        """
        # Get lists containing the serialized input and outputs sockets
        serialized_input_socket, serialized_output_sockets = [], []
        for socket in self.input_socket:
            socket_id = socket.get_id()
            serialized_input_socket.append(socket_id)
        for socket in self.output_sockets:
            socket_id = socket.get_id()
            serialized_output_sockets.append(socket_id)

        return dict([
            ('name', self.name),
            ('type', self.type),
            ('pos_x', self.graphics_state.scenePos().x()),
            ('pos_y', self.graphics_state.scenePos().y()),
            ('input_socket', serialized_input_socket),
            ('output_sockets', serialized_output_sockets),
            ('content', self.content.get_config(True)),
        ])

    def restore(self, data, socket_mapping={}, new_sockets=None):
        """
            Set all the parameters of the state to restore it to a previously saved state

            @param data: Dictionary containing the id, name, type, position, sockets and content of the state
            @param socket_mapping: Dictionary mapping the id of the sockets to the pointer of the actual object
            @param new_sockets: Dictionary that will contain the mapping between the old and new socket IDs. If set to
                                None, new socket IDs won't be generated.
        """
        # Set the position
        self.set_position(data["pos_x"], data["pos_y"])
        # Set the name of the state
        self.name = data["name"]
        # If new socket IDs must be generated
        if new_sockets is not None:
            # The sockets created in the __init__ will naturally have a different ID number that the one to be restored
            self.input_socket[0].register_id(socket_mapping)
            # Register the mapping between the old and new sockets
            new_sockets[data["input_socket"][0]] = self.input_socket[0].get_id()
            # Do the same thing for the output sockets
            for ind_socket, socket in enumerate(self.output_sockets):
                socket.register_id(socket_mapping)
                new_sockets[data["output_sockets"][ind_socket]] = self.output_sockets[ind_socket].get_id()
        # Otherwise, restore the ID of the sockets of this object
        else:
            self.input_socket[0].set_socket_id(data["input_socket"][0], socket_mapping)
            # Get the number of output sockets
            number_output_socket = len(data["output_sockets"])
            for ind_socket, socket in enumerate(self.output_sockets):
                # Make sure not to break if there is any discrepancy between the current configuration and the saved one
                if ind_socket < number_output_socket:
                    socket.set_socket_id(data["output_sockets"][ind_socket], socket_mapping)
        # Restore the content of the state
        self.content.set_config(data["content"])
