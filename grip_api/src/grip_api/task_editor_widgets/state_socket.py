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

from typing import List, Dict
from grip_api.abstract_widgets.abstract_socket import AbstractSocket
from grip_api.task_editor_widgets.state import State
from grip_api.task_editor_widgets.state_socket import StateSocket
from grip_api.task_editor_graphics.socket import GraphicsSocket
from grip_api.utils.formatted_print import format_raise_string


class StateSocket(AbstractSocket):
    """
        Object gathering all the logic related to sockets that are directly linked to a State
    """
    def __init__(self, state: State, socket_name: str, index: int = 0, multi_connections: bool = True,
                 count_on_this_side: int = 1) -> None:
        """
            Initialize the widget and set the graphical representation

            @param state: Object instanciated from the class State that will contain this socket
            @param socket_name: Name of the socket
            @param index: Index of the socket on the side of the state
            @param multi_connections: Whether the socket accepts multiple connections or not
            @param count_on_this_side: Total number of sockets on this side of the node
        """
        super().__init__(socket_name, index, multi_connections, is_terminal=False)
        # Store all the information
        self._state = state
        # Create and store the graphical socket to be displayed
        self.count_on_this_side = count_on_this_side
        self.graphics_socket = GraphicsSocket(self)
        self.position = count_on_this_side

    @graphics_socket.setter
    def graphics_socket(self, graphical_socket: GraphicsSocket) -> None:
        """
            Set the graphical representation of the socket

            @param graphical_socket: Instance of the GraphicsSocket class
        """
        if not isinstance(graphical_socket, GraphicsSocket):
            raise TypeError(format_raise_string("The property 'graphics_socket' must be an instance of GraphicsSocket"))
        self._graphics_socket = graphical_socket

    @property
    def position(self) -> List[float]:
        """
            Return the position of the socket in the graphical view

            @return: Current position of the socket in the following format [x, y]
        """
        # Since we can zoom out even after the state gets to its minimal size, we need to get a compensation factor
        # to keep the distance between the sockets constant
        if self.state.graphics_state.zoom < self.state.graphics_state.zoom_threshold:
            compensation_zoom = self.state.graphics_state.zoom_threshold - self.state.graphics_state.zoom
            compensation_factor = self.state.graphics_state.zoom_multiplier**compensation_zoom
        else:
            compensation_factor = 1
        # If the socket is used as an input one, set it at the top center
        if self.is_multi_connected:
            position_x, position_y = self.state.graphics_state.boundingRect().width() / 2., 0
        # Otherwise, depending on how many there are, compute there position given the state socket spacing
        else:
            # Add it at the bottom
            position_y = self.state.graphics_state.boundingRect().height()
            node_width = self.state.graphics_state.boundingRect().width()
            total_number_of_spaces = self.count_on_this_side - 1
            # Make sure to adapt the value of the socket spacing so it does not go over the node width
            if total_number_of_spaces * self.state.socket_spacing * compensation_factor >= node_width:
                socket_space = (node_width * 1./compensation_factor - 32) / total_number_of_spaces
            else:
                socket_space = self.state.socket_spacing
            # Scale it
            scaled_socket_space = socket_space * compensation_factor
            # Uniformly spread the sockets on the width of the state
            position_x = (node_width / 2. + self.index * scaled_socket_space -
                          total_number_of_spaces / 2. * scaled_socket_space)

        return [position_x, position_y]

    @position.setter
    def position(self, count_on_side: int) -> None:
        """
            Update the position of the socket in the graphical view

            @param count_on_side: Number of sockets that are on the same side of the state than this object
        """
        # Update the object's attribute
        self.count_on_this_side = count_on_side
        # Recompute position of the socket
        self.update_position()

    @property
    def state(self) -> State:
        return self._state

    @state.setter
    def state(self, state_object: State) -> None:
        if hasattr(state_object, "graphics_state"):
            self._state = state_object
        else:
            raise TypeError("The attribute 'state' must be an object of type State")

    @property
    def count_on_this_side(self) -> int:
        return self._count_on_this_side

    @count_on_this_side.setter
    def count_on_this_side(self, value: int) -> None:
        if isinstance(value, int) and value > 0:
            self._count_on_this_side = value
        else:
            raise TypeError("The attribute 'count_on_this_side' must be a positive integer")

    def update_name(self, new_name: str) -> None:
        """
            Set the name of the socket and update the text displayed by the graphical ToolTip

            @param new_name: String corresponding to the new name of the socket
        """
        self.name = new_name
        self.graphics_socket.setToolTip(self.name)

    def remove(self) -> None:
        """
            Remove this object from the graphics container
        """
        # Make sure no connectors are linked to the socket
        self.remove_all_connectors()
        # Remove the graphics representation from the graphics container
        self.state.container.graphics_container.removeItem(self.graphics_socket)
        self.graphics_socket = None

    def get_id(self) -> id:
        """
            Return the ID of the socket (required for restoring the connectors)

            @return: ID number of the object generated via the built-in id() function
        """
        return self.socket_id

    def set_socket_id(self, socket_id: id, socket_mapping: Dict[id, StateSocket]) -> None:
        """
            Set the ID of the socket and updates the provided socket_mapping object

            @param id: ID of the socket
            @param socket_mapping: Dictionary mapping the id of sockets to the actual objects
        """
        self.socket_id = socket_id
        self.register_id(socket_mapping)

    def register_id(self, socket_mapping: Dict[id, StateSocket]) -> None:
        """
            Add an entry to the input dictionary with the ID of the object as key and a pointer to this object as value

            @param socket_mapping: Dictionary
        """
        socket_mapping[self.socket_id] = self
