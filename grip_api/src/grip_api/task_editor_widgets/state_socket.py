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
from grip_api.task_editor_graphics.socket import GraphicsSocket
from grip_api.utils.formatted_print import format_raise_string


class StateSocket(AbstractSocket):
    """
        Object gathering all the logic related to sockets that are directly linked to a State
    """

    def __init__(self, state: 'State', socket_name: str, index: int = 0, multi_connections: bool = True) -> None:
        """
            Initialize the widget and set the graphical representation

            @param state: Object instantiated from the class State that will contain this socket
            @param socket_name: Name of the socket
            @param index: Index of the socket on the side of the state
            @param multi_connections: Whether the socket accepts multiple connections or not
        """
        super().__init__(socket_name, index, multi_connections, is_terminal=False, is_starting=multi_connections)
        # Store all the information
        self.state = state
        self.graphics_socket = GraphicsSocket(self)

    @AbstractSocket.graphics_socket.setter
    def graphics_socket(self, graphical_socket: GraphicsSocket) -> None:
        """
            Set the graphical representation of the socket

            @param graphical_socket: Instance of the GraphicsSocket class
        """
        if not isinstance(graphical_socket, GraphicsSocket):
            raise TypeError(format_raise_string("The property 'graphics_socket' must be an instance of GraphicsSocket"))
        self._graphics_socket = graphical_socket

    @AbstractSocket.name.setter
    def name(self, name_string: str) -> None:
        """
            Set the name of the socket and update the tool tip legend of its graphical representation

            @param name_string: New name to be given to the socket
        """
        # This syntax is mandatory to call the parent's property setter
        super(StateSocket, type(self)).name.fset(self, name_string)
        # If a graphical socket exists, set the tooltip accordingly
        if self.graphics_socket is not None:
            self.graphics_socket.setToolTip(name_string)

    @property
    def position(self) -> List[float]:
        """
            Return the position of the socket in the graphical view

            @return: Current position of the socket following the format [x, y]
        """
        return self.state.get_socket_position(self)

    def update_position(self) -> None:
        """
            Method updating the position of the object in the graphical view
        """
        self.graphics_socket.setPos(*self.position)

    def remove(self) -> None:
        """
            Remove this object from the graphics container
        """
        # Make sure no connectors are linked to the socket
        self.remove_all_connectors()
        # Remove the graphics representation from the graphics container
        self.state.container.graphics_container.removeItem(self.graphics_socket)
        del self.graphics_socket

    def get_id(self) -> id:
        """
            Return the ID of the socket (required for restoring the connectors)

            @return: ID number of the object generated via the built-in id() function
        """
        return self.socket_id

    def set_socket_id(self, socket_id: id, socket_mapping: Dict[id, 'StateSocket']) -> None:
        """
            Set the ID of the socket and updates the provided socket_mapping object

            @param id: ID of the socket
            @param socket_mapping: Dictionary mapping the id of sockets to the actual objects
        """
        self.socket_id = socket_id
        self.register_id(socket_mapping)

    def register_id(self, socket_mapping: Dict[id, 'StateSocket']) -> None:
        """
            Add an entry to the input dictionary with the ID of the object as key and a pointer to this object as value

            @param socket_mapping: Dictionary
        """
        socket_mapping[self.socket_id] = self
