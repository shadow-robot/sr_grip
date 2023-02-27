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

from grip_api.abstracts_widgets.abstract_socket import AbstractSocket
from grip_api.task_editor_graphics.socket import GraphicsSocket


class Socket(AbstractSocket):
    """
        Object gathering all the logic related to sockets that are directly linked to a State
    """
    def __init__(self, state, socket_name, index=0, multi_connections=True, count_on_this_side=1):
        """
            Initialize the widget and set the graphical representation

            @param state: Object (State) on which this widget is set
            @param socket_name: Name of the socket (string)
            @param index: Integer, stating the index of the socket on this side of the node
            @param multi_connections: Boolean stating if the socket accepts multiple connections
            @param count_on_this_side: Total number of sockets on this side of the node
        """
        super().__init__(socket_name, index, multi_connections)
        # Store all the information
        self._state = state
        # Create and store the graphical socket to be displayed
        self.count_on_this_side = count_on_this_side
        self.graphics_socket = GraphicsSocket(self)
        self.update_position(count_on_this_side)

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, state_object):
        if hasattr(state_object, "graphics_state"):
            self._state = state_object
        else:
            raise TypeError('state must be object of type State')

    @property
    def count_on_this_side(self):
        return self._count_on_this_side

    @count_on_this_side.setter
    def count_on_this_side(self, value):
        if isinstance(value, int):
            self._count_on_this_side = value
        else:
            raise TypeError('count_on_this_side must be an int')

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, position_x_y):
        if isinstance(position_x_y, list) and len(position_x_y) == 2:
            if isinstance(position_x_y[0], (int, float)) and isinstance(position_x_y[1], (int, float)):
                self._position = position_x_y
            else:
                raise ValueError('position values must be floats')
        else:
            raise TypeError('position must be list with elements position_x and position_y')

    def get_position(self):
        """
            Get the position of the socket

            @return: List (position_x,position_y) of the position of the socket
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

    def update_position(self, count_on_side):
        """
            Update the position of the socket with respect to the box-like representation

            @param count_on_side: New number of sockets on the given side of the state
        """
        # Update the object's attribute
        self.count_on_this_side = count_on_side
        # Recompute position of the socket
        self.position = self.get_position()
        # Change the position of the graphical representation
        self.graphics_socket.setPos(*self.position)

    def update_name(self, name):
        """
            Set the name of the socket and update the text displayed by the graphical ToolTip

            @param name: String corresponding to the new name of the socket
        """
        self.name = name
        self.graphics_socket.setToolTip(self.name)

    def remove(self):
        """
            Remove this object from the graphics container
        """
        # Make sure no connectors are linked to the socket
        self.remove_all_connectors()
        # Remove the graphics representation from the graphics container
        self.state.container.graphics_container.removeItem(self.graphics_socket)
        self.graphics_socket = None

    def get_id(self):
        """
            Return the ID of the socket (required for restoring the connectors)

            @return: ID number of the object generated via the built-in id() function
        """
        return self.socket_id

    def set_socket_id(self, socket_id, socket_mapping):
        """
            Set the ID of the socket and updates the provided socket_mapping object

            @param id: ID of the socket
            @param socket_mapping: Dictionary mapping the id of sockets to the actual objects
        """
        self.socket_id = socket_id
        self.register_id(socket_mapping)

    def register_id(self, socket_mapping):
        """
            Add an entry to the input dictionary with the ID of the object as key and a pointer to this object as value

            @param socket_mapping: Dictionary
        """
        socket_mapping[self.socket_id] = self
