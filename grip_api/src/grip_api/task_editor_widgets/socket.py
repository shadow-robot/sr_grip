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

from grip_api.task_editor_graphics.socket import GraphicsSocket


class Socket(object):

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
        # Store the id of the object
        self.id = id(self)
        # Store all the information
        self.state = state
        self.index = index
        self.name = socket_name
        self.is_multi_connected = multi_connections
        self.count_on_this_side = count_on_this_side
        # Compute the position of the socket wrt the state dimensions
        self.position = self.get_position()
        # Create the graphical representation of the socket
        self.graphics_socket = GraphicsSocket(self)
        # Set its pose in the view
        self.graphics_socket.setPos(*self.position)
        # List of all the connectors set to this socket
        self.connectors = list()
        # Allows to recognize terminal sockets from others when releasing edges
        self.is_terminal = False

    def get_position(self):
        """
            Get the position of the socket

            @return: List (x,y) of the position of the socket
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
            x, y = self.state.graphics_state.boundingRect().width() / 2., 0
        # Otherwise, depending on how many there are, compute there position given the state socket spacing
        else:
            # Add it at the bottom
            y = self.state.graphics_state.boundingRect().height()
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
            x = node_width / 2. + self.index * scaled_socket_space - total_number_of_spaces / 2. * scaled_socket_space

        return [x, y]

    def set_name(self, name):
        """
            Set the name of the socket and update the text displayed by the graphical ToolTip

            @param name: String corresponding to the new name of the socket
        """
        self.name = name
        self.graphics_socket.setToolTip(self.name)

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

    def add_connector(self, connector):
        """
            Add a provided connector to this socket

            @param connector: Connector object to be added to the attribute connectors
        """
        self.connectors.append(connector)

    def remove_connector(self, connector):
        """
            Remove a provided connector from this socket

            @param connector: Connector object to be removed from the attribute connectors
        """
        # Make sure it is part of the socket to avoid any exception
        if connector in self.connectors:
            self.connectors.remove(connector)

    def remove_all_connectors(self):
        """
            Remove all the connectors associated to this socket
        """
        # Using a while loop because some connectors were not properly deleted with a for loop for an unknown reason
        while self.connectors:
            connector = self.connectors.pop(0)
            connector.remove()

    def remove(self):
        """
            Remove this object from the graphics container
        """
        # Make sure no connectors are linked to the socket
        self.remove_all_connectors()
        # Remove the graphics representation from the graphics container
        self.state.container.graphics_container.removeItem(self.graphics_socket)
        self.graphics_socket = None

    def is_connected(self):
        """
            Return a boolean stating if the socket currently hosts a connector

            @return: True if the socket contains at least a connector, otherwise False
        """
        return not not self.connectors

    def get_id(self):
        """
            Return the ID of the socket (required for restoring the connectors)

            @return: ID number of the object generated via the built-in id() function
        """
        return self.id

    def set_id(self, id, socket_mapping={}):
        """
            Set the ID of the socket and update the optionally provided socket_mapping object

            @param id: ID of the socket
            @param socket_mapping: Dictionary mapping the id of sockets to the actual objects
        """
        self.id = id
        self.register_id(socket_mapping)

    def register_id(self, socket_mapping):
        """
            Add an entry to the input dictionary with the ID of the object as key and a pointer to this object as value

            @param socket_mapping: Dictionary
        """
        socket_mapping[self.id] = self
