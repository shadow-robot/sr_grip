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

from graphical_editor_base import Serializable
from grip_api.task_editor_graphics.socket import GraphicsSocket


class Socket(Serializable):

    """
        Object gathering all the logic realted to sockets that are directlt linked to a State
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
        super(Socket, self).__init__()
        # Store all the information
        self.state = state
        self.index = index
        self.name = socket_name
        self.is_multi_connected = multi_connections
        self.count_on_this_side = count_on_this_side
        # Compute the position of the state
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
            scaled_socket_space = self.state.socket_spacing * compensation_factor
            # Uniformely spread the sockets on the width of the state
            x = node_width / 2. + self.index * scaled_socket_space - total_number_of_spaces / 2. * scaled_socket_space

        return [x, y]

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
        # Remove the graphics representation from the graphics container
        self.state.container.graphics_container.removeItem(self.graphics_socket)
        self.graphics_socket = None

    def is_connected(self):
        """
            Return a boolean stating if the socket currently hosts a connector

            @return: True if the socket contains at least a connector, otherwise False
        """
        return not not self.connectors
