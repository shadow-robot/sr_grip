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

from grip_api.task_editor_graphics.connector import GraphicsConnector
from graphical_editor_base import Serializable
from terminal_socket import TerminalSocket


class Connector(Serializable):

    """
        Object that connects two states together through sockets
    """

    def __init__(self, container, start_socket=None, end_socket=None):
        """
            Initialize the object and set the graphical representation

            @param container: Container object that this object will be added to
            @param start_socket: First socket on which the user clicked to create this connector. Default is None
            @param end_socket: Target socket of this connector. Default is None
        """
        super(Connector, self).__init__()
        self.container = container
        # Default initialization
        self._start_socket = None
        self._end_socket = None
        # Use the property set to automatically carry out the different tests that go with these operations
        self.start_socket = start_socket
        self.end_socket = end_socket
        # Add the connector to the container
        self.container.add_connector(self)
        # Create the graphical representation and adds it to the graphical container
        self.graphics_connector = GraphicsConnector(self)
        self.container.graphics_container.addItem(self.graphics_connector)
        # Make sure the position of the connector's socket(s) is/are updated
        self.update_positions()

    @property
    def start_socket(self):
        """
            Return the start socket

            @return: Socket object set as the starting one for this connector
        """
        return self._start_socket

    @property
    def end_socket(self):
        """
            Return the end socket

            @return: Socket object set as the ending one for this connector
        """
        return self._end_socket

    @start_socket.setter
    def start_socket(self, socket):
        """
            Set the start socket to a provided one

            @param socket: Socket object to set as starting socket for this connector
        """
        # If the connector was previously assigned to another starting socket, delete it from the old socket
        if self._start_socket is not None:
            self._start_socket.remove_connector(self)

        # Assign a new starting socket
        self._start_socket = socket
        # Make sure that the input socket is not None and add the connector to the newly assigned socket
        if self.start_socket is not None:
            self.start_socket.add_connector(self)

    @end_socket.setter
    def end_socket(self, socket):
        """
            Set the end socket to a provided one

            @param socket: Socket object to set as ending socket for this connector
        """
        # If the connector was previously assigned to another socket, delete it from the old socket
        if self._end_socket is not None:
            self._end_socket.remove_connector(self)

        # Assign a new end socket
        self._end_socket = socket
        # Make sure that the input socket is not None and add the connector to the newly assigned socket
        if self.end_socket is not None:
            self.end_socket.add_connector(self)

    def update_positions(self):
        """
            Compute the proper source and destination positions the connector should link in the view
        """
        # Get the position of the socket in its local reference
        source_pos = self.start_socket.get_position()
        # Add it to the graphical representation's actual position in the view
        # The source pos can either be a Socket or a TerminalSocket
        if isinstance(self.start_socket, TerminalSocket):
            source_pos[0] = self.start_socket.graphics_socket.pos().x()
            source_pos[1] = self.start_socket.graphics_socket.pos().y()
        else:
            source_pos[0] += self.start_socket.state.graphics_state.pos().x()
            source_pos[1] += self.start_socket.state.graphics_state.pos().y()
        # Set it to the graphical representation
        self.graphics_connector.set_source(*source_pos)
        # If the connector is not being dragged, do the same of the destination position
        if self.end_socket is not None:
            end_pos = self.end_socket.get_position()
            # The end pos can either be another Socket or a TerminalSocket
            if isinstance(self.end_socket, TerminalSocket):
                end_pos[0] = self.end_socket.graphics_socket.pos().x()
                end_pos[1] = self.end_socket.graphics_socket.pos().y()
            else:
                end_pos[0] += self.end_socket.state.graphics_state.pos().x()
                end_pos[1] += self.end_socket.state.graphics_state.pos().y()
            self.graphics_connector.set_destination(*end_pos)
        else:
            self.graphics_connector.set_destination(*source_pos)
        # Repaint its graphical representation with the proper coordinates
        self.graphics_connector.update()

    def remove(self):
        """
            Remove the object from the container
        """
        self.end_socket = None
        self.start_socket = None
        self.container.remove_connector(self)
        self.graphics_connector = None

    def is_valid(self):
        """
            Return a boolean stating if one of the tip of the connector is loose (i.e. the connector is being dragged)

            @return: True if the connectors links two sockets, otherwise False
        """
        # We only need to check on the end socket knowing that this object can be created iif we have a starting socket
        return self.end_socket is not None
