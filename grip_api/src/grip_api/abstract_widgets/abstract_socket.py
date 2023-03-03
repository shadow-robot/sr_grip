#!/usr/bin/env python3

# Copyright 2023 Shadow Robot Company Ltd.
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

from abc import ABC, abstractmethod


class AbstractSocket(ABC):

    """
        Abstract Socket class with set of variables and methods common to Socket and TerminalSocket
    """

    def __init__(self, socket_name, index=0, multi_connections=True, is_terminal=False):
        """
            Abstract widget initialisation

            @param socket_name: Name of the socket (string)
            @param index: Index of the socket
            @param multi_connections: Boolean stating if the socket accepts multiple connections
        """
        # Store the id of the object
        self.socket_id = id(self)
        # Store all the information
        self.name = socket_name
        self.index = index
        self.is_multi_connected = multi_connections
        # List of all the connectors set to this socket
        self.connectors = []
        # Allows to recognize terminal sockets from others when releasing edges
        self.is_terminal = is_terminal
        # Socket position in the graphical interface
        self.position = [0.,0.]

    @property
    def socket_id(self):
        return self._socket_id

    @socket_id.setter
    def socket_id(self, socket_id):
        if not isinstance(socket_id, int):
            raise TypeError("The attribute 'socket_id' must be of unique integer")

        self._socket_id = socket_id

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, name_string):
        if not isinstance(name_string, str):
            raise TypeError("The attribute 'name' must be of type String")
        if not name_string:
            raise ValueError("The attribute 'name' must not be an empty String")

        self._name = name_string

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, position_x_y):
        if not (isinstance(position_x_y, list) and all(isinstance(value, (int, float)) for value in position_x_y)):
            raise TypeError("The attribute 'position' must be of type list of floats")
        if not len(position_x_y) == 2:
            raise ValueError("The attribute 'position' must be a list with two elements, position_x and position_y")
        self._position = position_x_y

    @abstractmethod
    def get_position(self):
        raise NotImplementedError("The method 'get_position' from AbstractSocket must be implemented!")

    @abstractmethod
    def update_position(self):
        raise NotImplementedError("The method 'update_position' from AbstractSocket must be implemented!")

    @abstractmethod
    def update_name(self, new_name):
        raise NotImplementedError("The method 'update_name' from AbstractSocket must be implemented!")

    @abstractmethod
    def remove(self):
        raise NotImplementedError("The method 'remove' from AbstractSocket must be implemented!")

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

    def is_connected(self):
        """
            Return a boolean stating if the socket currently hosts a connector

            @return: True if the socket contains at least a connector, otherwise False
        """
        return bool(self.connectors)

    def remove_all_connectors(self):
        """
            Remove all the connectors associated to this socket
        """
        # Using a while loop because some connectors were not properly deleted with a for loop for an unknown reason
        while self.connectors:
            connector = self.connectors.pop(0)
            connector.remove()
