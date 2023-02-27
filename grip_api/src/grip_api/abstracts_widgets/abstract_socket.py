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

from abc import ABC, abstractmethod


class AbstractSocket(ABC):

    """
        Abstract Socket class with set of variables and methods common to Socket and TerminalSocket
    """

    def __init__(self, socket_name, index=0, multi_connections=True):
        """
            Abstract widget initialisation

            @param socket_name: Name of the socket (string)
            @param index: Integer, stating the index of the socket on this side of the node
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
        self.is_terminal = False

    @abstractmethod
    def get_position(self):
        pass

    @abstractmethod
    def update_position(self):
        pass

    @abstractmethod
    def update_name(self):
        pass

    @abstractmethod
    def remove(self):
        pass

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
