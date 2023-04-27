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
from typing import List
from grip_api.utils.formatted_print import format_raise_string
from grip_api.abstract_graphics.abstract_socket import AbstractGraphicsSocket


class AbstractSocket(ABC):

    """
        Abstract socket class with set of variables and methods common to StateSocket and TerminalSocket
    """

    def __init__(self, socket_name: str, index: int = 0, multi_connections: bool = True,
                 is_terminal: bool = False, is_starting: bool = False) -> None:
        """
            Abstract widget initialisation

            @param socket_name: Name of the socket
            @param index: Index of the socket
            @param multi_connections: If the socket accepts multiple connections or not
            @param is_terminal: If the socket is a TerminalSocket or not
            @param is_starting: If the socket is a starting socket or not
        """
        # Store the id of the object
        self.socket_id = id(self)
        # Store all the information
        self.name = socket_name
        self.index = index
        self.is_multi_connected = multi_connections
        # List of all the connectors set to this socket
        self.connectors = []
        # Allows for recognizing terminal sockets from others when releasing edges
        self.is_terminal = is_terminal
        # Indicates if the socket marks the beginning of the object it is attached to.
        # For a StateSocket, if set to True, then the object corresponds to an input socket
        self.is_starting = is_starting
        # Initialise the graphics socket to be nothing at first
        self._graphics_socket = None

    @property
    def socket_id(self) -> int:
        """
            Return the id of the socket

            @return: ID of the object
        """
        return self._socket_id

    @socket_id.setter
    def socket_id(self, socket_id_: int) -> None:
        """
            Set the ID of the object

            @param socket_id_: ID to set to the socket
        """
        if not isinstance(socket_id_, int):
            raise TypeError(format_raise_string("The attribute 'socket_id' must be an integer"))
        self._socket_id = socket_id_

    @property
    def name(self) -> str:
        """
            Return the name of the socket

            @return: Name of the socket
        """
        return self._name

    @name.setter
    def name(self, name_string: str) -> None:
        """
            Set the name of the socket

            @param name_string: New name to be given to the socket
        """
        if not isinstance(name_string, str) or not name_string:
            raise TypeError(format_raise_string("The attribute 'name' must be a non-empty string"))

        self._name = name_string

    @property
    @abstractmethod
    def position(self) -> List[float]:
        """
            Return the position of the socket in the graphical view

            @return: Current position of the socket in the following format [x, y]
        """
        raise NotImplementedError(format_raise_string("Property 'position' from AbstractSocket must be implemented!"))

    @property
    def graphics_socket(self) -> AbstractGraphicsSocket:
        """
            Return the graphical representation of the socket

            @return: Object used to graphically represent the object in the graphical view
        """
        return self._graphics_socket

    @graphics_socket.setter
    @abstractmethod
    def graphics_socket(self, graphical_socket: AbstractGraphicsSocket) -> None:
        """
            Set the graphical representation of the socket

            @param graphical_socket: Instance of a class that inherits from AbstractGraphicsSocket
        """
        raise NotImplementedError(format_raise_string("The setter of 'graphics_socket' must be implemented!"))

    @abstractmethod
    def update_name(self, new_name: str) -> None:
        """
            Update the name of the socket

            @param new_name: New name to be given to the socket
        """
        raise NotImplementedError("The method 'update_name' from AbstractSocket must be implemented!")

    @abstractmethod
    def remove(self) -> None:
        """
            Remove the socket from its container
        """
        raise NotImplementedError("The method 'remove' from AbstractSocket must be implemented!")

    def update_position(self) -> None:
        """
            Method updating the position of the object in the graphical view
        """
        self._graphics_socket.setPos(*self.position)

    def add_connector(self, connector: 'Connector') -> None:
        """
            Add a provided connector to this socket

            @param connector: Object, instanciated from the Connector class to be added to the attribute connectors
        """
        self.connectors.append(connector)

    def remove_connector(self, connector: 'Connector') -> None:
        """
            Remove a provided connector from this socket

            @param connector: Object, instanciated from the Connector class to be removed from the attribute connectors
        """
        # Make sure it is part of the socket to avoid any exception
        if connector in self.connectors:
            self.connectors.remove(connector)

    def is_connected(self) -> bool:
        """
            Return a boolean stating if the socket currently hosts a connector

            @return: True if the socket contains at least a connector, otherwise False
        """
        return bool(self.connectors)

    def remove_all_connectors(self) -> None:
        """
            Remove all the connectors associated to this socket
        """
        # Using a while loop because some connectors were not properly deleted with a for loop for an unknown reason
        while self.connectors:
            connector = self.connectors.pop(0)
            connector.remove()
