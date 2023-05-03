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

from typing import List, Union
from grip_api.abstract_widgets.abstract_socket import AbstractSocket
from grip_api.task_editor_graphics.terminal_socket import TerminalGraphicsSocket
from grip_api.utils.formatted_print import format_raise_string
from grip_api.utils.common_dialog_boxes import warning_message


class TerminalSocket(AbstractSocket):
    """
        Object gathering the logic and graphical representation of a TerminalSocket
    """

    def __init__(self, container, socket_name, index, multi_connections=True, is_deletable=False):
        """
            Initialize the object

            @param container: Container object in which the terminal stocket will be added to
            @param socket_name: Name of the outcome the socket represents
            @param index: Index of the socket
            @param multi_connections: Indicate whether the socket can host several connectors. Default to True
        """
        super().__init__(socket_name, index, multi_connections, is_terminal=True, is_starting=socket_name == "Start")
        # Store the Container the socket is added to
        self.container = container
        # By default a terminal socket can't be deleted from the container
        self.is_deletable = is_deletable
        # Create and store the graphical socket to be displayed
        self.graphics_socket = TerminalGraphicsSocket(self)

    @AbstractSocket.graphics_socket.setter
    def graphics_socket(self, graphical_socket: TerminalGraphicsSocket) -> None:
        """
            Set the graphical representation of the socket

            @param graphical_socket: Instance of the TerminalGraphicsSocket class
        """
        if not isinstance(graphical_socket, TerminalGraphicsSocket):
            raise TypeError(format_raise_string("The property 'graphics_socket' must be a TerminalGraphicsSocket"))
        self._graphics_socket = graphical_socket

    @property
    def position(self) -> List[float]:
        """
            Return the position of the socket in the graphical view

            @return: Current position of the socket in the following format [x, y]
        """
        graphical_position = self._graphics_socket.pos()
        return [graphical_position.x(), graphical_position.y()]

    @position.setter
    def position(self, position_x_y: List[Union[int, float]]) -> None:
        """
            Set the position of the object in the graphical view

            @param position_x: x coordinate in the scene coordinates
            @param position_y: y coordinate in the scene coordinates        
        """
        # todo: ADDc check about type
        self.graphics_socket.setPos(*position_x_y)
        # Since we set the position of the object, the container is fully initialized
        self.container.is_complete = True

    @property
    def container(self):
        return self._container

    @container.setter
    def container(self, container_object):
        if hasattr(container_object, "graphics_container"):
            self._container = container_object
        else:
            raise TypeError("The attribute 'container' must be a 'Container' object")

    @property
    def is_starting(self):
        return self._is_starting

    @is_starting.setter
    def is_starting(self, value):
        if isinstance(value, bool):
            self._is_starting = value
        else:
            raise TypeError("The attribute 'is_starting' must be a boolean")

    @property
    def is_deletable(self):
        return self._is_deletable

    @is_deletable.setter
    def is_deletable(self, value):
        if isinstance(value, bool):
            self._is_deletable = value
        else:
            raise TypeError("The attribute 'is_deletable' must be a boolean")

    def update_name(self, new_name, save_history=True):  # pylint: disable=W0221
        """
            Update the name of this object

            @param new_name: Name (string) to be given to the object
            @param save_history: Boolean stating if this change should be saved into the container's history
        """
        # If a terminal socket has already the required name, then displays a warning message and stop
        if new_name in list(map(lambda x: x.name, self.container.terminal_sockets)):
            warning_message("Invalid name", "An outcome with the same name already exists!")
            return
        # If the name is valid, set it and replace the outcome in the container's attribute
        index_in_outcomes = self.container.outcomes.index(self.name)
        self.name = new_name
        self.container.outcomes[index_in_outcomes] = new_name
        # Update the name of the socket of the state-like representation of the container if possible
        if self.container.state_machine is not None:
            self.container.state_machine.output_sockets[self.index].update_name(new_name)
        if save_history:
            self.container.history.store_current_history()

    def remove(self):
        """
            Remove the terminal socket from its corresponding container
        """
        # Remove first all connectors connected to it
        self.remove_all_connectors()
        # Remove the terminal socket from the container
        self.container.remove_terminal_socket(self)
        # Make sure the graphical representation is set to None
        self.graphics_socket = None

    def set_initial_position(self):
        """
            Set the initial position of the socket, based on the area of the screen taken by the view
        """
        # Number of sockets (if it's the starting socket, then it is alone)
        num_sockets = 1 if self.is_starting else len(self.container.outcomes)
        # Get the view attached to the editor
        view = self.container.get_view()
        # Get the area of the scene currently displayed (in scene coordinates)
        view_scene_size = view.mapToScene(view.viewport().rect()).boundingRect()
        # Get the total height of the socket (socket + title)
        total_height = self.graphics_socket.get_total_height()
        # Set the y to the top of the screen
        position_y = view_scene_size.y()
        # If it is the starting socket, then add the total height of the object downward so we can read the title
        if self.is_starting:
            position_y += total_height
        # If not, add the height of the view_scene and add the total height upward so we can read the title
        else:
            position_y += view_scene_size.height() - total_height
        # Compute the spacing between the sockets
        width = view_scene_size.width()
        total_number_of_spaces = num_sockets - 1
        socket_spacing = width / (num_sockets * 3.)
        position_x = self.index * socket_spacing - (total_number_of_spaces) / 2. * socket_spacing
        # Set the computed coordinates
        self.update_position(position_x, position_y)

    def save(self):
        """
            Save the current properties of the object so it can be restored later on

            @return: Dictionary containing the id, the name and position of the object
        """
        # Get the current position of the terminal socket
        current_position = self.graphics_socket.pos()
        return dict([
            ("id", self.socket_id),
            ("name", self.name),
            ("position_x", current_position.x()),
            ("position_y", current_position.y())
        ])

    def restore(self, properties, socket_mapping):
        """
            Restore the object to a previously saved configuration

            @param properties: Dictionary containing the id, the name and position of the terminal socket
            @param socket_mapping: Dictionary mapping the id of sockets to the actual objects
        """
        self.socket_id = properties["id"]
        # If the name to restore is different than the current one, update it
        if self.name != properties["name"]:
            # Since we just restore the name, don't save it in the history
            self.update_name(properties["name"], save_history=False)
            # Make sure the change is updated on the screen
            self.graphics_socket.update_name()
        # Set the position of the socket
        self.update_position(properties["position_x"], properties["position_y"])
        socket_mapping[properties["id"]] = self
