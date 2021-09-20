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

from collections import OrderedDict
from grip_api.task_editor_graphics.terminal_socket import TerminalGraphicsSocket
from grip_api.utils.common_dialog_boxes import warning_message


class TerminalSocket(object):

    """
        Object gathering the logic and graphical representation of a socket
    """

    def __init__(self, container, socket_name, index, multi_connections=True):
        """
            Initialize the object

            @param container: Container object in which the terminal stocket will be added to
            @param socket_name: Name of the outcome the socket represents
            @param index: Index of the socket
            @param multi_connections: Indicate whether the socket can host several connectors. Default to True
        """
        # Set the id of the object
        self.id = id(self)
        # Store the Container the socket is added to
        self.container = container
        # Name of the outcome corresponding to the socket
        self.name = socket_name
        # Store whether this terminal socket is the one marking the beginning of the state machine or not
        self.is_starting = self.name == "Start"
        self.index = index
        # Set the multi connected attribute
        self.is_multi_connected = multi_connections
        # Create and store the graphical socket to be displayed
        self.graphics_socket = TerminalGraphicsSocket(self)
        # Get all the connectors connected to this terminal socket
        self.connectors = list()
        # Allows to recognize terminal sockets from others when releasing edges
        self.is_terminal = True
        # By default a terminal socket can't be deleted from the container
        self.is_deletable = False

    def set_position(self, x, y):
        """
            Set the position of the object in the scene

            @param x: Float or integer, corresponding to the x coordinate in the scene coordinates
            @param y: Float or integer, corresponding to the y coordinate in the scene coordinates
        """
        self.graphics_socket.setPos(x, y)
        # Since we set the position of the object, the container is fully initialized
        self.container.is_complete = True

    def get_position(self):
        """
            Return a list of two elements corresponding to the position of this widget in the scene

            @return: List of two floats (x, y), which are the coordinates of the widget in the scene coordinates
        """
        position = self.graphics_socket.pos()
        return [position.x(), position.y()]

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
        y = view_scene_size.y()
        # If it is the starting socket, then add the total height of the object downward so we can read the title
        if self.is_starting:
            y += total_height
        # If not, add the height of the view_scene and add the total height upward so we can read the title
        else:
            y += view_scene_size.height() - total_height
        # Compute the spacing between the sockets
        width = view_scene_size.width()
        total_number_of_spaces = num_sockets - 1
        socket_spacing = width / (num_sockets * 3.)
        x = self.index * socket_spacing - (total_number_of_spaces) / 2. * socket_spacing
        # Set the computed coordinates
        self.set_position(x, y)

    def add_connector(self, connector):
        """
            Add a provided connector to this terminal socket

            @param connector: Connector object to be added to the attribute connectors
        """
        self.connectors.append(connector)

    def remove_connector(self, connector):
        """
            Remove a provided connector from this terminal socket

            @param connector: Connector object to be removed from the attribute connectors
        """
        # Make sure it is part of the socket to avoid any exception
        if connector in self.connectors:
            self.connectors.remove(connector)

    def remove_all_connectors(self):
        """
            Remove all the connectors associated to this terminal socket
        """
        # Using a while loop because some connectors were not properly deleted with a for loop for an unknown reason
        while self.connectors:
            connector = self.connectors.pop(0)
            connector.remove()

    def update_name(self, new_name, save_history=True):
        """
            Update the name of this object

            @param new_name: Name (string) to be given to the object
            @param save_history: Boolean stating if this change should be saved into the container's history
        """
        # If a terminal socket has already the required name, then displays a warning message and stop
        if new_name in map(lambda x: x.name, self.container.terminal_sockets):
            warning_message("Invalid name", "An outcome with the same name already exists!")
            return
        # If the name is valid, set it and replace the outcome in the container's attribute
        index_in_outcomes = self.container.outcomes.index(self.name)
        self.name = new_name
        self.container.outcomes[index_in_outcomes] = new_name
        # Update the name of the socket of the state-like representation of the container if possible
        if self.container.state_machine is not None:
            self.container.state_machine.output_sockets[self.index].set_name(new_name)
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

    def save(self):
        """
            Save the current properties of the object so it can be restored later on

            @return: Dictionary containing the id, the name and position of the object
        """
        # Get the current position of the terminal socket
        current_position = self.graphics_socket.pos()
        return OrderedDict([
            ("id", self.id),
            ("name", self.name),
            ("position_x", current_position.x()),
            ("position_y", current_position.y())
        ])

    def restore(self, properties, socket_mapping={}):
        """
            Restore the object to a previously saved configuration

            @param properties: Dictionary containing the id, the name and position of the terminal socket
            @param socket_mapping: Dictionary mapping the id of sockets to the actual objects
        """
        self.id = properties["id"]
        # If the name to restore is different than the current one, update it
        if self.name != properties["name"]:
            # Since we just restore the name, don't save it in the history
            self.update_name(properties["name"], save_history=False)
            # Make sure the change is updated on the screen
            self.graphics_socket.update_name()
        # Set the position of the socket
        self.set_position(properties["position_x"], properties["position_y"])
        socket_mapping[properties["id"]] = self
