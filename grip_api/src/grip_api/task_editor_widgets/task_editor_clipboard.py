#!/usr/bin/env python

# Copyright 2021 Shadow Robot Company Ltd.
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
from connector import Connector
from state_machine import StateMachine


class Clipboard(object):

    """
        Object that stores some items selected in a container
    """

    def __init__(self):
        """
            Initialize the class
        """
        # Content of the clipboard is set to None at first since nothing has been copied yet
        self.content = None

    def copy_selected_items(self, container, remove_copied=False):
        """
            Copy the elements selected in a given container and potentially remove then afterwards

            @param container: Container object from which the items need to be copied from
            @param remove_copied: Boolean stating if the copied elements should be removed. Default is False
        """
        # Lists that will contain all the different objects that are selected by the user
        selected_states, selected_state_machines, selected_connectors = list(), list(), list()
        # When copying a state machines we also want to copy its content, i.e. its definition
        selected_containers = list()
        # To get the mapping between old and new socket ids
        for selected_item in container.graphics_container.selectedItems():
            # Only consider items that can be loaded from the robot config
            if hasattr(selected_item, "state") and selected_item.state.get_opacity() == 1.:
                selected_states.append(selected_item.state.save())
            if hasattr(selected_item, "state_machine") and selected_item.state_machine.get_opacity() == 1.:
                selected_state_machines.append(selected_item.state_machine.save())
                # Get the definition of the state machine
                selected_containers.append(selected_item.state_machine.def_container.save())
            elif hasattr(selected_item, "connector"):
                selected_connectors.append(selected_item.connector.save())

        non_valid_connectors = list()
        # Get the ID of all the sockets that were selected
        selected_sockets = map(lambda x: x["input_socket"] + x["output_sockets"],
                               selected_states + selected_state_machines)
        # Make several lists into a single one
        selected_sockets = [item for sublist in selected_sockets for item in sublist]
        # Remove all the connectors that are not connected on both ends
        for connector in selected_connectors:
            if connector["start"] not in selected_sockets or connector["end"] not in selected_sockets:
                non_valid_connectors.append(connector)
        for connector in non_valid_connectors:
            selected_connectors.remove(connector)

        # Store the valid content
        self.content = OrderedDict([('states', selected_states), ('state_machines', selected_state_machines),
                                    ('connectors', selected_connectors), ('containers', selected_containers)])
        # If required, remove the selected items
        if remove_copied:
            container.get_view().delete_selected()

    def paste(self, container):
        """
            Paste the items that were previously copied to the clipboard to a target container

            @param container: Target Container object to which the items need to be copied to
        """
        # If nothing is in the clipboard then quits
        if self.content is None:
            return

        # Get the last valid posiiton of the cursor in the view
        mouse_scene_pos = container.get_view().latest_valid_cursor_position
        # If the user has not clicked or moved inside the target container, do nothing
        if mouse_scene_pos is None:
            return

        # Calculate selected objects bbox and center
        min_x, max_x, min_y, max_y = 1e10, -1e10, 1e10, -1e10
        for node_data in self.content['states'] + self.content["state_machines"]:
            x, y = node_data['pos_x'], node_data['pos_y']
            min_x, min_y = min(min_x, x), min(min_y, y)
            max_x, max_y = max(max_x, x), max(max_y, y)

        bbox_center_x = (min_x + max_x) / 2
        bbox_center_y = (min_y + max_y) / 2

        # Calculate the offset of the nodes to be pasted
        offset_x = mouse_scene_pos.x() - bbox_center_x
        offset_y = mouse_scene_pos.y() - bbox_center_y
        # Get a copy of all the elements that can be pasted
        states_data = [state_dict.copy() for state_dict in self.content["states"]]
        state_machines_data = [state_machine_dict.copy() for state_machine_dict in self.content["state_machines"]]
        containers_data = [container_dict.copy() for container_dict in self.content["containers"]]
        connectors_data = [connector_dict.copy() for connector_dict in self.content["connectors"]]

        # Initialize the dictionary that records the mapping between the id of the sockets and the actual objects
        socket_mapping = {}
        # Dictionary that will contain the mapping between the old and new IDs of the sockets
        new_sockets = {}
        # Restore the states, but make sure to change the ID of all the sockets since we want "new" sockets
        container.restore_states(states_data, socket_mapping, offset=(offset_x, offset_y), new_sockets=new_sockets)

        # Get the index of the container in which we want to paste the state machines
        container_index = container.editor_widget.parent().mdiArea().get_current_subwindow_index()
        # Create subwindows for each state machine added
        for state_machine_data, container_data in zip(state_machines_data, containers_data):
            # Make sure the name is unique within and between containers
            state_machine_name = container.get_unique_name(state_machine_data["name"])
            state_machine_name = container.editor_widget.parent().mdiArea().get_unique_name(state_machine_name)
            # Create a new subwindow
            container.editor_widget.parent().mdiArea().add_subwindow(state_machine_name, state_machine_data["type"])
            # Get the container in which the new state mahcine will be defined
            secondary_container = container.editor_widget.parent().mdiArea().focused_subwindow.widget().container
            # Create a state like representation to be displayed in the current widget
            dropped_state_machine = StateMachine(container, secondary_container)
            # Restore the state machine
            dropped_state_machine.restore(state_machine_data, socket_mapping)
            # Apply the translation
            dropped_state_machine.translate(offset_x, offset_y)
            # Restore the definition of the state machine
            secondary_container.restore(container_data)
        # Make sure the user only sees the original container (not switching to another subwindow)
        container.editor_widget.parent().mdiArea().set_current_subwindow_from_index(container_index)

        # Add the connectors
        if connectors_data:
            for connector_data in connectors_data:
                # Get the ID of the connectors
                start, end = connector_data["start"], connector_data["end"]
                # If the socket ID has been remapped
                if start in new_sockets and end in new_sockets:
                    # Get the new ID of the socket
                    mapped_start, mapped_end = new_sockets[start], new_sockets[end]
                    # Create the connector from the new socket numbers
                    Connector(container, socket_mapping[mapped_start], socket_mapping[mapped_end])
        # When the elements are pasted, store the new content of the container
        container.history.store_current_history(set_modified=True)
