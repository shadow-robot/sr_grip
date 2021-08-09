# !/usr/bin/env python

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

from PyQt5.QtWidgets import QListWidget, QAbstractItemView, QListWidgetItem
from PyQt5.QtGui import QPixmap, QDrag
from PyQt5.QtCore import Qt, QByteArray, QDataStream, QMimeData, QIODevice, QPoint
from collections import OrderedDict
from list_item_widgets import BoxItemContent
from grip_core.utils.file_parsers import AVAILABLE_STATES, AVAILABLE_STATEMACHINES
from grip_api.utils.files_specifics import LISTITEM_MIMETYPE
from grip_core.utils.common_paths import STATE_ICON, STATE_MACHINE_ICON


class CommonDraggableListWidget(QListWidget):

    """
        Common widget allowing to show a list of items that can be dragged
    """

    def __init__(self, parent=None):
        """
            Initialize the widget

            @param parent: Parent of the widget
        """
        super(CommonDraggableListWidget, self).__init__(parent)
        # By default each element of the list is not associated to any icon
        self.icon = QPixmap(".")
        self.init_ui()
        # Get a track of the name of the items displayed in the list
        self.item_names = list()

    def init_ui(self):
        """
            Initialize the UI of the widget
        """
        self.setSelectionMode(QAbstractItemView.SingleSelection)
        # Make it draggable
        self.setDragEnabled(True)

    def add_item(self, item_name, item_description, is_state=True):
        """
            Add an item to the list widget

            @param item_name: Name of the item (string)
            @param item_description: Description of the item (string)
            @param is_state: Boolean specifying whether the item is a state or not. Default is True
        """
        # If an item with the same name already exists, then pass
        if item_name in self.item_names:
            return
        # Create a new QListWidgetItem
        list_item = QListWidgetItem()
        # Create a widget properly formatting the content of the item to display
        widget_item = BoxItemContent(item_name, item_description, is_state=is_state, parent=self)
        # Adjust the size of the list widget item
        list_item.setSizeHint(widget_item.size())
        list_item.setData(Qt.UserRole, self.icon)
        # Add the item to the list
        self.addItem(list_item)
        # Store the name of the item
        self.item_names.append(item_name)
        # Set our widget to the list widget item
        self.setItemWidget(list_item, widget_item)

    def startDrag(self, *args, **kwargs):
        """
            Function triggered when the object is being dragged. It embeds all required information to display the
            corresponding state
        """
        # Get the item being dragged
        item = self.currentItem()
        # Extract its content (=widget)
        widget = self.itemWidget(item)
        # Extract the info re. whether it is a state or not
        is_state = widget.is_state
        # Store the type of the widget (ConcurrentStateMachine, Plan, etc.)
        item_type = widget.name
        # Store the icon corresponding to the class of box (state or state machine)
        pixmap = QPixmap(item.data(Qt.UserRole))
        # Create an object that will store the widget's extracted info
        item_data = QByteArray()
        # Wrap it in a stream so it can be read when dropped
        data_stream = QDataStream(item_data, QIODevice.WriteOnly)
        # Set the different attributes (the order matter)
        data_stream.writeBool(is_state)
        data_stream.writeQString(item_type)
        data_stream << pixmap
        # We need to create a Multipurpose Internet Mail Extension (MIME) data to be able to drag the object around
        mime_data = QMimeData()
        mime_data.setData(LISTITEM_MIMETYPE, item_data)
        # Create a QDrag for the object
        drag = QDrag(self)
        # Set the MIME data
        drag.setMimeData(mime_data)
        # Set the hot spot (where is the mouse centered), relative to top left corner
        drag.setHotSpot(QPoint(pixmap.width() / 2, pixmap.height() / 2))
        # Set the pixmap (icon)
        drag.setPixmap(pixmap)
        # Execute the drag action
        drag.exec_(Qt.MoveAction)


class StateMachineListWidget(CommonDraggableListWidget):

    """
        List widget gathering all the state machines that can be used in the task editor
    """

    def __init__(self, parent=None):
        """
            Initialize the widget
        """
        super(StateMachineListWidget, self).__init__(parent=parent)
        # Set the icon of each item
        self.icon = QPixmap(STATE_MACHINE_ICON).scaledToHeight(32)
        self.add_items(AVAILABLE_STATEMACHINES)

    def add_items(self, items_to_fill):
        """
            Add the items to the list

            @param items_to_fill: Dictionary containing the elements to display
        """
        for item_name, item_parameters in items_to_fill.items():
            self.add_item(item_name, item_description=item_parameters["description"], is_state=False)

    def update_content(self):
        """
            Update the content of the list
        """
        self.clear()
        super(StateMachineListWidget, self).add_items(AVAILABLE_STATEMACHINES)


class StateListWidget(CommonDraggableListWidget):

    """
        List widget gathering all the states that can be used in the task editor
    """

    def __init__(self, parent=None):
        """
            Initialize the widget
        """
        super(StateListWidget, self).__init__(parent=parent)
        # Set the icon of all the items
        self.icon = QPixmap(STATE_ICON).scaledToHeight(32)
        # Create the attribute of the class that will store all the different kind of states to be displayed
        self.states_to_display = OrderedDict()
        # This widget can have different "kind" of states to display, but external and some provided states should
        # always be displayed
        self.add_items("Constant")

    def add_items(self, state_type):
        """
            Add all the items part of a "kind" of states

            @param state_type: Kind of state to be integrated. Must be either "Constant", "Commander" or "Generated"
        """
        # If state_type does not correspond to expected input then just quit
        if state_type not in ["Constant", "Commander", "Generated"]:
            return
        # If we want to add items of a category of states that is not part of the attribute, then get them
        if state_type == "Constant" and state_type not in self.states_to_display:
            self.extract_constant_states()
        elif state_type == "Commander" and state_type not in self.states_to_display:
            self.extract_commander_states()
        # Add items corresponding to the category of states input
        for state_name, state_info in self.states_to_display[state_type].items():
            self.add_item(state_name, state_info["description"])

    def remove_items(self, state_type):
        """
            Remove all the items part of "kind" of states

            @param state_type: Kind of state to be integrated. Must be either "Constant", "Commander" or "Generated".
        """
        # If state_type does not correspond to expected input then just quit
        if state_type not in ["Constant", "Commander", "Generated"]:
            return
        # If state_type is not already part of the attribute then just quit
        if state_type not in self.states_to_display:
            return
        # For each state
        for state_name in self.states_to_display[state_type].keys():
            # Remove the item from the widget
            self.takeItem(self.item_names.index(state_name))
            # And remove its name from the interal list
            self.item_names.remove(state_name)
        # Remove the category of states
        del self.states_to_display[state_type]

    def update_generated_items(self, new_items):
        """
            Update the display of generated states

            @param new_items: Dictionary containing the name and description of the states to display
        """
        # If we already have some, remove them to make sure we have the latest version of integrated components
        if "Generated" in self.states_to_display:
            self.remove_items("Generated")
        # Set and add the items
        self.states_to_display["Generated"] = new_items
        self.add_items("Generated")

    def extract_constant_states(self):
        """
            Store all the "constant" states, i.e. the one that should always be displayed, from the different sources
        """
        # Get all "constant" states, i.e. the one that should always be displayed
        constant_states = OrderedDict()
        for state_name, state_info in AVAILABLE_STATES.items():
            if state_name != "Commander":
                constant_states[state_name] = state_info
        # Store them into the class' attribute
        self.states_to_display["Constant"] = constant_states

    def extract_commander_states(self):
        """
            Store all the states that allows the user to operate the robot through MoveIt!
        """
        # Store the commander states
        self.states_to_display["Commander"] = AVAILABLE_STATES["Commander"]

    def update_content(self):
        """
            Update the states which correspond to the "Constant" part of the list
        """
        self.remove_items("Constant")
        self.add_items("Constant")
