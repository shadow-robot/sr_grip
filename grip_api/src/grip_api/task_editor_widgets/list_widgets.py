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
from list_item_widgets import BoxItemContent
from grip_core.utils.file_parsers import AVAILABLE_STATES, AVAILABLE_STATEMACHINES
from grip_api.utils.files_specifics import LISTITEM_MIMETYPE
from grip_core.utils.common_paths import STATE_ICON, STATE_MACHINE_ICON


class CommonDraggableListWidget(QListWidget):

    """
        Common widget allowing to show a list of items that can be dragged
    """

    def __init__(self, items, parent=None):
        """
            Initialize the widget and fill the list

            @param items: Dictionary containing the name of elements to integrate as keys and the description as value
            @param parent: Parent of the widget
        """
        super(CommonDraggableListWidget, self).__init__(parent)
        self.init_ui()
        self.add_items(items)

    def init_ui(self):
        """
            Initialize the UI of the widget
        """
        self.setSelectionMode(QAbstractItemView.SingleSelection)
        # Make it draggable
        self.setDragEnabled(True)

    def add_items(self, items_to_fill):
        """
            Add the items to the list

            @param items_to_fill: Dictionary containing the elements to integrate
        """
        for item_name, item_parameters in items_to_fill.items():
            # Function defined in each children
            self.add_item(item_name, item_description=item_parameters["description"])

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
        # Set the icon of each item
        self.icon = QPixmap(STATE_MACHINE_ICON).scaledToHeight(32)
        super(StateMachineListWidget, self).__init__(items=AVAILABLE_STATEMACHINES, parent=parent)

    def add_item(self, item_name, item_description):
        """
            Add an item to the list widget

            @param item_name: Name of the item (string)
            @param item_description: Description of the item (string)
        """
        # Create a new QListWidgetItem
        list_item = QListWidgetItem()
        # Create a widget properly formatting the content of the item to display
        widget_item = BoxItemContent(item_name, item_description, is_state=False, parent=self)
        # Adjust the size of the list widget item
        list_item.setSizeHint(widget_item.size())
        list_item.setData(Qt.UserRole, self.icon)
        # Add the item to the list
        self.addItem(list_item)
        # Set our widget to the list  widget item
        self.setItemWidget(list_item, widget_item)

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
        # Set the icon of all the items
        self.icon = QPixmap(STATE_ICON).scaledToHeight(32)
        super(StateListWidget, self).__init__(items=AVAILABLE_STATES, parent=parent)

    def add_item(self, item_name, item_description):
        """
            Add an item to the list widget

            @param item_name: Name of the item (string)
            @param item_description: Description of the item (string)
        """
        # Create a new QListWidgetItem
        list_item = QListWidgetItem()
        # Create a widget properly formatting the content of the item to display
        widget_item = BoxItemContent(item_name, item_description, parent=self)
        # Adjust the size of the list widget item
        list_item.setSizeHint(widget_item.size())
        list_item.setData(Qt.UserRole, self.icon)
        # Add the item to the list
        self.addItem(list_item)
        # Set our widget to the list widget item
        self.setItemWidget(list_item, widget_item)

    def update_content(self):
        """
            Update the content of the list
        """
        self.clear()
        super(StateListWidget, self).add_items(AVAILABLE_STATES)
