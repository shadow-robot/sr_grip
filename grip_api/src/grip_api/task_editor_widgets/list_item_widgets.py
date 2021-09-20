#!/usr/bin/env python3

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

from PyQt5.QtGui import QFont, QCursor
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QVBoxLayout, QLabel, QFrame


class BoxItemContent(QFrame):

    """
        Widget containing the information about a state or state machine that is going to be displayed in a list
    """

    def __init__(self, name, item_description, is_state=True, parent=None):
        """
            Initialize the widget

            @param name: Name of the box (string)
            @param item_description: String describing what the item is for
            @param is_state: Boolean specifying whether the box displays a state or a state machine
            @param parent: Parent of the widget
        """
        super().__init__(parent=parent)
        # Set the name of the box (= widget)
        self.name = name
        self.is_state = is_state
        self.init_ui()
        # Set the content of the widget
        self.set_content(item_description)
        # Change the defulat cursor when hovering the widget
        self.setCursor(QCursor(Qt.OpenHandCursor))

    def init_ui(self):
        """
            Initialize the UI of the widget
        """
        self.layout = QVBoxLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(self.layout)
        # Change the shape and shadow of the widget so it looks like it is draggable
        self.setFrameShape(QFrame.WinPanel)
        self.setFrameShadow(QFrame.Raised)
        self.setLineWidth(2)
        self.setMidLineWidth(3)
        # Change background color
        style_sheet = "background-color: {}".format("rgb(235, 232, 229)" if self.is_state else "rgb(220, 217, 214)")
        self.setStyleSheet(style_sheet)

    def set_content(self, item_description):
        """
            Set the content of the widget

            @param item_description: Description (string) explaining what the item is for
        """
        # Define the main label of the widget
        main_label = QLabel(self.name)
        # Center it
        main_label.setAlignment(Qt.AlignCenter)
        # Make it bold
        font = QFont()
        font.setBold(True)
        main_label.setFont(font)
        # Make sure to get a consistent padding size
        main_label_height = main_label.fontMetrics().boundingRect(main_label.text()).height() + 5
        main_label.setMaximumHeight(main_label_height)
        # Add the widget to the layout
        self.layout.addWidget(main_label)
        # Now add the description
        element_description = QLabel(item_description)
        # Since it can be long, justify it
        element_description.setAlignment(Qt.AlignJustify)
        # Since the description can be longer than allocated space, make sure the box fits the parent size
        parent_width = self.parent().sizeHint().width()
        element_description.setMaximumWidth(parent_width)
        # Make the description multi line
        element_description.setWordWrap(True)
        # Adjust the size of the widget with the new options
        element_description.adjustSize()
        # Get the multi line height
        height = element_description.size().height()
        # Set it to the description
        element_description.setMaximumHeight(height)
        # Add a margin to make it look nicer
        element_description.setMargin(5)
        self.layout.addWidget(element_description)
        # Update the size
        self.adjustSize()
