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

import os
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QPushButton, QFileDialog
from PyQt5.QtCore import pyqtSignal
from list_widgets import StateListWidget, StateMachineListWidget
from grip_core.utils.common_paths import CATKIN_WS
from grip_api.utils.common_dialog_boxes import error_message
from grip_core.utils.file_parsers import fill_available_states, fill_available_state_machines


class CommonSideDisplayer(QWidget):

    """
        Widget allowing the user to see what elements are available in the task editor and to potentially add some
    """

    def __init__(self, list_widget, push_button_text, parent=None):
        """
            Initialize the class

            @param list_widget: QListWidget containing what to display
            @param push_button_text: Text that will be set to a push button allowing to add new elements
            @param parent: Parent of the widget
        """
        super(CommonSideDisplayer, self).__init__(parent=parent)
        self.init_ui()
        self.list_widget = list_widget
        self.add_widgets(push_button_text)

    def init_ui(self):
        """
            Initialize the UI
        """
        self.layout = QVBoxLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(self.layout)

    def add_widgets(self, push_button_text):
        """
            Add the widgets to the layout

            @param push_button_text: Text set to a push button allowing to add new elements
        """
        # Create and add the push button
        push_button = QPushButton(push_button_text)
        push_button.clicked.connect(self.on_click)
        self.layout.addWidget(push_button)
        # Make sure the widget has the correct dimensions
        self.list_widget.adjustSize()
        # Make sure the widget gets the proper width
        self.setMinimumWidth(self.list_widget.size().width())
        self.layout.addWidget(self.list_widget)

    def on_click(self):
        """
            Function that will carry out the import procedure
        """
        raise NotImplementedError()


class StateMachinesDisplayer(CommonSideDisplayer):
    """
        Widget displaying the available state machines that can used to create nested behaviours
    """
    # Signal stating that a new source for the state machines has been added
    stateMachineSourceAdded = pyqtSignal(str)

    def __init__(self, parent=None):
        """
            Initialize the class

            @param parent: Parent of the widget
        """
        super(StateMachinesDisplayer, self).__init__(StateMachineListWidget(), "Import templates", parent)

    def on_click(self):
        """
            Method allowing to add new state machines from external folder
        """
        returned_path = QFileDialog.getExistingDirectory(self, "Select the folder containing state machine templates",
                                                         options=QFileDialog.DontUseNativeDialog, directory=CATKIN_WS)
        if not returned_path:
            return
        # Make sure the directory contains at least one correct file
        if not any(any(y.endswith(".template") for y in x[-1]) for x in os.walk(returned_path)):
            error_message("Error", "No template files have been found in the provided directory", parent=self)
            return
        # Add the new state machines to the common dictionary
        fill_available_state_machines([returned_path])
        # Update the list widget
        self.list_widget.update_content()
        # Trigger the signal
        self.stateMachineSourceAdded.emit(returned_path)


class StatesDisplayer(CommonSideDisplayer):

    """
        Widget displaying the available states that can be used to create state machines
    """
    # Signal stating that a new source for the states has been added
    stateSourceAdded = pyqtSignal(str)

    def __init__(self, parent=None):
        """
            Initialize the class

            @param parent: Parent of the widget
        """
        super(StatesDisplayer, self).__init__(StateListWidget(), "Import states", parent)

    def on_click(self):
        """
            Method adding new states from external folder
        """
        returned_path = QFileDialog.getExistingDirectory(self, "Select the folder containing the states",
                                                         options=QFileDialog.DontUseNativeDialog, directory=CATKIN_WS)
        if not returned_path:
            return
        # Make sure the directory contains at least one correct file
        if not any(any(y.endswith(".py") for y in x[-1]) for x in os.walk(returned_path)):
            error_message("Error", "No template files have been found in the provided directory", parent=self)
            return
        # Add the new states to the common dictionary
        fill_available_states([returned_path])
        # Update the list widget
        self.list_widget.update_content()
        # Trigger the signal
        self.stateSourceAdded.emit(returned_path)
