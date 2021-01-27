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
from collections import OrderedDict
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QPushButton, QFileDialog
from PyQt5.QtCore import pyqtSignal
from list_widgets import StateListWidget, StateMachineListWidget
from grip_core.utils.common_paths import CATKIN_WS
from grip_api.utils.common_dialog_boxes import error_message
from grip_core.utils.file_parsers import fill_available_states, fill_available_state_machines
from grip_api.utils.files_specifics import EDITOR_TO_DESCRIPTION


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
        # Going to store whether at least one of the hardware part is configured through MoveIt!
        self.commander_configs = dict()
        # Going to store the name of all the components integrated to the framework
        self.external_sources = dict()
        # Get reference to the integration area to access the signals sent by the different editors
        integration_area = self.parent().framework_gui.robot_integration_area
        arm_config = integration_area.arm_config_widget
        hand_config = integration_area.hand_config_widget
        # Enable/disable the commander states
        arm_config.moveit_planners_config.contentUpdated.connect(self.update_commander_states)
        hand_config.moveit_planners_config.contentUpdated.connect(self.update_commander_states)
        # Enable/disable states from what has been integrated to the framework
        arm_config.external_controller.contentUpdated.connect(self.update_generated_states)
        hand_config.external_controller.contentUpdated.connect(self.update_generated_states)
        arm_config.kinematic_libraries_config.contentUpdated.connect(self.update_generated_states)
        hand_config.kinematic_libraries_config.contentUpdated.connect(self.update_generated_states)
        arm_config.external_motion_planner.contentUpdated.connect(self.update_generated_states)
        hand_config.external_motion_planner.contentUpdated.connect(self.update_generated_states)
        # Editor in which high level methods are integrated
        integration_area.settings_config_widget.external_methods.contentUpdated.connect(self.update_generated_states)
        # Editor in which sensors are integrated
        integration_area.settings_config_widget.sensor_configs.contentUpdated.connect(self.update_generated_states)

    def update_commander_states(self):
        """
            Update the states used for robots configured through MoveIt!
        """
        # If a MoveIt! motion planner has been properly configured and is enabled (i.e. config package provided),
        # store whether it comes from the arm or hand.
        self.commander_configs[self.sender()] = self.sender().valid_input and self.sender().isEnabled()
        # If any is good, then add the states
        if any(self.commander_configs.values()):
            self.list_widget.add_items("Commander")
        # Otherwise remove them
        else:
            self.list_widget.remove_items("Commander")

    def update_generated_states(self):
        """
            Update the generated states, created from the components integrated in GRIP's editors
        """
        # If the editor's valid_input is None (= file closed), then remove it from the sources (if part of it)
        if self.sender().valid_input is None:
            if self.sender() in self.external_sources:
                del self.external_sources[self.sender()]
        # Otherwise get the name of the components to integrate
        else:
            self.external_sources[self.sender()] = self.sender().valid_input.keys()
        # Create a temporary dictionary that contains all the states to display
        up_to_date_integrated = OrderedDict()
        for sender, components in self.external_sources.items():
            for component_name in components:
                # Set the same format as the other states
                up_to_date_integrated[component_name] = {"description": EDITOR_TO_DESCRIPTION[sender.objectName()]}
        # Update the generated items
        self.list_widget.update_generated_items(up_to_date_integrated)

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
