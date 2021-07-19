# !/usr/bin/env python

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

from PyQt5.QtWidgets import QWidget, QHBoxLayout, QTabWidget
from grip_api.task_editor_widgets.task_editor_mdi import TaskEditorMDIArea
from grip_api.task_editor_widgets.side_displayers import StatesDisplayer, StateMachinesDisplayer


class TaskEditorArea(QWidget):

    """
        Widget containing the MDI area and state/state machine displayer required to design and execute tasks
    """

    def __init__(self, parent=None):
        """
            Initialise the different widgets

            @param parent: Parent of the widget
        """
        super(TaskEditorArea, self).__init__(parent=parent)
        # Set the object name to be able to look it up and restore it
        self.setObjectName("Task editor area")
        self.framework_gui = parent
        # Flag stating whether the widget can be saved or not
        self.can_be_saved = False
        self.init_ui()

    def init_ui(self):
        """
            Initialize the UI of the widget
        """
        layout = QHBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        # Add the Multi Document Interface area
        self.mdi_area = TaskEditorMDIArea(parent=self)
        # Connect the signal sent by the MDI area to update the can_be_saved attribute of the object
        self.mdi_area.canBeSaved.connect(self.update_savable)
        layout.addWidget(self.mdi_area)
        # Create a tab widget to display both states and state machines
        displayers = QTabWidget(self)
        # Move the tab to the bottom so it looks better
        displayers.setTabPosition(QTabWidget.South)
        # Add the widget showing available states and to import some new ones
        self.state_displayer = StatesDisplayer(self)
        displayers.addTab(self.state_displayer, "States")
        # Add the widget showing available state machines and to import some new ones
        self.state_machine_displayer = StateMachinesDisplayer(self)
        displayers.addTab(self.state_machine_displayer, "State machines")
        # By design, the sizeHint of the tab corresponds to the maximum size to optimize the editor area
        displayers.setMaximumWidth(displayers.sizeHint().width())
        layout.addWidget(displayers)
        self.setLayout(layout)

    def update_savable(self, can_be_saved):
        """
            Update the flag that specifies if this object is in a state that can be saved or not

            @param can_be_saved: Boolean specifying if the object can be saved (has been modified) or not
        """
        self.can_be_saved = can_be_saved

    def save_config(self, settings):
        """
            Store the state of this widget and its children into settings

            @param settings: QSettings object in which widgets' information are stored
        """
        settings.beginGroup(self.objectName())
        # Make sure not to have some residual configurations (e.g. removed state machine)
        settings.remove("")
        class_name = self.metaObject().className()
        settings.setValue("type", class_name)
        # For each subwindow part of the task editor, save its configuration
        for subwindow in self.mdi_area.subWindowList():
            subwindow.widget().save_config(settings)
        # Save the index of the current subwindow being active
        settings.setValue("current_subwindow", self.mdi_area.get_current_subwindow_index())
        settings.endGroup()
        # Update the flag
        self.can_be_saved = False

    def restore_config(self, settings):
        """
            Restore the widget's children from the configuration saved in settings

            @param settings: QSettings object that contains information of the widgets to restore
        """
        settings.beginGroup(self.objectName())
        # For each subwindow that has been previously stored
        for subwindow_index in range(len(settings.childGroups())):
            # Restore the corresponding GraphicalEditorWidget
            self.mdi_area.subWindowList()[subwindow_index].widget().restore_config(settings)
        # Activate the same subwindow as when the config was saved
        self.mdi_area.set_current_subwindow_from_index(settings.value("current_subwindow", type=int))
        settings.endGroup()
        # Update the flag
        self.can_be_saved = False
