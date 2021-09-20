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
from grip_core.utils.file_parsers import AVAILABLE_STATES
from grip_api.utils.common_dialog_boxes import error_message
from grip_api.utils.common_checks import is_state_source_valid


class TaskEditorArea(QWidget):

    """
        Widget containing the MDI area and state/state machine displayer required to design and execute tasks
    """

    def __init__(self, parent=None):
        """
            Initialise the different widgets

            @param parent: Parent of the widget
        """
        super().__init__(parent=parent)
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

    def reset(self, task_name="root"):
        """
            Reset the widget, i.e. reset the available states and clear the editors as when opened for the first time

            @param task_name: Name of the main window (i.e. task) to be created after closing all the others
        """
        # Update the state sources of the parent widget to only be the internal ones
        self.framework_gui.state_sources = self.framework_gui.state_sources[:1]
        # Clear the available states
        AVAILABLE_STATES.clear()
        # Reload the states
        self.state_displayer.load_states(self.framework_gui.state_sources)
        # Reset the editors
        self.mdi_area.reset(task_name)

    def check_external_sources(self, external_sources):
        """
            Make sure the sources for new states saved in the settings are still valid. If that is the case, load them.

            @param external_sources: List of path poitning to directories with the states to import
            @return: None if the sources could not have been read properly from the settings. True otherwise
        """
        # Make sure the argument external_sources is really a list
        if not isinstance(external_sources, list):
            error_message("Error while restoring task configuration",
                          "The source of the states in the task configuration file must be a list!", parent=self)
            return None
        # Flag to update the state_displayer, by default is False
        update_state_displayer = False
        # For each path, check if it is valid or not
        for path in external_sources:
            is_valid = is_state_source_valid(path, self)
            # If at leas one is valid, update the flag and add it to the list of state sources
            if is_valid:
                update_state_displayer = True
                self.framework_gui.state_sources.append(path)
        # If need be, update the content of the state displayer
        if update_state_displayer:
            self.state_displayer.load_states(self.framework_gui.state_sources[1:])
        return True

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
        # If there are any external sources, save them inside the settings, otherwise remove the key if it exists
        if len(self.framework_gui.state_sources) > 1:
            settings.setValue("state_sources", self.framework_gui.state_sources[1:])
        else:
            settings.remove("state_sources")

    def restore_config(self, settings):
        """
            Restore the widget's children from the configuration saved in settings

            @param settings: QSettings object that contains information of the widgets to restore
        """
        # If any extra source of states must be loaded
        if settings.contains("state_sources"):
            is_valid = self.check_external_sources(settings.value("state_sources"))
            # If an issue occured while reading the configuration file stop the restoration
            if is_valid is None:
                return
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
