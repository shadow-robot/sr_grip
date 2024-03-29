#!/usr/bin/env python

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

from PyQt5.QtWidgets import QGridLayout, QWidget, QInputDialog, QLineEdit, QMenu
from PyQt5.QtCore import Qt, QDataStream, QIODevice, pyqtSignal
from grip_api.task_editor_graphics.view import TaskEditorView
from container import Container
from state_machine import StateMachine
from grip_api.utils.files_specifics import LISTITEM_MIMETYPE
from grip_core.state_machine_generator.state_machine_generator import generate_state_machines
from grip_core.utils.common_paths import GENERATED_STATE_MACHINE_FOLDER, BASE_STATE_MACHINE_FOLDER
from grip_api.utils.common_dialog_boxes import warning_message
from state import State
import os
import subprocess


class GraphicalEditorWidget(QWidget):

    """
        Widget gathering the high level logic and event handler allowing the user to edit state machines
    """
    # Signal stating when the content of the editor widget has been modified
    hasBeenModified = pyqtSignal(bool)
    # Maximum number of outcomes for the container
    MAX_NUMBER_OUTCOMES = 10

    def __init__(self, container_name, container_type, parent=None):
        """
            Initialize the widget

            @param container_name: Name given to both the state machine container and this widget
            @param container_type: Type of the state machine container to load
            @param parent: Parent of the widget
        """
        super(GraphicalEditorWidget, self).__init__(parent=parent)
        # Create the container associated to this widget
        self.container = Container(editor_widget=self, container_type=container_type)
        self.init_ui()
        # Initialize the context menu
        self.init_context_menu()
        self.set_name(container_name)
        # Process used to launch the state machine
        self.launch_process = None
        # Update the above attribute according to whether the robot is launched or not
        self.robot_integration_area = self.parent().parent().parent().framework_gui.robot_integration_area
        # The container can be executed only if a robot is running
        self.can_be_executed = self.robot_integration_area.launch_process is not None
        self.robot_integration_area.robotCanBeStopped.connect(self.update_execution)
        # Boolean specifying if the widget hosts the root of the task
        self.is_root = container_type == "base"

    def init_ui(self):
        """
            Initialize the UI and view of the widget
        """
        # Main layout
        self.layout = QGridLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(self.layout)
        # Create graphical view
        self.editor_view = TaskEditorView(self.container.graphics_container, self)
        # Create the terminal sockets (for each outcome + the starting socket)
        self.container.create_initial_terminal_sockets()
        # Link the drag and drop event that occurs in the view to methods defined here
        self.editor_view.add_drag_enter_listener(self.on_drag_enter)
        self.editor_view.add_drop_listener(self.on_drop)
        # Add the graphics view in the layout
        self.layout.addWidget(self.editor_view)
        # Make sure that if the window containing the widget is deleted, this widget is properly removed as well
        self.setAttribute(Qt.WA_DeleteOnClose)

    def update_execution(self, can_be_stopped):
        """
            Set can_be_executed to the value can_be_stopped

            @param can_be_stopped: Boolean stating whether the robot can be stopped (i.e. is it launched)
        """
        self.can_be_executed = can_be_stopped

    def init_context_menu(self):
        """
            Initialize the proper context menu according to this widget's container
        """
        self.context_menu = QMenu(self)
        self.execute_action = self.context_menu.addAction("Execute")
        if self.container.type != "base":
            self.execute_action.setVisible(False)
        self.rename_action = self.context_menu.addAction("Rename")
        self.add_new_outcome = self.context_menu.addAction("Add new outcome")
        self.connect_free_sockets = self.context_menu.addAction("Connect free sockets")

    def set_name(self, name):
        """
            Set the name of both the subwindow and container

            @param name: Name given to the subwindow and container
        """
        self.setWindowTitle(name)
        self.container.set_name(name)

    def on_drag_enter(self, event):
        """
            Filter out what is accepted when receiving a drag enter event

            @param event: QDragEnterEvent sent py PyQt5
        """
        # If we have either a state or state machine entering accept the action otherwise ignore it
        if event.mimeData().hasFormat(LISTITEM_MIMETYPE):
            event.acceptProposedAction()
        else:
            event.setAccepted(False)

    def on_drop(self, event):
        """
            Handle the widget that is dropped to the graphical editor

            @param event: QDropEvent sent py PyQt5
        """
        # Check whether the MIME container has the proper format, otherwise ignore it
        if event.mimeData().hasFormat(LISTITEM_MIMETYPE):
            # Extract the data from the MIME container
            event_data = event.mimeData().data(LISTITEM_MIMETYPE)
            # Get the data stream
            data_stream = QDataStream(event_data, QIODevice.ReadOnly)
            # Extract the information from the stream
            is_state = data_stream.readBool()
            item_type = data_stream.readQString()
            # Get the position on which the object has been dropped
            mouse_position = event.pos()
            # Map it to the view coordinates
            view_position = self.container.get_view().mapToScene(mouse_position)
            if is_state:
                # Create a State object from the extracted information
                dropped_state = State(self.container, item_type)
                dropped_state.set_position(view_position.x(), view_position.y())
            else:
                state_machine_name, ok = QInputDialog().getText(self, "Input name", "Name of the state machine:",
                                                                QLineEdit.Normal)
                if state_machine_name and ok:
                    # Make sure we have unique names within the same container and between the containers
                    state_machine_name = self.container.get_unique_name(state_machine_name)
                    state_machine_name = self.parent().parent().parent().get_unique_name(state_machine_name)
                    # Create another tab and extract the container
                    self.parent().mdiArea().add_subwindow(state_machine_name, item_type)
                    container = self.parent().mdiArea().focused_subwindow.widget().container
                    # Create a state like representation to be displayed in the current widget
                    dropped_state_machine = StateMachine(self.container, container)
                    dropped_state_machine.set_position(view_position.x(), view_position.y())
                    # Link it to the newly created container
                    container.set_state_like(dropped_state_machine)
            # Now that the item has been added to the container, save a snapshot
            self.container.history.store_current_history()
            # Accept the drop action
            event.setDropAction(Qt.MoveAction)
            event.accept()
        else:
            event.ignore()

    def update_validity_icon(self):
        """
            Update the icon showing if this widget is properly configured
        """
        self.parent().change_icon(self.container.is_valid)

    def rename(self):
        """
            Spawn a dialog window to ask for the new name of the editor widget (and container)
        """
        container_name, ok = QInputDialog().getText(self, "Change name", "New name of {}:".format(self.windowTitle()),
                                                    QLineEdit.Normal)
        # If a name is provided and the OK button is pressed, rename the task configured in this widget
        # Otherwise just terminate this method
        if container_name and ok:
            self.set_name(container_name)
        else:
            return
        # If we try to rename the root, make sure to update the name of the task_config_file
        if self.is_root:
            self.parent().parent().parent().parent().framework_gui.update_task_config_file(container_name)

    def execute_container(self):
        """
            Generate all the python files required to run the state machine defined in the container and execute
            the resulting state machine
        """
        # If the root state machine has not be renamed, ask for a new name in order to avoid erasing a previously
        # generated file
        if self.container.type == "base" and self.container.name == "root":
            self.rename()
            # If the container has not been renamed then quit
            if self.container.name == "root":
                return
        # Parse the container
        parsed_container = self.container.get_parsed_container()
        task_editor_mdi_area = self.parent().mdiArea().parent()
        # Get the latest version of the sources
        state_source = task_editor_mdi_area.framework_gui.state_sources[0]
        # Get the state machine sources and add the folder containing the base state machine
        state_machine_sources = task_editor_mdi_area.framework_gui.state_machine_sources[:]
        state_machine_sources.append(BASE_STATE_MACHINE_FOLDER)
        # Get the commanders configuration
        commanders_config = self.robot_integration_area.commander_config
        # Generate the state machines
        generate_state_machines(parsed_container, state_source, state_machine_sources, GENERATED_STATE_MACHINE_FOLDER,
                                commanders_config)
        # Get the name of the root file
        python_file_to_run = os.path.join(GENERATED_STATE_MACHINE_FOLDER, parsed_container["name"] + ".py")
        self.launch_process = subprocess.Popen(['python -B {}'.format(python_file_to_run)], shell=True)
        # While running the execute option is disabled
        self.can_be_executed = False
        self.launch_process.communicate()
        # Once done, set the launch process to None
        self.launch_process = None
        # Re-enable the execute option
        self.can_be_executed = True

    def add_outcome(self, event_position):
        """
            Add a new terminal outcome for the task being currently edited

            @param event_position: QPointF corresponding to where the the action has been triggered by the user
        """
        if len(self.container.outcomes) == self.MAX_NUMBER_OUTCOMES:
            warning_message("Invalid operation", "The maximum number of outcomes has been reached!")
            return
        # Ask the user for the name of the new outcome
        outcome_name, ok = QInputDialog().getText(self, "Outcome name", "Name of the new outcome:", QLineEdit.Normal)
        # If OK is not pressed, then quit
        if not ok:
            return
        # Make sure we do not have two sockets with the same name
        if outcome_name in self.container.outcomes:
            warning_message("Invalid input", "An outcome with the same name already exists!",
                            "Please retry with a valid name", parent=self)
            return
        # Get the position on which the object should be added
        view_position = self.container.get_view().mapToScene(event_position)
        # Add a terminal socket to the container
        self.container.add_terminal_socket(outcome_name, [view_position.x(), view_position.y()])
        # Store the new content of the container
        self.container.history.store_current_history()

    def contextMenuEvent(self, event):
        """
            Function triggered when right click is pressed to make a context menu appear

            @param event: QContextMenuEvent sent by PyQt5
        """
        self.execute_action.setEnabled(self.container.is_valid and self.can_be_executed)
        # Execute the context menu
        action = self.context_menu.exec_(event.globalPos())
        if action == self.connect_free_sockets:
            self.container.connect_to_default_socket()
        elif action == self.rename_action:
            self.rename()
        elif action == self.execute_action:
            self.execute_container()
        elif action == self.add_new_outcome:
            self.add_outcome(event.pos())

    def save_config(self, settings):
        """
            Store the configuration of this widget into settings

            @param settings: QSettings object in which widgets' information are stored
        """
        settings.beginGroup("root" if self.is_root else self.windowTitle())
        # Get the name of the window so that we can restore it (only for root widget)
        if self.is_root:
            settings.setValue("name", self.windowTitle())
        # Get all information related to the container as a dictionary and save it
        settings.setValue("container", self.container.save())
        # Save the view
        self.editor_view.save_config(settings)
        # If saved, set the initial snapshot
        self.container.history.set_initial_snapshot()
        settings.endGroup()

    def restore_config(self, settings):
        """
            Restore the configuration of this widget from the parameters saved in settings

            @param settings: QSettings object in which widgets' information are stored
        """
        settings.beginGroup("root" if self.is_root else self.windowTitle())
        # Set the name of the window and container (only for root widget)
        if self.is_root:
            self.set_name(settings.value("name"))
        # Restore the container
        self.container.restore(settings.value("container"))
        # Store the saved configuration, but won't restore them as it must be done at execution time
        self.editor_view.store_config(settings)
        settings.endGroup()
        # Make sure to have everything as good as new
        self.container.history.clear()
        # Now that editor's content has been restored, save a snapshot
        self.container.history.store_current_history()
