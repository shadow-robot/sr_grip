#!/usr/bin/env python

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

from PyQt5.QtWidgets import QGridLayout, QWidget, QInputDialog, QLineEdit, QMenu
from PyQt5.QtCore import Qt, QDataStream, QIODevice
from grip_api.task_editor_graphics.view import TaskEditorView
from container import Container
from state_machine import StateMachine
from grip_api.utils.files_specifics import LISTITEM_MIMETYPE
from grip_core.state_machine_generator.state_machine_generator import generate_state_machines
from grip_core.utils.common_paths import GENERATED_STATE_MACHINE_FOLDER, BASE_STATE_MACHINE_FOLDER
from state import State
import os
import subprocess


class GraphicalEditorWidget(QWidget):

    """
        Widget gathering the high level logic and event handler allowing the user to edit state machines
    """

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
        # By default the container of this editor cannot be launched
        self.can_be_executed = False
        # Update the above attribute according to whether the robot is launched or not
        self.robot_integration_area = self.parent().parent().parent().framework_gui.robot_integration_area
        self.robot_integration_area.robotCanBeStopped.connect(self.update_execution)

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
        self.container.create_terminal_sockets()
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
                    # Create another tab and extract the container
                    self.parent().mdiArea().add_subwindow(state_machine_name, item_type)
                    container = self.parent().mdiArea().focused_subwindow.widget().container
                    # Create a state like representation to be displayed in the current widget
                    dropped_state_machine = StateMachine(self.container, container)
                    dropped_state_machine.set_position(view_position.x(), view_position.y())
                    # Link it to the newly created container
                    container.set_state_like(dropped_state_machine)
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
        if container_name and ok:
            self.set_name(container_name)

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

    def save_config(self, settings):
        """
            Store the configuration of this widget into settings

            @param settings: QSettings object in which widgets' information are stored
        """
        settings.beginGroup("root")
        # Get the name of the window so that we can restore that
        settings.setValue("name", self.windowTitle())
        # Get all information relared to the container as a dictionary and saves it
        settings.setValue("container", self.container.save())
        # Save the view
        self.editor_view.save_config(settings)
        settings.endGroup()

    def restore_config(self, settings):
        """
            Restore the configuration of this widget from the parameters saved in settings

            @param settings: QSettings object in which widgets' information are stored
        """
        settings.beginGroup("root")
        # Set the name of the window and container
        self.set_name(settings.value("name"))
        # Restore the container
        self.container.restore(settings.value("container"))
        # Restore the view
        self.editor_view.restore_config(settings)
        settings.endGroup()
