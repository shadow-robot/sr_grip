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

import sys
import rospy
import signal
from PyQt5.QtWidgets import QMainWindow, QAction, QApplication, QTabWidget, QFileDialog
from PyQt5.QtCore import QFileInfo, QSettings
from grip_core.utils.common_paths import (ROBOT_CONFIG_FOLDER, MAIN_CONFIG_FILE, DEFAULT_TASK_CONFIG_FILE,
                                          DEFAULT_ROBOT_CONFIG_FILE, STATES_FOLDER, STATE_MACHINES_TEMPLATES_FOLDER)
from grip_core.utils.file_parsers import fill_available_states, fill_available_state_machines
from robot_integration_area import RobotIntegrationArea
from task_editor_area import TaskEditorArea
from grip_api.utils.common_dialog_boxes import can_save_warning_message, error_message


class FrameworkGui(QMainWindow):

    """
        Main window containing the GUI of the framework
    """

    # Main settings of the UI
    settings = QSettings(MAIN_CONFIG_FILE, QSettings.IniFormat)

    def __init__(self):
        """
            Initialize the class by generating the UI
        """
        super(FrameworkGui, self).__init__()
        # Load available state machines
        self.load_state_machines()
        # Load available states
        self.load_states()
        # Initialise the UI
        self.init_ui()
        # Contains the widget configurations required to load previous robot configurations
        self.robot_config_path = DEFAULT_ROBOT_CONFIG_FILE
        self.task_config_path = DEFAULT_TASK_CONFIG_FILE
        # Configure the GUI
        self.init_from_configs()

    def init_ui(self):
        """
            Set up the geometry of the main window and create the main widgets
        """
        self.init_main_widget()
        # status bar
        self.initialize_status_bar()
        # actions
        self.create_actions()
        # menus
        self.create_menus()
        # set window properties
        self.setGeometry(200, 200, 1000, 800)
        self.setWindowTitle("GRIP GUI")
        self.showMaximized()
        self.show()

    def init_main_widget(self):
        """
            Initialize the main widget that will contain all the others
        """
        # Initialize the tab widget (central widget)
        self.tab_container = QTabWidget(self)
        # Widget containing all the components required to configure and launch a robot
        self.robot_integration_area = RobotIntegrationArea(self)
        # Widget allowing to graphically design state machines
        self.task_editor_area = TaskEditorArea(self)
        # Update the menu related to the robot
        self.robot_integration_area.robotCanBeLaunched.connect(self.update_robot_launch_action)
        self.robot_integration_area.robotCanBeStopped.connect(self.update_robot_stop_action)
        # Update the task editor view corresponding to the vailidy of the robot config
        self.robot_integration_area.enableTaskEditor.connect(self.update_task_editor)
        # Update imported state machines/states if a new source is added
        self.task_editor_area.state_machine_displayer.stateMachineSourceAdded.connect(self.save_state_machine_source)
        self.task_editor_area.state_displayer.stateSourceAdded.connect(self.save_state_source)
        # Add widgets to corresponding tabs
        self.tab_container.addTab(self.robot_integration_area, "Integrate a robot")
        self.tab_container.addTab(self.task_editor_area, "Task editor")
        # By default, i.e. without any config, the task editor should not be accessible
        self.tab_container.setTabEnabled(1, False)
        self.setCentralWidget(self.tab_container)

    def initialize_status_bar(self):
        """
            Create and set a welcome message to the status bar
        """
        self.status_bar = self.statusBar()
        self.status_bar.showMessage("GUI launched -- Welcome!", 10000)

    def create_actions(self):
        """
            Create the different actions composing menus
        """
        # Allows to open a robot config
        self.action_open = QAction('&Open', self, shortcut='Ctrl+O', statusTip="Open file", triggered=self.open_file)
        # Save the current robot config
        self.action_save = QAction('&Save', self, shortcut='Ctrl+S', statusTip="Save file", triggered=self.save_file)
        # Save the current robot integration as
        self.action_save_as = QAction('Save &As...', self, shortcut='Ctrl+Shift+S', statusTip="Save file as...",
                                      triggered=self.save_file_as)
        # Exit the application
        self.action_exit = QAction('E&xit', self, shortcut='Ctrl+Q', statusTip="Exit application", triggered=self.exit)

        # Launch the robot
        self.launch_robot = QAction("&Launch", self, shortcut="Ctrl+L", statusTip="Launch the robot", enabled=False,
                                    triggered=self.robot_integration_area.launch_robot)
        self.stop_robot = QAction("Sto&p", self, shortcut="Ctrl+P", statusTip="Stop the robot", enabled=False,
                                  triggered=self.robot_integration_area.stop_robot)
        # Delete the selection in the task editor
        self.delete_selection = QAction('&Delete', self, shortcut='Del', statusTip="Delete selected items",
                                        triggered=self.delete)

    def save_state_machine_source(self, source):
        """
            Add a new state machine templates source

            @param source: Path to the folder containing templates
        """
        # Make sure not to have duplicates
        if source not in self.state_machine_sources:
            return
        # Add the path to the state machine sources
        self.state_machine_sources.append(source)
        self.settings.setValue("state_machine_sources", self.state_machine_sources)

    def save_state_source(self, source):
        """
            Add a new state source

            @param source: Path to the folder containing states
        """
        # Make sure not to have duplicates
        if source in self.state_sources:
            return
        # Add the path to the state sources
        self.state_sources.append(source)
        self.settings.setValue("state_sources", self.state_sources)

    def create_menus(self):
        """
            Create the "File" menu allowing to manage the robot integration config
        """
        # Create a menu bar
        menubar = self.menuBar()
        # Add the "File" menu
        self.file_menu = menubar.addMenu('&File')
        # Add the different actions
        self.file_menu.addAction(self.action_open)
        self.file_menu.addAction(self.action_save)
        self.file_menu.addAction(self.action_save_as)
        self.file_menu.addSeparator()
        self.file_menu.addAction(self.action_exit)
        # Add the "Robot" menu
        self.robot_menu = menubar.addMenu('&Robot')
        self.robot_menu.addAction(self.launch_robot)
        self.robot_menu.addAction(self.stop_robot)
        # Add the "Edit" menu
        self.edit_menu = menubar.addMenu("&Edit")
        self.edit_menu.addAction(self.delete_selection)
        # Make sure the content of the edit menu is only made available when the focus is on the task editor
        self.edit_menu.aboutToShow.connect(self.update_edit_menu)

    def update_edit_menu(self):
        """
            Enable/Disable the action allowing to delete the items part of the task editor
        """
        self.delete_selection.setEnabled(self.tab_container.currentWidget() is self.task_editor_area)

    def update_robot_launch_action(self, is_robot_launchable):
        """
            Enable/Disable the action allowing to launch the robot

            @param is_robot_launchable: Boolean coming from the signal and stating if the robot can be launched
        """
        self.launch_robot.setEnabled(is_robot_launchable)

    def update_robot_stop_action(self, is_robot_running):
        """
            Enable/Disable the action allowing to stop the robot

            @param is_robot_running: Boolean coming from the signal and stating whether the robot can be stopped
        """
        self.stop_robot.setEnabled(is_robot_running)

    def update_task_editor(self, enable_task_editor):
        """
            Enable/Disable the task editor

            @param enable_task_editor: Boolean coming from a signal and stating if the task editor should be accessible
        """
        # Activate/Deactivate the task editor
        self.tab_container.setTabEnabled(1, enable_task_editor)

    def open_file(self):
        """
            Open an already created config file, depending on which this function is called. It can either be the robot
            integration file or the task definition.
        """
        # Get the current widget the user is on when this function is called
        current_widget = self.tab_container.currentWidget()
        # Ask the user what to do if the current configuration has been changed
        user_action = self.check_if_save(current_widget)
        if user_action is None:
            return

        if current_widget is self.robot_integration_area:
            robot_config_path, _ = QFileDialog.getOpenFileName(self, "Open robot integration config file",
                                                               filter="ini(*.ini)",
                                                               directory=ROBOT_CONFIG_FOLDER)
            if robot_config_path:
                self.robot_config_path = robot_config_path
                self.settings.setValue("latest_robot_config", robot_config_path)
                self.init_robot_config()

    def save_file(self):
        """
            Save the current config file, depending on which widget this function is called. It can either be the robot
            integration file or the task definition.
        """
        # Save the current state of the robot configuration or the task, depending on which widget the user is on
        current_widget = self.tab_container.currentWidget()
        if current_widget is self.robot_integration_area:
            current_widget.save_config(self.latest_robot_config)
            self.settings.setValue("latest_robot_config", self.robot_config_path)
        else:
            current_widget.save_config(self.latest_task_config)
            self.settings.setValue("latest_task_config", self.task_config_path)

    def save_file_as(self):
        """
            Save the current config file (task or robot integration) with a specific name provided by the user
        """
        robot_config_path, _ = QFileDialog.getSaveFileName(self, "Save robot integration config file as",
                                                           filter="ini(*.ini)",
                                                           directory=ROBOT_CONFIG_FOLDER)
        # Ensures we get the correct extension for the config file
        if not robot_config_path.endswith(".ini"):
            robot_config_path += ".ini"

        self.robot_config_path = robot_config_path
        self.latest_robot_config = QSettings(self.robot_config_path, QSettings.IniFormat)
        self.save_file()

    def check_if_save(self, widget=None):
        """
            Check whether some changes are not saved. If it is the case ask the user what to do

            @return: True saving has been perofrmed by the user request, False changes are not to be saved and None
                     if user clicked on Cancel
        """
        if widget is None:
            widget = self.robot_integration_area
        should_save = False
        # Notifies the user that there are unsaved changes that can be lost
        if widget.can_be_saved:
            should_save = can_save_warning_message("Before leaving...", "The current configuration has been modified",
                                                   additional_text="Do you want to save your changes?", parent=self)
            if should_save:
                self.save_file()
                # Must call sync to force Qt to remain open, otherwise close without generating the files
                self.settings.sync()
                if widget is self.robot_integration_area:
                    self.latest_robot_config.sync()
        return should_save

    def exit(self):
        """
            Exit the GUI
        """
        if self.check_if_save() is None:
            return
        # If a robot is running, kill the process before exiting
        self.robot_integration_area.stop_robot()
        QApplication.exit()

    def delete(self):
        """
            Remove the selected items from the scene and associated graphics view
        """
        # This function will be called only when the focus is on the task editor
        self.tab_container.currentWidget().mdi_area.focused_subwindow.widget().container.get_view().delete_selected()

    def load_state_machines(self):
        """
            Load all the state machines from both internal and external sources
        """
        # Load state machines from all sources that might have been defined by the user as well
        if self.settings.contains("state_machine_sources"):
            self.state_machine_sources = self.settings.value("state_machine_sources")
        else:
            # Points to state machines provided by the framework
            self.state_machine_sources = [STATE_MACHINES_TEMPLATES_FOLDER]
        # Load state machines
        is_path_wrong = fill_available_state_machines(self.state_machine_sources)
        # If any of the path points to an invalid path then display a message
        if is_path_wrong:
            error_message("Error", "Error while processing the provided state machines!", parent=self)

    def load_states(self):
        """
            Load all the states from both internal and external sources
        """
        # Load states from all sources that might have been defined by the user as well
        if self.settings.contains("state_sources"):
            self.state_sources = self.settings.value("state_sources")
        else:
            # Points to states provided by the framework
            self.state_sources = [STATES_FOLDER]
        # Load states
        is_path_wrong = fill_available_states(self.state_sources)
        # If any of the path points to an invalid path then display a message
        if is_path_wrong:
            error_message("Error", "Error while processing the provided states!", parent=self)

    def init_from_configs(self):
        """
            Restore if possible all the widgets to their state saved into the robot and task configuration widgets
        """
        self.init_robot_config()
        self.init_task_config()

    def init_robot_config(self):
        """
            Restore (if possible) the state of the widgets allowing the user to interface a robot
        """
        # If a robot configuration has already been saved in a file, get its path
        if self.settings.contains("latest_robot_config"):
            self.robot_config_path = self.settings.value("latest_robot_config")

        # Create a Qt info file required to save the state of each widget used to configure a robot
        info_file = QFileInfo(self.robot_config_path)
        # Initialize the Qt settings file
        self.latest_robot_config = QSettings(self.robot_config_path, QSettings.IniFormat)
        # Introspect all the children widgets and call their restore_config() function
        if info_file.exists() and info_file.isFile():
            widget_names = self.latest_robot_config.childGroups()
            for widget_name in widget_names:
                widget = self.findChild(self.str_to_class(widget_name + "/type"), widget_name)
                widget.restore_config(self.latest_robot_config)

    def init_task_config(self):
        """
            Restore (if possible) the state of the widgets allowing the user to design the task of a robot
        """
        # If a task configuration has already been saved in a file, get its path
        if self.settings.contains("latest_task_config"):
            self.task_config_path = self.settings.value("latest_task_config")

        # Create a Qt info file required to save the state of each editor widget
        info_file = QFileInfo(self.task_config_path)
        # Initialize the Qt settings file
        self.latest_task_config = QSettings(self.task_config_path, QSettings.IniFormat)
        # Restore the configuration of the task editor if one is found
        if info_file.exists() and info_file.isFile():
            self.task_editor_area.restore_config(self.latest_task_config)
            # Make sure to restore the saved views for each subwindow
            self.tab_container.currentChanged.connect(self.restore_task_editor_views)

    def restore_task_editor_views(self, index):
        """
            Function called only once when the user clicks on the Task Editor tab and that the latter must restore a
            previously saved config

            @param index: Index of the tab that is being clicked on (0 for Robot Integration Area and I for Task Editor)
        """
        # If the Task Editor is clicked
        if index:
            # Restore the view of all the GraphicalEditorWidgets
            self.task_editor_area.mdi_area.restore_views()
            # Make sure to restore the views only once
            self.tab_container.currentChanged.disconnect()

    def str_to_class(self, class_name):
        """
            Turns a string (name of a class) into its class type

            @param class_name: String corresponding to the name of a class
            @return: Type of class
        """
        return getattr(sys.modules[__name__], self.latest_robot_config.value(class_name))

    def closeEvent(self, event):
        """
            Overwrites the default behaviour by calling the check_if_save function before proceeding

            @param event: QCloseEvent sent by PyQt5
        """
        if self.check_if_save() is not None:
            event.accept()
        else:
            event.ignore()

if __name__ == "__main__":
    rospy.init_node("framework_GUI")
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    framework_gui = FrameworkGui()
    framework_gui.show()

    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())
