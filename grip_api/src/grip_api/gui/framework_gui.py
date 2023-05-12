#!/usr/bin/env python3

# Copyright 2020, 2021, 2023 Shadow Robot Company Ltd.
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
import os
import rospy
import signal
from PyQt5.QtWidgets import QMainWindow, QAction, QApplication, QTabWidget, QFileDialog, QInputDialog, QLineEdit
from PyQt5.QtCore import QFileInfo, QSettings
from grip_core.utils.common_paths import (ROBOT_CONFIG_FOLDER, MAIN_CONFIG_FILE, DEFAULT_TASK_CONFIG_FILE,
                                          STATES_FOLDER, DEFAULT_ROBOT_CONFIG_FILE, STATE_MACHINES_TEMPLATES_FOLDER,
                                          TASK_CONFIG_FOLDER)
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
        super().__init__()
        # Load available state machines
        self.load_state_machines()
        # Load available states
        self.load_states()
        # Initialise the UI
        self.init_ui()
        # Contains the widget configurations required to load previous robot configurations
        self.robot_config_path = DEFAULT_ROBOT_CONFIG_FILE
        self.task_config_path = DEFAULT_TASK_CONFIG_FILE
        # Load the robot and task configuration
        self.init_robot_config()
        self.init_task_config()
        self.views_to_be_restored = True

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
        # Update the task editor view corresponding to the validity of the robot config
        self.robot_integration_area.enableTaskEditor.connect(self.update_task_editor)
        # Update imported state machines/states if a new source is added
        self.task_editor_area.state_machine_displayer.stateMachineSourceAdded.connect(self.save_state_machine_source)
        self.task_editor_area.state_displayer.stateSourceAdded.connect(self.save_state_source)
        # Add widgets to corresponding tabs
        self.tab_container.addTab(self.robot_integration_area, "Integrate a robot")
        self.tab_container.addTab(self.task_editor_area, "Task editor")
        # By default, i.e. without any config, the task editor should not be accessible
        self.tab_container.setTabEnabled(1, False)
        # Make sure sanity checks are carried out when users go on the task editor
        self.tab_container.currentChanged.connect(self.on_task_editor)
        self.setCentralWidget(self.tab_container)

    def on_task_editor(self, index):
        """
            Run sanity checks and make sure the views of each task editor is properly set

            @param index: Index of the tab that is eing activated by the user. 0 is for the integration and 1 for task
        """
        # Do nothing when going on the Robot integration area
        if not index:
            return
        # If the views need to be manually restored (it happens when loading the task in the __init__)
        if self.views_to_be_restored:
            self.task_editor_area.mdi_area.restore_views()
            self.views_to_be_restored = False
        # Make sure all the states currently used in the task editors match with the robot configuration
        # It is important since the robot integration can be changed while the task editor is on
        self.task_editor_area.mdi_area.update_items_availability()

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
        self.action_new = QAction('&New', self, shortcut='Ctrl+N', statusTip="New file", triggered=self.new_file)
        # Open a config (either robot or task)
        self.action_open = QAction('&Open', self, shortcut='Ctrl+O', statusTip="Open file", triggered=self.open_file)
        # Save the current config (either robot or task)
        self.action_save = QAction('&Save', self, shortcut='Ctrl+S', statusTip="Save file", triggered=self.save_file)
        # Save the current configuration as
        self.action_save_as = QAction('Save &As...', self, shortcut='Ctrl+Shift+S', statusTip="Save file as...",
                                      triggered=self.save_file_as)
        # Exit the application
        self.action_exit = QAction('E&xit', self, shortcut='Ctrl+Q', statusTip="Exit application", triggered=self.exit)

        # Launch and stop the robot
        self.launch_robot = QAction("&Launch", self, shortcut="Ctrl+L", statusTip="Launch the robot", enabled=False,
                                    triggered=self.robot_integration_area.launch_robot)
        self.stop_robot = QAction("Sto&p", self, shortcut="Ctrl+P", statusTip="Stop the robot", enabled=False,
                                  triggered=self.robot_integration_area.stop_robot)
        # Update the managers
        self.update_managers = QAction("Reset &Managers", self, shortcut="Ctrl+M", statusTip="Reset the managers",
                                       enabled=False, triggered=self.robot_integration_area.reset_managers)
        # Copy the elements that are selected in the task editor
        self.action_copy = QAction('&Copy', self, shortcut='Ctrl+C', statusTip="Copy selected items",
                                   triggered=self.task_editor_area.mdi_area.copy_task_editor_elements)
        # Cut the elements that are selected in the task editor
        self.action_cut = QAction('C&ut', self, shortcut='Ctrl+X', statusTip="Cut selected items",
                                  triggered=self.task_editor_area.mdi_area.copy_task_editor_elements)
        # Paste the elements that were previously copied in the task editor
        self.action_paste = QAction('&Paste', self, shortcut='Ctrl+V', statusTip="Paste previously copied items",
                                    triggered=self.task_editor_area.mdi_area.paste)

        self.action_undo = QAction('&Undo', self, shortcut='Ctrl+Z', statusTip="Undo last operation",
                                   triggered=self.task_editor_area.mdi_area.undo)
        self.action_redo = QAction('&Redo', self, shortcut='Ctrl+Y', statusTip="Redo last operation",
                                   triggered=self.task_editor_area.mdi_area.redo)
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
        # Make sure to write only the additional source of states in the task config file
        self.latest_task_config.setValue("state_sources", self.state_sources[1:])

    def create_menus(self):
        """
            Create the "File" menu allowing to manage the robot integration config
        """
        # Create a menu bar
        menubar = self.menuBar()
        # Add the "File" menu
        self.file_menu = menubar.addMenu('&File')
        # Add the different actions
        self.file_menu.addAction(self.action_new)
        self.file_menu.addAction(self.action_open)
        self.file_menu.addAction(self.action_save)
        self.file_menu.addAction(self.action_save_as)
        self.file_menu.addSeparator()
        self.file_menu.addAction(self.action_exit)
        # Add the "Edit" menu
        self.edit_menu = menubar.addMenu("&Edit")
        self.edit_menu.addAction(self.action_undo)
        self.edit_menu.addAction(self.action_redo)
        self.edit_menu.addSeparator()
        self.edit_menu.addAction(self.action_copy)
        self.edit_menu.addAction(self.action_cut)
        self.edit_menu.addAction(self.action_paste)
        self.edit_menu.addSeparator()
        self.edit_menu.addAction(self.delete_selection)
        # Add the "Robot" menu
        self.robot_menu = menubar.addMenu('&Robot')
        self.robot_menu.addAction(self.launch_robot)
        self.robot_menu.addAction(self.stop_robot)
        self.robot_menu.addAction(self.update_managers)
        # Make sure the content of the edit menu is only made available when the focus is on the task editor
        self.edit_menu.aboutToShow.connect(self.update_edit_menu)

    def update_edit_menu(self):
        """
            Enable/Disable the actions only related to the task editor
        """
        # Are we on the task editor area
        is_task_editor = self.tab_container.currentWidget() is self.task_editor_area
        self.action_copy.setEnabled(is_task_editor)
        self.action_cut.setEnabled(is_task_editor)
        self.action_paste.setEnabled(is_task_editor)
        self.action_undo.setEnabled(is_task_editor)
        self.action_redo.setEnabled(is_task_editor)
        self.delete_selection.setEnabled(is_task_editor)

    def update_robot_launch_action(self, is_robot_launchable):
        """
            Enable/Disable the action allowing to launch the robot

            @param is_robot_launchable: Boolean coming from the signal and stating if the robot can be launched
        """
        self.launch_robot.setEnabled(is_robot_launchable)

    def update_robot_stop_action(self, is_robot_running):
        """
            Enable/Disable the action allowing to stop the robot and to reset the managers

            @param is_robot_running: Boolean coming from the signal and stating whether the robot can be stopped
        """
        self.stop_robot.setEnabled(is_robot_running)
        self.update_managers.setEnabled(is_robot_running)

    def update_task_editor(self, enable_task_editor):
        """
            Enable/Disable the task editor

            @param enable_task_editor: Boolean coming from a signal and stating if the task editor should be accessible
        """
        # Activate/Deactivate the task editor
        self.tab_container.setTabEnabled(1, enable_task_editor)

    def new_file(self):
        """
            Create a new config file, either to interface a robot or to design a task, depending on where the user is
        """
        # Get the current widget the user is on when this function is called
        current_widget = self.tab_container.currentWidget()
        # Ask the user what to do if the current configuration has been changed
        user_action = self.check_if_save(current_widget)
        if user_action is None:
            return

        # Requests the name of the new configuration
        config_name, ok = QInputDialog().getText(self, "New config", "Name of the new configuration:", QLineEdit.Normal)
        config_filename = config_name
        # If a name has been given and OK has been pressed
        if config_name and ok:
            # Make sure the config filename ends with .ini
            if not config_name.endswith(".ini"):
                config_filename += ".ini"
        # Otherwise, stop there
        else:
            return

        if current_widget is self.robot_integration_area:
            # Get the path to the config file
            config_file_path = os.path.join(ROBOT_CONFIG_FOLDER, config_filename)
            # Load a new configuration file
            self.load_robot_config(config_file_path, new=True)
        else:
            # Get the path to the config file for task edition
            config_file_path = os.path.join(TASK_CONFIG_FOLDER, config_filename)
            # Load a new configuration file
            self.load_task_config(config_file_path, config_name=config_name, new=True)

    def open_file(self):
        """
            Open an already created config file (either for robot integration or for task definition).
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
                # Load the configuration file that has been chosen by the user
                self.load_robot_config(robot_config_path)
        else:
            task_config_path, _ = QFileDialog.getOpenFileName(self, "Open task config file", filter="ini(*.ini)",
                                                              directory=TASK_CONFIG_FOLDER)
            if task_config_path:
                # Load the task configuration file
                self.load_task_config(task_config_path)
                # Make sure we have the views restored
                self.task_editor_area.mdi_area.restore_views()

    def save_file(self, widget_to_save=None):
        """
            Save the state of a widget inside a config file, depending on which widget is passed to the function.
            If None is passed, it will save the widget contained in the current tab opened (task editor or integration)

            @param widget_to_save: Widget contained in one of the two tabs to save. If None is passed, get the current
                                   widget the user is on.
        """
        if widget_to_save is None or isinstance(widget_to_save, bool):
            widget_to_save = self.tab_container.currentWidget()

        if widget_to_save is self.robot_integration_area:
            widget_to_save.save_widget_configuration(self.latest_robot_config)
            self.settings.setValue("latest_robot_config", self.robot_config_path)
        else:
            if self.latest_task_config is None:
                self.init_task_config()
            widget_to_save.save_widget_configuration(self.latest_task_config)
            self.settings.setValue("latest_task_config", self.task_config_path)

    def save_file_as(self):
        """
            Save the current config file (task or robot integration) with a specific name provided by the user
        """
        # Get the current widget this function is called on
        current_widget = self.tab_container.currentWidget()
        # Get the path to where the config will be saved
        if current_widget is self.robot_integration_area:
            config_path, ok = QFileDialog.getSaveFileName(self, "Save robot integration config file as",
                                                          filter="ini(*.ini)",
                                                          directory=ROBOT_CONFIG_FOLDER)
        else:
            config_path, ok = QFileDialog.getSaveFileName(self, "Save task config file as", filter="ini(*.ini)",
                                                          directory=TASK_CONFIG_FOLDER)
        # If one path is provided and "Save" has been pressed
        if config_path and ok:
            # Make sure the config filename ends with .ini
            if not config_path.endswith(".ini"):
                config_path += ".ini"
        # Update the attribute of the class and create a new ini file
        if current_widget is self.robot_integration_area:
            self.robot_config_path = config_path
            self.latest_robot_config = QSettings(self.robot_config_path, QSettings.IniFormat)
        else:
            self.task_config_path = config_path
            self.latest_task_config = QSettings(self.task_config_path, QSettings.IniFormat)
            # Set the name of the task to the first window
            task_name = os.path.basename(config_path).replace(".ini", "")
            self.task_editor_area.mdi_area.subWindowList()[0].widget().set_name(task_name)
        # Save the current config in the new file
        self.save_file(current_widget)

    def check_if_save(self, widget=None):
        """
            Check if some changes are not saved. If it is the case ask the user what to do

            @param widget: Widget for which we want to know if unsaved changes have been made. If set to None, check all
                           the tabs of the tab_container.
            @return: True if saving has been performed by the user request, False if changes are not to be saved
                     and None if the user clicked on Cancel
        """
        # If no widget is provided, go over all the widgets contained in the tabs
        if widget is None:
            should_save = list()
            for tab_index in range(self.tab_container.count()):
                should_save.append(self.check_if_save(self.tab_container.widget(tab_index)))
            # If the user clicked on Cancel for any of the tab, return None, otherwise return False
            # Note that it can be True, as for now it does not matter
            return None if any(x is None for x in should_save) else False

        should_save = False
        # Notifies the user that there are unsaved changes that can be lost
        if widget.can_be_saved:
            should_save = can_save_warning_message("Before leaving...", "The current configuration has been modified",
                                                   additional_text="Do you want to save your changes?", parent=self)
            if should_save:
                self.save_file(widget)
                # Must call sync to force Qt to remain open, otherwise close without generating the files
                self.settings.sync()
                if widget is self.robot_integration_area:
                    self.latest_robot_config.sync()
                else:
                    self.latest_task_config.sync()
        return should_save

    def load_robot_config(self, config_file_path, new=False):
        """
            Load or create a robot configuration from a provided configuration file

            @param config_file_path: Path to the configuration file. Should end with the extension .ini
            @param new: Boolean stating if a new configuration file should be created
        """
        # Update the path to the current config file path
        self.robot_config_path = config_file_path
        self.settings.setValue("latest_robot_config", config_file_path)
        if new:
            # Create the .ini file
            self.latest_robot_config = QSettings(self.robot_config_path, QSettings.IniFormat)
        # Clear, i.e. reset all the widgets allowing to interface a new robot
        self.robot_integration_area.reset()
        # If we load an existing one, restore all the widgets
        if not new:
            self.init_robot_config()

    def load_task_config(self, config_file_path, config_name="root", new=False):
        """
            Load or create a task configuration from a provided configuration file

            @param config_file_path: Path to the configuration file. Should end with the extension .ini
            @param config_name: Name of the main window of the task editor
            @param new: Boolean stating if a new configuration file should be created
        """
        self.task_config_path = config_file_path
        self.settings.setValue("latest_task_config", config_file_path)
        if new:
            # Create the .ini file
            self.latest_task_config = QSettings(self.task_config_path, QSettings.IniFormat)
        # Clear, i.e. reset all the editors allowing to design a task and available states
        self.task_editor_area.reset(config_name)
        # If we load an existing one, restore all the widgets
        if not new:
            self.init_task_config()

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
            Load all the states from internal sources
        """
        # Points to states provided by the framework
        self.state_sources = [STATES_FOLDER]
        # Load states
        is_path_wrong = fill_available_states(self.state_sources)
        # If any of the path points to an invalid path then display a message
        if is_path_wrong:
            error_message("Error", "Error while processing the provided states!", parent=self)

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
        # Introspect all the children widgets and call their restore_widget_configuration() function
        if info_file.exists() and info_file.isFile():
            widget_names = self.latest_robot_config.childGroups()
            for widget_name in widget_names:
                widget = self.findChild(self.str_to_class(widget_name + "/type"), widget_name)
                widget.restore_widget_configuration(self.latest_robot_config)

    def init_task_config(self):
        """
            Restore (if possible) the state of the widgets allowing the user to design the task of a robot
        """
        # Make sure we don't restore the task editor area when the user can't access it due to a non valid config
        if not self.task_editor_area.isEnabled():
            self.latest_task_config = None
            return
        # If a task configuration has already been saved in a file, get its path
        if self.settings.contains("latest_task_config"):
            self.task_config_path = self.settings.value("latest_task_config")

        # Create a Qt info file required to save the state of each editor widget
        info_file = QFileInfo(self.task_config_path)
        # Initialize the Qt settings file
        self.latest_task_config = QSettings(self.task_config_path, QSettings.IniFormat)
        # Restore the configuration of the task editor if one is found
        if info_file.exists() and info_file.isFile():
            self.task_editor_area.restore_widget_configuration(self.latest_task_config)

    def str_to_class(self, class_name):
        """
            Turns a string (name of a class) into its class type

            @param class_name: String corresponding to the name of a class
            @return: Type of class
        """
        return getattr(sys.modules[__name__], self.latest_robot_config.value(class_name))

    def update_task_config_file(self, task_name):
        """
            Update the name of the task configuration file so it matches with the latest name of the main container

            @param task_name: Name of the main (i.e. root) container
        """
        previous_ini_config_path = self.task_config_path
        # Update the path of the config file
        self.task_config_path = os.path.join(os.path.dirname(self.task_config_path), "{}.ini".format(task_name))
        # Set it in the settings of the main window
        self.settings.setValue("latest_task_config", self.task_config_path)
        # Create a new QSettings for this specific file
        self.latest_task_config = QSettings(self.task_config_path, QSettings.IniFormat)
        # Store the current state of the task editor
        self.save_file()
        # If the previous ini file still exists, remove it since we want to replace it
        if os.path.exists(previous_ini_config_path):
            os.remove(previous_ini_config_path)

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
