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

import os
from PyQt5.QtWidgets import QWidget, QGridLayout, QLabel, QLineEdit, QToolButton, QFileDialog
from PyQt5.QtCore import pyqtSignal
import rospkg
from grip_core.utils.common_paths import CATKIN_WS
from grip_api.utils.files_specifics import FILE_TO_EXTENSION


class GenericUserEntryWidget(QWidget):

    """
        Generic widget allowing the user to fill in entries either manually or using a browse button
    """
    # Signal used to notify that the provided input changed
    inputChanged = pyqtSignal()
    # Signal triggered when the content of the user entry can be saved
    canBeSaved = pyqtSignal(bool)

    def __init__(self, entry_name, is_optional, browser_button=True, placeholder_text=None, enabled=True, parent=None):
        """
            Initialize the widget

            @param entry_name: String to be displayed to specify what is expected
            @param is_optional: Boolean stating whether the user entry is optional to launch the robot
            @param browser_button: Boolean stating whether the browse button should be displayed
            @param placeholder_text: Optional text that can be displayed initially inside the edit line
            @param enabled: Boolean stating whether the widget should be initially enabled or not
        """
        super(GenericUserEntryWidget, self).__init__(parent=parent)
        self.entry_name = entry_name
        self.empty_value = str() if is_optional else None
        self.valid_input = self.empty_value
        self.initial_input = self.empty_value
        # Set the name of the object (UE stands for user entry)
        self.setObjectName("UE {}".format(entry_name))
        self.init_ui()
        self.create_entry(browser_button, placeholder_text)
        self.setEnabled(enabled)

    def init_ui(self):
        """
            Set the layout of the widget
        """
        self.layout = QGridLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)

    def create_entry(self, browser_button, placeholder_text):
        """
            Create an entry that consists of a label an edit line and potnetially a browse button

            @param browser_button: Boolean stating whether the browse button should be displayed
            @param placeholder_text: Optional text that can be displayed initially inside the edit line
        """
        # Label to display what the entry is about
        self.entry_label = QLabel(self.entry_name + ":", objectName="label {}".format(self.entry_name))
        # Line edit entry to let the user write and read the content of the entry
        self.entry_edit_line = QLineEdit(objectName="entry_edit_line {}".format(self.entry_name))
        # Set a small button allowing to clear the whole line
        self.entry_edit_line.setClearButtonEnabled(True)
        # When the text of the line is changed, check whether the input is valid
        self.entry_edit_line.textChanged.connect(self.check_input_validity)
        # If specified, set a placeholder text to show for isntance what is expected in this entry
        if placeholder_text is not None:
            self.entry_edit_line.setPlaceholderText(placeholder_text)

        self.layout.addWidget(self.entry_label, 0, 0)
        self.layout.addWidget(self.entry_edit_line, 0, 1)
        # If the user is expected to select a file or folder, create the corresponding button to do so
        if browser_button:
            self.browser_tool_button = QToolButton(objectName="browser {}".format(self.entry_name))
            self.browser_tool_button.setText("...")
            self.browser_tool_button.clicked.connect(self.fill_line_with_browsing)
            self.layout.addWidget(self.browser_tool_button, 0, 2)
        else:
            self.browser_tool_button = None
        self.setLayout(self.layout)

    def fill_line_with_browsing(self):
        """
            Open a window allowing the user to browse through the file system to provide either a file or a directory
            and automatically set it to the entry edit line
        """
        # Get the filter information for the given user entry
        filter_info = FILE_TO_EXTENSION[self.entry_name]
        # If it's not empty then open a dialog box allowing the user to select a file
        if filter_info:
            description, name, extension = filter_info
            returned_path, _ = QFileDialog.getOpenFileName(self, "Select the {} file".format(description),
                                                           filter="{}(*{})".format(name, extension),
                                                           directory=CATKIN_WS)
        # Otherwise it means what is expected is a folder
        else:
            returned_path = QFileDialog.getExistingDirectory(self, "Select the {} ".format(self.entry_name.lower()),
                                                             directory=CATKIN_WS)
        # If a folder or file has been selected then set the text with its path
        if returned_path:
            self.entry_edit_line.setText(returned_path)

    def check_input_validity(self):
        """
            Method that will be specific to each instance that checks if the input is valid
        """
        pass

    def check_file_validity(self, extension):
        """
            Check whether the provided input is a file which the expected extension

            @param extension: Extension the filename should have
            @return: True if the edit line text corresponds to a file with the given extension, False otherwise
        """
        current_input = self.entry_edit_line.text()
        is_valid_file = current_input.endswith(extension)
        return current_input and os.path.isfile(current_input) and is_valid_file or not current_input

    def update_valid_input(self, value, is_valid):
        """
            Update the value of the attribute valid_input and emit the signal if required.
            It also changes the edit line's background colour to show whether the input is valid or not

            @param value: new_value that should be set to valid_input if valid is True
            @param is_valid: Boolean that states whether the input is valid or not
        """
        entry_text = self.entry_edit_line.text()
        final_valid = value if is_valid else self.empty_value if not entry_text else None
        if final_valid != self.valid_input:
            self.valid_input = final_valid
            self.inputChanged.emit()
            self.canBeSaved.emit(self.valid_input != self.initial_input and (is_valid or not entry_text))
        if not is_valid and entry_text:
            self.entry_edit_line.setStyleSheet("background-color: rgba(255, 0, 0, 75)")
        else:
            self.entry_edit_line.setStyleSheet("background-color: rgb(255, 255, 255)")

    def reset_init_input(self):
        """
            Set the value of initial input ot the current valid
        """
        if self.valid_input != self.initial_input:
            self.canBeSaved.emit(False)
        self.initial_input = self.valid_input

    def save_config(self, settings):
        """
            Save the current state of the widget

            @param settings: PyQt5 object (QSettings) containing the information about the configuration of each widget
        """
        object_name = self.objectName()
        settings.beginGroup(object_name)
        settings.setValue("type", self.metaObject().className())
        settings.setValue("value", self.valid_input)
        settings.setValue("enabled", self.isEnabled())
        settings.endGroup()
        self.reset_init_input()

    def restore_config(self, settings):
        """
            Set the different components of the widget according to a specified configuration

            @param settings: PyQt5 object (QSettings) containing the information about the configuration of each widget
        """
        settings.beginGroup(self.objectName())
        stored_value = settings.value("value")
        value_to_set = stored_value[0] if isinstance(stored_value, tuple) else stored_value
        self.initial_input = stored_value
        self.entry_edit_line.setText(value_to_set)
        self.setEnabled(settings.value("enabled", type=bool))
        settings.endGroup()


class UrdfEntryWidget(GenericUserEntryWidget):

    """
        Widget specific to the input of the URDF file
    """

    def __init__(self, browser_button=True, placeholder_text=None, enabled=True, parent=None):
        """
            Initialize the widget

            @param browser_button: Boolean stating whether the browse button should be displayed
            @param placeholder_text: Optional text that can be displayed initially inside the edit line
            @param enabled: Boolean stating whether the widget should be initially enabled or not
        """
        super(UrdfEntryWidget, self).__init__("Robot's URDF file", False, browser_button, placeholder_text,
                                              enabled, parent)

    def check_input_validity(self):
        """
            Check that the current text corresponds to a file ending either with .urdf or .urdf.xacro
        """
        current_input = self.entry_edit_line.text()
        is_valid_file = current_input.endswith(".urdf") or current_input.endswith(".urdf.xacro")
        is_valid = current_input and is_valid_file and os.path.isfile(current_input) or not current_input
        self.update_valid_input(current_input, is_valid and current_input != "")


class UrdfArgumentsEntryWidget(GenericUserEntryWidget):

    """
        Widget specific to the input of optional URDF arguments
    """

    def __init__(self, browser_button=True, placeholder_text=None, enabled=True, parent=None):
        """
            Initialize the widget

            @param browser_button: Boolean stating whether the browse button should be displayed
            @param placeholder_text: Optional text that can be displayed initially inside the edit line
            @param enabled: Boolean stating whether the widget should be initially enabled or not
        """
        super(UrdfArgumentsEntryWidget, self).__init__("URDF arguments (optional)", True, browser_button,
                                                       placeholder_text, enabled, parent)

    def check_input_validity(self):
        """
            Check that the format of the current text corresponds is valid
        """
        current_input = self.entry_edit_line.text()
        space_split_input = current_input.split(" ")
        is_valid = current_input and all(":=" in x for x in space_split_input) or not current_input
        self.update_valid_input(current_input, is_valid and current_input != "")


class LaunchFileEntryWidget(GenericUserEntryWidget):

    """
        Widget specific to the input of a custom launch file path
    """

    def __init__(self, browser_button=True, placeholder_text=None, enabled=True, parent=None):
        """
            Initialize the widget

            @param browser_button: Boolean stating whether the browse button should be displayed
            @param placeholder_text: Optional text that can be displayed initially inside the edit line
            @param enabled: Boolean stating whether the widget should be initially enabled or not
        """
        super(LaunchFileEntryWidget, self).__init__("Custom launch file", True, browser_button, placeholder_text,
                                                    enabled, parent)

    def check_input_validity(self):
        """
            Check that the format of the current text corresponds to a file ending with .launch and is part of a ROS pkg
        """
        current_input = self.entry_edit_line.text()
        is_extension_valid = current_input.endswith(".launch")
        ros_pkg = rospkg.get_package_name(current_input)
        is_valid = current_input and is_extension_valid and ros_pkg is not None and os.path.exists(current_input)
        self.update_valid_input((current_input, ros_pkg), is_valid and current_input != "")


class CollisionFileEntryWidget(GenericUserEntryWidget):

    """
        Widget specific to the input of a collision file
    """

    def __init__(self, browser_button=True, placeholder_text=None, enabled=True, parent=None):
        """
            Initialize the widget

            @param browser_button: Boolean stating whether the browse button should be displayed
            @param placeholder_text: Optional text that can be displayed initially inside the edit line
            @param enabled: Boolean stating whether the widget should be initially enabled or not
        """
        super(CollisionFileEntryWidget, self).__init__("Collision scene", True, browser_button, placeholder_text,
                                                       enabled, parent)

    def check_input_validity(self):
        """
            Check if the current text corresponds to a file ending by ".scene"
        """
        is_valid = self.check_file_validity(".scene")
        current_input = self.entry_edit_line.text()
        self.update_valid_input(current_input, is_valid and current_input != "")


class GazeboWorldEntryWidget(GenericUserEntryWidget):

    """
        Widget specific to the input of a Gazebo's world file
    """

    def __init__(self, browser_button=True, placeholder_text=None, enabled=True, parent=None):
        """
            Initialize the widget

            @param browser_button: Boolean stating whether the browse button should be displayed
            @param placeholder_text: Optional text that can be displayed initially inside the edit line
            @param enabled: Boolean stating whether the widget should be initially enabled or not
        """
        super(GazeboWorldEntryWidget, self).__init__("Gazebo world file", False, browser_button, placeholder_text,
                                                     enabled, parent)

    def check_input_validity(self):
        """
            Check if the current text corresponds to a file ending by ".world"
        """
        is_valid = self.check_file_validity(".world")
        current_input = self.entry_edit_line.text()
        self.update_valid_input(current_input, is_valid and current_input != "")


class GazeboFolderEntryWidget(GenericUserEntryWidget):

    """
        Widget specific to the input of a Gazebo's model folder
    """

    def __init__(self, browser_button=True, placeholder_text=None, enabled=True, parent=None):
        """
            Initialize the widget

            @param browser_button: Boolean stating whether the browse button should be displayed
            @param placeholder_text: Optional text that can be displayed initially inside the edit line
            @param enabled: Boolean stating whether the widget should be initially enabled or not
        """
        super(GazeboFolderEntryWidget, self).__init__("Gazebo model folder", False, browser_button, placeholder_text,
                                                      enabled, parent)

    def check_input_validity(self):
        """
            Check if the current text corresponds to a path to a folder
        """
        current_input = self.entry_edit_line.text()
        is_valid = current_input and os.path.isdir(current_input) or not current_input
        self.update_valid_input(current_input, is_valid and current_input != "")


class StartingPoseEntryWidget(GenericUserEntryWidget):

    """
        Widget specific to the input of the starting pose of the robot in simulation
    """

    def __init__(self, browser_button=True, placeholder_text=None, enabled=True, parent=None):
        """
            Initialize the widget

            @param browser_button: Boolean stating whether the browse button should be displayed
            @param placeholder_text: Optional text that can be displayed initially inside the edit line
            @param enabled: Boolean stating whether the widget should be initially enabled or not
        """
        super(StartingPoseEntryWidget, self).__init__("Starting pose", True, browser_button, placeholder_text,
                                                      enabled, parent)

    def check_input_validity(self):
        """
            Check if the current text follow the format "-J joint_name joint_value"
        """
        current_input = self.entry_edit_line.text()
        contains_joint_keyword = current_input.count("-J ") != 0
        # The first element is always a space
        split_input = current_input.split("-J ")[1:]
        # Makes sure no -J has been forgotten
        proper_number = current_input.count("-J ") == len(split_input)
        # Makes sure that things between -J are properly configured. The strip remove potential trailing space
        all_args_good = all(len(argument.strip(" ").split(" ")) == 2 for argument in split_input)
        is_valid = current_input and contains_joint_keyword and proper_number and all_args_good or not current_input
        self.update_valid_input(current_input, is_valid and current_input != "")


class MoveitPackageEntryWidget(GenericUserEntryWidget):

    """
        Widget specific to the input of a MoveIt! package
    """

    def __init__(self, browser_button=True, placeholder_text=None, enabled=True, parent=None):
        """
            Initialize the widget

            @param browser_button: Boolean stating whether the browse button should be displayed
            @param placeholder_text: Optional text that can be displayed initially inside the edit line
            @param enabled: Boolean stating whether the widget should be initially enabled or not
        """
        super(MoveitPackageEntryWidget, self).__init__("Moveit package", True, browser_button, placeholder_text,
                                                       enabled, parent)

    def check_input_validity(self):
        """
            Check if the current text corresponds to a ROS package
        """
        current_input = self.entry_edit_line.text()
        ros_pkg = rospkg.get_package_name(current_input)
        is_valid = current_input and os.path.isdir(current_input) and ros_pkg is not None or not current_input
        self.update_valid_input((current_input, ros_pkg), is_valid and current_input != "")
