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

import copy
import os
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QWidget, QGridLayout, QLabel, QPushButton, QSpacerItem, QFileDialog
from grip_core.utils.common_paths import CATKIN_WS
from grip_api.utils.common_dialog_boxes import can_save_warning_message
from grip_api.config_widgets.code_editors import YamlCodeEditor, XmlCodeEditor


class YAMLEditorWidget(GenericEditorWidget):

    """
        Widget containing the header and editor required to work with YAML files.
    """

    def __init__(self, name, enabled=True, parent=None):
        """
            Initialize the class by setting up the layout and the widgets

            @param name: String specifying what is the editor for
            @param enabled: Boolean determining whether the widget should be enabled or not when initialized
            @param parent: parent of the widget
        """
        self.file_path = None
        self.initial_path = None
        super().__init__(name=name, enabled=enabled, parent=parent)
        self.initial_input = dict()
        self.valid_input = dict()
        self.update_init_state = False
        self.should_emit_signal = True

    def get_file_path(self):
        """
            Returns the path of the file linked to the editor

            @return: Path to the file corresponding to the editor's content
        """
        return self.file_path

    def _create_header(self):
        """
            Create the header allowing to create, open, save, and close a new YAML file
        """
        super()._create_header()
        self.new_button = QPushButton("New")
        # Allows to have button that fit the text width
        self.new_button.setMaximumWidth(self.new_button.fontMetrics().boundingRect(self.new_button.text()).width() + 10)
        self.new_button.clicked.connect(self.new_file)
        self.open_button = QPushButton("Open")
        self.open_button.setMaximumWidth(
            self.open_button.fontMetrics().boundingRect(self.open_button.text()).width() + 10)
        self.open_button.clicked.connect(self.open_file)
        self.save_button = QPushButton("Save")
        self.save_button.setMaximumWidth(
            self.save_button.fontMetrics().boundingRect(self.save_button.text()).width() + 10)
        self.save_button.clicked.connect(self.save_file)
        self.save_as_button = QPushButton("Save as")
        self.save_as_button.setMaximumWidth(
            self.save_as_button.fontMetrics().boundingRect(self.save_as_button.text()).width() + 10)
        self.save_as_button.clicked.connect(self.save_file_as)
        self.close_button = QPushButton("Close")
        self.close_button.setMaximumWidth(
            self.close_button.fontMetrics().boundingRect(self.close_button.text()).width() + 10)
        self.close_button.clicked.connect(self.close_file)
        # Set widgets to the layout
        self.layout.addItem(QSpacerItem(30, 0), 0, 1)
        self.layout.addWidget(self.new_button, 0, 2)
        self.layout.addWidget(self.open_button, 0, 3)
        self.layout.addWidget(self.save_button, 0, 4)
        self.layout.addWidget(self.save_as_button, 0, 5)
        self.layout.addWidget(self.close_button, 0, 6)

    def _create_editor(self):
        """
            Initialize and set a YAML editor to the layout
        """
        self._code_editor = YamlCodeEditor()
        self.layout.addWidget(self._code_editor, 1, 0, 1, 7)
        self._code_editor.contentIsModified.connect(self.check_arguments_validity)

    def check_arguments_validity(self, is_different):
        """
            Make sure the parsed arguments are valid for the given editor (will be overloaded by children)

            @param is_different: Boolean sent by the signal stating whether the changes made lead to a different
                                 state of the editor
        """
        current_valid = self._code_editor.parsed_content if not self._code_editor.wrong_format_lines else None
        if not self.should_emit_signal:
            self.initial_input = copy.deepcopy(current_valid) if current_valid is not None else None
            self.should_emit_signal = True

        if current_valid != self.valid_input:
            self.valid_input = current_valid
            self.canBeSaved.emit(self.valid_input != self.initial_input and self.valid_input is not None)

        if self._code_editor.wrong_format_lines:
            test = True
        else:
            test = is_different

        self._header_title.setText(self.name + "*" if test and self.file_path else self.name)

        if self.update_init_state:
            self.update_init_widget()

    def update_init_widget(self):
        """
            Set the current state of the widget as the initial one
        """
        self._code_editor.reset_initial_content()
        self.update_init_state = False
        self._header_title.setText(self.name)

    def load_file(self):
        """
            Display the content of the file linked to the editor
        """
        with open(self.file_path, "r") as file_:
            yaml_content = "".join(file_.readlines())
        self.update_init_state = True
        self.set_editor_content(yaml_content)

    def open_file(self):
        """
            Open an already existing YAML file selected by the user
        """
        # If a file is already open make sure the user would not loose unaved changes
        if self.file_path and "*" in self._header_title.text():
            should_save = can_save_warning_message("Before closing...", "The current file has been modified",
                                                   additional_text="Do you want to save your changes?", parent=self)
            if should_save:
                self.save_file()
            elif should_save is None:
                return

        returned_path, _ = QFileDialog.getOpenFileName(self, "Select the {} file".format(self.name),
                                                       filter="{}(*{})".format("YAML", ".yaml"), directory=CATKIN_WS)
        if returned_path:
            self.file_path = returned_path
            self.load_file()

    def save_file(self):
        """
            Save the content of the editor to the linked file
        """
        if self.file_path is not None:
            with open(self.file_path, "w") as file_:
                file_.writelines(self._code_editor.text())
            self.update_init_widget()

    def save_file_path(self, message):
        """
            Makes sure the user specifies the path where the file must be saved

            @param message: Message to display when asking the user where the file should be saved
            @return: Boolean stating whether the required name as been provided
        """
        file_path, _ = QFileDialog.getSaveFileName(self, message, filter="YAML(*.yaml)", directory=CATKIN_WS)
        if file_path:
            if not file_path.endswith(".yaml"):
                file_path += ".yaml"
            self.file_path = file_path
        return file_path != ""

    def save_file_as(self):
        """
            Save the content of the editor to a file specified by the user
        """
        if self.save_file_path("Save configuration file as"):
            self.save_file()

    def new_file(self):
        """
            Create a new YAML file
        """
        # If a file is already open make sure the user would not loose unaved changes
        if self.file_path and "*" in self._header_title.text():
            should_save = can_save_warning_message("Before closing...", "The current file has been modified",
                                                   additional_text="Do you want to save your changes?", parent=self)
            if should_save:
                self.save_file()
            elif should_save is None:
                return
        if self.save_file_path("Save new configuration file as"):
            self._code_editor.make_editable()
            self._code_editor.remove_text()

    def close_file(self):
        """
            Unlinks the file from the editor, but the latter reamins enabled
        """
        self._code_editor.reinitialize()
        self.file_path = None
        self._header_title.setText(self.name)

    def reset(self):
        """
            Reset the editor and unlinks any file from the editor
        """
        self._code_editor.remove_text()
        self.file_path = None
        self._header_title.setText(self.name)

    def get_yaml_formatted_content(self):
        """
            Returns the content of the editor as a dictionary

            @return: None if the content is empty, otherwise a YAML-formated dictionary
        """
        if not self._code_editor.text():
            return None

        return self._code_editor.parsed_content

    def update_number_of_elements(self):
        """
            Update the number of elements comtained in the editor by taking into account potential user input
        """
        yaml_formatted = self.get_yaml_formatted_content()
        if yaml_formatted is None:
            self.number_components = 0
        else:
            self.number_components = len(yaml_formatted)

    def save_widget_configuration(self, settings):
        """
            Save the current state of the widget

            @param settings: PyQt5 object (QSettings) containing the information about the configuration of each widget
        """
        self.save_file()
        object_name = self.objectName()
        settings.beginGroup(object_name)
        settings.setValue("type", self.metaObject().className())
        settings.setValue("enabled", self.isEnabled())
        if self.file_path is not None:
            settings.setValue("file_path", self.file_path)
        else:
            settings.remove("file_path")
        settings.endGroup()

    def restore_widget_configuration(self, settings):
        """
            Set the different components of the widget according to a specified configuration

            @param settings: PyQt5 object (QSettings) containing the information about the configuration of each widget
        """
        # Make sure to close an already open file
        self.close_file()
        settings.beginGroup(self.objectName())
        if settings.contains("file_path") and os.path.exists(settings.value("file_path")):
            self.should_emit_signal = False
            self.file_path = settings.value("file_path")
            self.load_file()
        else:
            self._code_editor.remove_text()
        self.setEnabled(settings.value("enabled", type=bool))
        settings.endGroup()


class XMLEditorWidget(GenericEditorWidget):

    """
        Widget containing the header and editor required to work with XML files.
    """

    def __init__(self, name, enabled=False, parent=None):
        """
            Initialize the class by setting up the layout and the widgets

            @param name: String specifying what is the editor for
            @param enabled: Boolean determining whether the widget should be enabled or not when initialized
            @param parent: parent of the widget
        """
        super().__init__(name=name, enabled=enabled, parent=parent)
        self.reinitialize_inputs()

    def get_formated_arguments(self):
        """
            Return the arguments with proper format

            @return: String containing the formated arguments
        """
        if not self.valid_input:
            return
        formated_arguments = ""
        for argument in self.valid_input:
            formated_arguments += "{}\n\t".format(argument)
        formated_arguments = formated_arguments.rsplit("\t", 1)[0]
        return formated_arguments

    def _create_editor(self):
        """
            Initialize and set a XML compatible editor to the layout
        """
        self._code_editor = XmlCodeEditor()
        self._code_editor.contentIsModified.connect(self.check_arguments_validity)
        self.layout.addWidget(self._code_editor)

    def check_arguments_validity(self):
        """
            Make sure the parsed arguments are valid for the given editor
        """
        final_valid = None
        if self._code_editor.parsed_content is not None and not self._code_editor.wrong_format_lines:
            final_valid = self._code_editor.parsed_content[:]
        if final_valid != self.valid_input:
            self.valid_input = final_valid
            self.canBeSaved.emit(self.valid_input != self.initial_input and self.valid_input is not None)

    def reset_initial_input(self):
        """
            Set the initial input attribute value to the current valid input
        """
        if self.valid_input != self.initial_input:
            self.canBeSaved.emit(False)
        self.initial_input = self.valid_input[:] if self.valid_input is not None else None

    def reinitialize_inputs(self):
        """
            Set the valid and initial input to default value (None)
        """
        self.valid_input = None
        self.initial_input = None

    def save_widget_configuration(self, settings):
        """
            Save the current state of the widget

            @param settings: PyQt5 object (QSettings) containing the information about the configuration of each widget
        """
        object_name = self.objectName()
        settings.beginGroup(object_name)
        settings.setValue("type", self.metaObject().className())
        settings.setValue("enabled", self.isEnabled())
        if self.valid_input is not None:
            settings.setValue("value", self.valid_input)
        settings.endGroup()
        self.reset_initial_input()

    def restore_widget_configuration(self, settings):
        """
            Set the different components of the widget according to a specified configuration

            @param settings: PyQt5 object (QSettings) containing the information about the configuration of each widget
        """
        settings.beginGroup(self.objectName())
        if settings.contains("value"):
            retrieved_value = settings.value("value")
            self.valid_input = list() if retrieved_value is None else retrieved_value
        else:
            self.valid_input = None
        self.setEnabled(settings.value("enabled", type=bool))
        settings.endGroup()
        self.reset_initial_input()
        # Fill in the arguments
        if isinstance(self.valid_input, list):
            for index, input in enumerate(self.valid_input):
                self._code_editor.insertAt("  " + input + "\n", 2 + index, 0)
