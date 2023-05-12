#!/usr/bin/env python3

# Copyright 2023 Shadow Robot Company Ltd.
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
from abc import ABC, abstractmethod
import copy
from typing import Optional, List
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QWidget, QGridLayout, QLabel, QPushButton, QSpacerItem, QFileDialog
from grip_core.utils.common_paths import CATKIN_WS
from grip_api.utils.common_dialog_boxes import can_save_warning_message
from grip_api.utils.formatted_print import format_raise_string


class BaseEditorWidget(QWidget, ABC):

    """
        Abstract QWidget allowing users to graphically edit configuration files
    """
    # Signal triggered when the edited configuration file is in a state to be saved
    canBeSaved = pyqtSignal(bool)

    def __init__(self, name: str, enabled: bool = False, parent: Optional[QWidget] = None) -> None:
        """
            Initialize the class by setting up the layout and the widgets

            @param name: Name of the editor
            @param enabled: Whether the widget should be enabled or not when initialized
            @param parent: Parent of the widget
        """
        super().__init__(objectName=f"Editor {name}", parent=parent)
        self._name = name
        self._layout = None
        self._code_editor = None
        self._valid_input = None
        self._initial_input = None
        self._init_layout()
        self._create_header()
        self._create_editor()
        self.setEnabled(enabled)

    @property
    def valid_input(self) -> List[str]:
        """

        """
        return self._valid_input

    @valid_input.setter
    def valid_input(self, value: List[str]) -> None:
        """

        """
        raise NotImplemented(format_raise_string("You should not have to set `valid_input` from outside the object"))

    def _init_layout(self) -> None:
        """
            Set the object's layout
        """
        self._layout = QGridLayout()
        self._layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(self._layout)

    def _create_header(self) -> None:
        """
            Create a header displaying the name of the widget
        """
        self._header_title = QLabel(self._name)
        self._layout.addWidget(self._header_title)

    @abstractmethod
    def _create_editor(self) -> None:
        """
            Instantiate the proper code editor
        """
        raise NotImplementedError(format_raise_string("The method '_create_editor' has not been implemented!"))

    def set_editor_content(self, content: str) -> None:
        """
            Set the content of the editor

            @param content: Text to be displayed in the editor
        """
        if not isinstance(content, str):
            raise TypeError(format_raise_string("Editor's content can only be set"))
        # Make sure the editor is lexed before setting any text
        if self._code_editor.lexer() is None:
            self._code_editor.make_editable()
        self._code_editor.set_text_and_trigger_checks(content)
        self._code_editor.setReadOnly(False)
        self.setEnabled(True)
