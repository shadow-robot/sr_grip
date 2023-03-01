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

import abc
from PyQt5 import Qsci
from PyQt5.QtGui import QColor
from PyQt5.QtCore import pyqtSignal, QTimer, Qt


class MetaTextEditor(type(Qsci.QsciScintilla), abc.ABCMeta):
    """
        Meta class that allows for creating abstract base classes for text editors, inherited from Scintilla
    """


class BaseTextEditor(Qsci.QsciScintilla, abc.ABC, metaclass=MetaTextEditor):
    """
        QScintilla-based widget allowing to create a generic code editor
    """
    # Signal sent when the parsed content changes
    contentIsModified = pyqtSignal(bool)

    def __init__(self, parent=None):
        """
            Initialize the class by setting up the editor

            @param parent: parent of the widget
        """
        super().__init__(parent)
        self.is_lexed = False
        self.init_ui()
        self.init_backround_markers()
        self.lexer_ = None
        self.initial_content = {}
        # Dictionary that will have as keys the content of the line and as value the index of the line of new dicts
        self._new_dicts_index = {}
        # Will contain the index of the lines wrongly formatted
        self.wrong_format_lines = []
        # Timer used to check content format and that will be handled by a different thread
        self.timer = QTimer()
        # Set it to single shot (i.e. does not run continuously)
        self.timer.setSingleShot(True)
        # Once the timer is timing out then starts the check
        # We do this to avoid having stuff signaled as wrong while editing
        self.timer.timeout.connect(self.parse_and_format_editor)
        # Each time a new character is inserted in the editor, restart the timer
        self.textChanged.connect(self.start_timer)

    @property
    def is_lexed(self):
        """
            Return a boolean stating wheter the editor is lexed or not

            @return: Boolean indicating if the editor is lexed
        """
        return self._is_lexed

    @is_lexed.setter
    def is_lexed(self, boolean_value):
        """
            Set the boolean indicating whether the editor is lexed or not

            @param boolean_value: Boolean indicating if the editor is lexed or not
        """
        if not isinstance(boolean_value, bool):
            raise TypeError("The input value for 'is_lexed' must be a boolean")
        # Assing the input value if everything is all right
        self._is_lexed = boolean_value

    def init_ui(self):
        """
            Initialize the editor
        """
        # By default no lexer is set
        self.setLexer(None)
        # Set a grayish colour
        self.empty_color = QColor("#cccccc")
        self.setPaper(self.empty_color)
        # Set the tab width to 2 to save space
        self.setTabWidth(2)
        # Change tabs to spaces
        self.setIndentationsUseTabs(False)
        # Help to visualize the indentation
        self.setIndentationGuides(True)
        # Set auto indentation
        self.setAutoIndent(True)
        # Cannot be edited by the user
        self.setReadOnly(True)
        # Remove some of the standard shortcut embedded in QScintilla
        commands = self.standardCommands()
        commands.boundTo(Qt.ControlModifier | Qt.Key_L).setKey(0)
        commands.boundTo(Qt.ControlModifier | Qt.Key_T).setKey(0)

    def init_backround_markers(self):
        """
            Define markers to highlight lines not properly formatted using a red-ish colour
        """
        self.markerDefine(Qsci.QsciScintilla.Background, 0)
        self.setMarkerBackgroundColor(QColor("#40FF0000"), 0)

    def parse_and_format_editor(self):
        """
            Run the parser on the editor's content and signal which lines are not well formatted
        """
        # Parse the content of the editor
        self.parse_content()
        # Make the background of wrongly formatted lines red-ish
        self.update_background()

    def start_timer(self):
        """
            Start the timer that triggers the content's format checking
        """
        # The timer would timeout after 600 ms meaning that the check would happend 600ms after the last text edit
        self.timer.start(600)

    def stop_timer(self):
        """
            Stop the timer that triggers the content's format checking. After running this function, the content that
            will be set to the editor WON'T be automatically parsed and checked.
        """
        self.timer.stop()

    def set_text_and_trigger_checks(self, content):
        """
            Set the content of the editor and immedialty trigger checks about its validity

            @param content: String to be displayed in the editor
        """
        # Stop the timer (so we can trigger checks without having to wait for 600 ms)
        self.stop_timer()
        # Set the text and immediately trigger the checks
        self.setText(content)
        self.parse_and_format_editor()
        # Restart the timer so that when the users interact with the editor, things don't turn red too quickly
        self.start_timer()

    def update_background(self):
        """
            Update the markers based on which lines are detected as wrong
        """
        self.markerDeleteAll(0)
        lines = self.wrong_format_lines
        for line in lines:
            self.markerAdd(line, 0)

    @abc.abstractmethod
    def parse_content(self):
        """
            Parse the content of the editor
        """
        raise NotImplementedError(
            "The method 'parse content' has not been implemented!")

    def set_lexer(self):
        """
            Allow the user to edit the object
        """
        self.setLexer(self.lexer_)
        self.setReadOnly(False)
        self.is_lexed = True

    def reinitialize(self):
        """
            Set the editor to its initial state (uneditable with empty background)
        """
        self.clear()
        self.setLexer(None)
        self.is_lexed = False
        self.setReadOnly(True)
        self.setPaper(self.empty_color)
        self.markerDeleteAll()

    def reset(self):
        """
            Clean the editor (i.e. remove content and reset attributes) but keep it editable
        """
        self.clear()
        self.initial_content = {}
        self.parse_and_format_editor()

    @staticmethod
    def to_format(input_string):
        """
            Turn the input string to the intended format (string, int, float or boolean)

            @param input: String to convert
            @return: Either a string, an int, a float or a boolean
        """
        try:
            return int(input_string)
        except (TypeError, ValueError):
            pass
        try:
            return float(input_string)
        except (TypeError, ValueError):
            pass
        if input_string in ("true", "True"):
            return True
        if input_string in ("false", "False"):
            return False
        return str(input_string)
