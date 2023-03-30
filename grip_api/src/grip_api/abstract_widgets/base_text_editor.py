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
from typing import Union
from PyQt5 import Qsci
from PyQt5.QtGui import QColor
from PyQt5.QtCore import pyqtSignal, QTimer, Qt


class MetaTextEditor(type(Qsci.QsciScintilla), abc.ABCMeta):
    """
        Meta class that allows for creating abstract base classes for text editors, inherited from Scintilla
    """
    # When the text editor is empty (i.e. no new page or anything), define the background color as grey
    EMPTY_COLOR = QColor("#cccccc")
    # Amount of time (in ms) to wait since the last text edition to trigger parsing and validity checks
    TIME_BEFORE_PARSING = 600
    # Color of the background set to the lines that are wrongly formatted
    WRONG_FORMAT_COLOR = QColor("#40FF0000")


class BaseTextEditor(Qsci.QsciScintilla, abc.ABC, metaclass=MetaTextEditor):
    """
        QScintilla-based abstract text editor
    """
    # Signal sent when the parsed content changes
    contentIsModified = pyqtSignal(bool)

    def __init__(self) -> None:
        """
            Initialize the class by setting up the editor
        """
        super().__init__()
        self.text_lexer = None
        self._init_ui()
        # Initial content of the editor (i.e. text it is initialized with)
        self._initial_content = {}
        # Dictionary that will contain the name of new dictionaries (keys) and the line of their definition (values)
        self._new_dict_lines = {}
        # Will contain the index of the lines wrongly formatted
        self.wrong_format_lines = []
        # Timer used to check content format and that will be handled by a different thread
        self._timer = QTimer()
        # Set it to single shot (i.e. does not run continuously)
        self._timer.setSingleShot(True)
        # Once the timer is timing out then starts the check
        # We do this to avoid having stuff signaled as wrong while editing
        self._timer.timeout.connect(self.parse_and_format_editor)
        # Each time a new character is inserted in the editor (signal coming from QScintilla), restart the timer
        self.textChanged.connect(self._start_timer)

    def _init_ui(self) -> None:
        """
            Initialize the editor
        """
        # By default, make the editor non editable
        self.make_uneditable()
        # Set the background color with the corresponding color when the editor is empty
        self.setPaper(MetaTextEditor.EMPTY_COLOR)
        # Set the tab width to 2 to save space
        self.setTabWidth(2)
        # Change tabs to spaces
        self.setIndentationsUseTabs(False)
        # Help to visualize the indentation
        self.setIndentationGuides(True)
        # Set auto indentation
        self.setAutoIndent(True)
        # Remove some of the standard shortcut embedded in QScintilla
        commands = self.standardCommands()
        commands.boundTo(Qt.ControlModifier | Qt.Key_L).setKey(0)
        commands.boundTo(Qt.ControlModifier | Qt.Key_T).setKey(0)
        # Define markers to highlight lines not properly formatted using a red-ish color
        self.markerDefine(Qsci.QsciScintilla.Background, 0)
        self.setMarkerBackgroundColor(MetaTextEditor.WRONG_FORMAT_COLOR, 0)

    def parse_and_format_editor(self) -> None:
        """
            Run the parser on the editor's content and signal which lines are not well formatted
        """
        # Parse the content of the editor
        self._parse_content()
        # Make the background of wrongly formatted lines red-ish
        self._update_background()

    def _start_timer(self) -> None:
        """
            Start the timer that triggers the content's format checking
        """
        # The timer would timeout after 600 ms meaning that the checks would happen 600 ms after the last text edit
        self._timer.start(MetaTextEditor.TIME_BEFORE_PARSING)

    def _stop_timer(self):
        """
            Stop the timer that triggers the content's format checking. After running this function, the content that
            will be set to the editor WON'T be automatically parsed and checked.
        """
        self._timer.stop()

    def set_text_and_trigger_checks(self, input_text: str) -> None:
        """
            Set some input text to the editor and immediately trigger checks about its validity

            @param input_text: Raw text to be written to the editor
        """
        # Stop the timer (so we can trigger checks without having to wait for 600 ms)
        self._stop_timer()
        # Set the text and immediately trigger the checks
        self.setText(input_text)
        self.parse_and_format_editor()
        # Restart the timer so that when the users interact with the editor, things don't turn red too quickly
        self._start_timer()

    def _update_background(self) -> None:
        """
            Update the markers based on which lines are detected as wrong
        """
        self.markerDeleteAll(0)
        # Add markers
        for line_index in self.wrong_format_lines:
            self.markerAdd(line_index, 0)

    @abc.abstractmethod
    def _parse_content(self) -> None:
        """
            Parse the content of the editor
        """
        raise NotImplementedError("The method 'parse content' has not been implemented!")

    def make_editable(self) -> None:
        """
            Allow the user to edit the object
        """
        self.setLexer(self.text_lexer)
        self.setReadOnly(False)

    def make_uneditable(self) -> None:
        """
            Make sure the user cannot modify the current state of the text editor
        """
        self.setLexer(None)
        self.setReadOnly(True)

    def reinitialize(self) -> None:
        """
            Reinitialize the text editor to its initial state, i.e. uneditable with empty background
        """
        self.clear()
        self.make_uneditable()
        self.setPaper(MetaTextEditor.EMPTY_COLOR)
        self.markerDeleteAll()

    def remove_text(self) -> None:
        """
            Clear the editor (i.e. remove text and reset attributes) but keep it editable
        """
        self.clear()
        self._initial_content = {}
        self.parse_and_format_editor()

    @staticmethod
    # Note that in Python 3.10+, output could be hinted as type1 | type2 | type3
    def to_format(input_string: str) -> Union[int, float, bool, str]:
        """
            Turn the input string to the intended format (string, int, float or boolean)

            @param input_string: String to convert
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
