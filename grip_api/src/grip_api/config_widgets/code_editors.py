#!/usr/bin/env python3

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

import re
from collections import OrderedDict
import copy
from PyQt5 import Qsci
from PyQt5.QtGui import QColor
from PyQt5.QtCore import pyqtSignal, QTimer, Qt


class GenericCodeEditor(Qsci.QsciScintilla):

    """
        QScintilla-based widget allowing to create a generic code editor
    """
    # Signal sent when the the parsed content changes
    contentIsModified = pyqtSignal(bool)

    def __init__(self, parent=None):
        """
            Initialize the class by setting up the editor

            @param parent: parent of the widget
        """
        super(GenericCodeEditor, self).__init__(parent)
        self.init_ui()
        self.init_backround_markers()
        self.lexer_ = None
        self.initial_content = OrderedDict()
        # Will contain the index of the lines wrongly formatted
        self.wrong_format_lines = list()
        # Timer used to check content format and that will be handled by a different thread
        self.timer = QTimer()
        # Set it to single shot (i.e. does not run continuously)
        self.timer.setSingleShot(True)
        # Once the timer is timing out then starts the check
        # We do this to avoid having stuff signaled as wrong while editing
        self.timer.timeout.connect(self.parse_and_format_editor)
        # Each time a new character is inserted in the editor, restart the timer
        self.textChanged.connect(self.start_timer)

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
        self.is_lexed = False
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

    def parse_content(self):
        """
            Parse the content of the editor (will be overridden by children classes)
        """
        return

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
        self.initial_content = OrderedDict()
        self.parse_and_format_editor()

    @staticmethod
    def to_format(input):
        """
            Turn the string input to the intended format (string, int, float or boolean)

            @param input: String to convert
            @return: Either a string, an int, a float or a boolean
        """
        try:
            return int(input)
        except (TypeError, ValueError):
            pass
        try:
            return float(input)
        except (TypeError, ValueError):
            pass
        if input in ("true", "True"):
            return True
        elif input in ("false", "False"):
            return False
        else:
            return str(input)


class YamlCodeEditor(GenericCodeEditor):

    """
        QScintilla-based widget allowing to create a YAML code editor
    """

    def __init__(self, parent=None):
        """
            Initialize the class by setting up the editor

            @param parent: parent of the widget
        """
        super(YamlCodeEditor, self).__init__(parent)
        self.init_symbol_margin()
        self.lexer_ = Qsci.QsciLexerYAML(self)
        # Will contain the parsed content
        self.parsed_content = OrderedDict()

    def init_symbol_margin(self):
        """
            Initialize the margin with a symbol allowing to help integrating a given component
        """
        # Give the ability to set symbols in the margin
        self.setMarginType(1, Qsci.QsciScintilla.SymbolMargin)
        # Make sure the margin does not become too large
        self.setMarginWidth(1, "00")
        # Define a plus marker (index 0)
        self.markerDefine(Qsci.QsciScintilla.Plus, 1)
        # Make the margin clickable
        self.setMarginSensitivity(1, True)

    def set_autocompletion(self, items):
        """
            Allow the strings contained in items to be autocompleted after two characters

            @param items: List of strings containing the words to propose for autocompletion
        """
        self.setAutoCompletionSource(Qsci.QsciScintilla.AcsAPIs)
        self.setAutoCompletionThreshold(2)
        self.api = Qsci.QsciAPIs(self.lexer_)
        for item in items:
            self.api.add(item)
        self.api.prepare()

    def turn_off_autocompletion(self):
        """
            Turn off autocompletion
        """
        self.setAutoCompletionSource(Qsci.QsciScintilla.AcsNone)

    def parse_content(self):
        """
            Parse the current editor's content and store the valid content in the parsed_content attribute
        """
        # Reinitialize the list of wrong lines
        self.wrong_format_lines = list()
        editor_content = self.text()
        # If the editor has been emptied can just stop the parsing here
        if not editor_content:
            self.parsed_content = OrderedDict()
            self.contentIsModified.emit(self.initial_content != self.parsed_content)
            return
        # Get the text line by line
        split_content = editor_content.split("\n")
        # Get indices of all lines starting without any space or \n (so lines defining a new root component)
        zero_depth_indices = [i for i, x in enumerate(split_content) if re.search("^(\w+)", x) is not None]
        # If nothing has been found (meaning that all the content is wrong), set all the lines in a single list
        if not len(zero_depth_indices):
            self.slices = [split_content]
        # If at least one is found, create slices from these indices (without forgetting from line 0 to first index)
        else:
            self.slices = [split_content[:zero_depth_indices[0]]] if zero_depth_indices[0] else []
            self.slices += [split_content[zero_depth_indices[i]:zero_depth_indices[i + 1]]
                            for i in range(len(zero_depth_indices) - 1)] + [split_content[zero_depth_indices[-1]:]]
        # Current line number
        line_number = 0
        # Will contain the YAML parsed content of all valid root components of the editor
        parsed = OrderedDict()
        # This dictionary is used to recursively fill the different dictionaries.
        # Keys correspond to the depth of the parent of the current element
        # Values correspond to the container of such elements
        parent_dictionary = OrderedDict([(-1, parsed)])
        # Dictionary that will have as keys the content of the line and as value the index of the line of new dicts
        # It includes of course the indices previously computed but also the dicts created at higher levels
        self.new_dicts_index = OrderedDict()
        # For each slice corresponding to a root component (depth = 0)
        for slice in self.slices:
            # Expected depth is used to detect wrong indentation
            expected_depth = 0
            # For each line of the given slice
            for line in slice:
                # Search for spaces at the beginning of the line
                starting_spaces = re.search("(^\s{1,})", line)
                # Get the number of spaces out of the search
                number_space = len(starting_spaces.group(1)) if starting_spaces else 0
                # As depth increases byt two spaces computes it
                # To be valid the number of space should be even. If it's not the case set depth to a higher value
                depth = number_space / 2 if number_space % 2 == 0 else expected_depth + 1
                # If depth is higher than expected_depth it means that the indentation is wrong
                if depth > expected_depth and line.strip():
                    self.wrong_format_lines.append(line_number)
                    line_number += 1
                    continue
                # If depth is smaller than the expected one update the expected to detect potential future wrong depth
                if depth < expected_depth and depth:
                    expected_depth = depth
                # Remove any trailing space from the line
                clean = line.strip()
                # Use a regex to extract all kind of information that can be part of a line
                split_line = re.search("([^\#\:\s\-\{\}]*)(\s?\:\s?)?(?(2)([^\[\{\:\s\#]*)|)?(\-\s?)?"
                                       "(?(4)([^\{\[\#]*)|)?(\{[^\#\[\{\(\]\}\)]*\})?(\[[^\#\:\[\{\]\}\(\)]*\])?"
                                       "(\s*\#.*)?(.*)?", clean).groups()
                # Unstack all the information contained in the line into different variables
                key_name, column, value, dash, list_element, condensed_dict, condensed_list, comment, trash = split_line
                # If there are unexpected stuff on the line then continue and go to the next line
                if trash:
                    self.wrong_format_lines.append(line_number)
                    line_number += 1
                    continue
                # Otherwise remove the last element for further tests
                else:
                    split_line = split_line[:-1]
                # If it's an empty line then continue to the next line
                if all(not x for x in split_line):
                    line_number += 1
                    continue
                # If we have only a comment on the line then go to the next line
                if comment and all(not x for x in split_line[:-1]):
                    line_number += 1
                    continue
                # If only text is present on the line (without :) then it is invalid
                if key_name and all(not x for x in split_line[1:-1]):
                    # Add the line to wrong format so that it appears in red-ish
                    self.wrong_format_lines.append(line_number)
                    line_number += 1
                    continue
                # If we have only a keyword and a column, mark the line as wrong as we don't know what's following
                # However add the element to new_dicts_index
                if key_name and column and all(not x for x in split_line[2:-1]):
                    self.wrong_format_lines.append(line_number)
                    self.new_dicts_index[key_name] = line_number
                    # Since it might be the beginning of another subelement, increment expected_depth
                    expected_depth += 1
                # If a root component is a list or a condensed element, it is signaled as invalid and go to next line
                if not depth and not key_name and (dash or condensed_dict or condensed_list):
                    self.wrong_format_lines.append(line_number)
                    line_number += 1
                    continue
                # If a dash is not followed by anything valid then make the line wrong and go to the next one
                if dash and not (list_element or condensed_list or condensed_dict):
                    self.wrong_format_lines.append(line_number)
                    line_number += 1
                    continue
                # If the line contains too many things like key_name: value -/[]/{} make it wrong and got to the next
                if all(split_line[:3]) and any(split_line[3:-1]):
                    self.wrong_format_lines.append(line_number)
                    line_number += 1
                    continue
                # If trying to add anything that is not a list after a list
                if depth - 1 not in parent_dictionary and not dash:
                    self.wrong_format_lines.append(line_number)
                    line_number += 1
                    continue

                # Start parsing the content
                # Parse potential condensed dictionary
                if condensed_dict:
                    content = re.search("\{(.*)\}", condensed_dict).group(1)
                    dict_args = re.findall("([^\:\s\{\,]*)\s?:\s?([^\:\s\}\,]*)", condensed_dict)
                    # If there is some text but not properly formatted mark the line as wrong and go to the next one
                    if not dict_args and content:
                        self.wrong_format_lines.append(line_number)
                        line_number += 1
                        continue
                    # If one of the arguments is badly formatted
                    if content and len(re.split(",", content)) != len(dict_args):
                        self.wrong_format_lines.append(line_number)
                        line_number += 1
                        continue
                    # If one of the value of the condensed dict is empty
                    if any(not x or not y for x, y in dict_args):
                        self.wrong_format_lines.append(line_number)
                        line_number += 1
                        continue
                    formatted_dict_args = [(self.to_format(x), self.to_format(y)) for x, y in dict_args]
                    checked_condensed_dict = OrderedDict(formatted_dict_args)

                # Parse potential condensed list
                if condensed_list:
                    content = re.search("\[(.*)\]", condensed_list).group(1)
                    elements = content.split(",") if content else list()
                    checked_condensed_list = [self.to_format(element.strip()) for element in elements]

                # Get the name of the parent component and corresponding line
                parent_name = parent_dictionary[depth - 2].keys()[-1] if depth else ""
                parent_line = self.new_dicts_index[parent_name] if depth else -1

                # If the line is at least composed of a keyname and column
                if key_name and column:
                    # If a value is provided
                    if value:
                        parent_dictionary[depth - 1][split_line[0]] = self.to_format(split_line[2])
                    # If a condensed dictionary is provided
                    elif condensed_dict:
                        parent_dictionary[depth - 1][split_line[0]] = checked_condensed_dict
                    elif condensed_list:
                        parent_dictionary[depth - 1][split_line[0]] = checked_condensed_list
                    # If no value is provided then create a new empty dictionary that is going to be filled by elements
                    # coming from higher depths
                    else:
                        new_empty_dict = OrderedDict()
                        parent_dictionary[depth - 1][split_line[0]] = new_empty_dict
                        parent_dictionary[depth] = new_empty_dict

                # If line corresponds to a list
                elif dash:
                    # Get the object in which we should add the list
                    object_to_fill = parent_dictionary[depth - 2].values()[-1]
                    # If some text is provided after the dash
                    if list_element:
                        element = list_element.strip()
                        # look for a potential one element dictionary such as - a : b
                        one_elem_dict = re.search("([^\:\s]*)\s?:\s?([^\:\s]*)", element)
                        # If the list_element starts with a space or the one element search found something incomplete
                        # make the line wrong and go the next one
                        is_list_wrong = list_element.startswith(" ") or "," in list_element
                        if is_list_wrong or (one_elem_dict and not all(one_elem_dict.groups())):
                            self.wrong_format_lines.append(line_number)
                            line_number += 1
                            continue
                        # Otherwise process it
                        # Get either the simple element of the list or create a one element dictionary
                        if one_elem_dict:
                            key_, value_ = one_elem_dict.groups()
                            formatted_elems = [(self.to_format(key_), self.to_format(value_))]
                        element_to_add = OrderedDict(formatted_elems) if one_elem_dict else self.to_format(element)
                    # If a condensed dict is provided
                    elif condensed_dict:
                        element_to_add = checked_condensed_dict
                    # If a condensed list
                    elif condensed_list:
                        element_to_add = checked_condensed_list
                    # If a list already exists for the given parent then append the element
                    if isinstance(object_to_fill, list):
                        object_to_fill.append(element_to_add)
                    # Otherwise turns it to a list with the element inside
                    else:
                        parent_dictionary[depth - 2][parent_name] = [element_to_add]
                        # Delete the dictionary that was created in the parents before hand
                        del parent_dictionary[depth - 1]

                # Remove the parent line as wrong if possible
                if parent_line in self.wrong_format_lines:
                    self.wrong_format_lines.remove(parent_line)
                line_number += 1
        # Get the parsed content
        self.parsed_content = parsed
        # Emit the singal if it's different than the initial
        self.contentIsModified.emit(self.initial_content != self.parsed_content)

    def reset_init_content(self):
        """
            Reset the initial content
        """
        self.initial_content = copy.deepcopy(self.parsed_content) if self.parsed_content else OrderedDict()

    def mark_component(self, component_name):
        """
            Mark all lines belonging to dictionary named component_name as wrongly formatted
        """
        # Get which slice it is
        slice_index = 0
        for slice in self.slices:
            if slice[0].startswith(component_name):
                break
            slice_index += 1
        # Get the starting line
        begin_line = 0
        # If the element is part of new_dicts_index then get the beginning line from the dict
        potential_key = self.slices[slice_index][0].strip(":").strip() if slice_index < len(self.slices) else -1
        if potential_key in self.new_dicts_index:
            begin_line = self.new_dicts_index[potential_key]
        # Otherwise it means it is a "complete" element and needs to get he beginning line
        else:
            for i in range(slice_index):
                begin_line += len(self.slices[i])
        # Remove all trailing spaces of each element of the slice
        striped_slice = list(map(lambda x: x.strip(), self.slices[slice_index]))
        # Get the ending line
        end_line = begin_line + len(self.slices[slice_index]) - striped_slice.count("")
        # For all these lines mark them as bad
        lines_indices = range(begin_line, end_line)
        for line_index in lines_indices:
            if line_index not in self.wrong_format_lines:
                self.wrong_format_lines.append(line_index)
        self.update_background()

    def set_margin_marker(self):
        """
            Make a symbol appears in the margin of the editor and update it when text is typed
        """
        self.textChanged.connect(self.display_margin_marker)

    def display_margin_marker(self):
        """
            Update the margin marker allowing to add components
        """
        self.markerDeleteAll(1)
        self.markerAdd(0, 1)

    def reset(self):
        """
            Clean the editor (i.e. remove content and reset attributes) but keep is editable
        """
        super(YamlCodeEditor, self).reset()
        self.parsed_content = OrderedDict()


class XmlCodeEditor(GenericCodeEditor):

    """
        QScintilla-based widget allowing to create a XML code editor
    """

    def __init__(self, parent=None):
        """
            Initialize the class by setting up the editor

            @param parent: parent of the widget
        """
        super(XmlCodeEditor, self).__init__(parent)
        self.lexer_ = Qsci.QsciLexerXML(self)
        self.initial_content = None
        self.parsed_content = None

    def parse_content(self):
        """
            Parse the XML file to capture correctly formatted arguments
        """
        self.wrong_format_lines = list()
        editor_content = self.text()

        if not editor_content:
            self.parsed_content = None
            self.contentIsModified.emit(self.initial_content != self.parsed_content)
            return

        raw_arguments = re.search("\<include file=.*?\>(.*?)\<\/include\>", editor_content, re.DOTALL)
        if raw_arguments is None:
            self.parsed_content = None
            self.contentIsModified.emit(self.initial_content != self.parsed_content)
            return

        raw_arguments = re.sub("<!-- You can add any options you want to the file -->", "", raw_arguments.group(1))
        # Strip is used to remove possible spaces at the head and tail of the string
        arguments_list = re.split("\n", raw_arguments.strip())
        filtered_arguments = [x.strip() for x in arguments_list if x]

        editor_list = re.split("\n", editor_content.strip())
        filtered_editor = [x.strip() for x in editor_list if x]

        self.parsed_content = list()

        for argument in filtered_arguments:
            template_search = re.search("\<arg name\s?=\s?(.*?) value\s?=\s?(.*?)\s?\/\>", argument)
            if template_search is None:
                self.wrong_format_lines.append(filtered_editor.index(argument))
            else:
                self.parsed_content.append(argument)

        self.contentIsModified.emit(self.initial_content != self.parsed_content)

    def update_background(self):
        """
            Update the markers based on which lines are detected as wrong
        """
        super(XmlCodeEditor, self).update_background()
        # In case the editor's content in empty notifies that something is wrong
        if self.parsed_content is None and self.isEnabled():
            for line in range(self.lines()):
                self.markerAdd(line, 0)

    def reinitialize(self):
        """
            Set the editor to its initial state
        """
        self.clear()
        self.setReadOnly(True)
        self.markerDeleteAll()
