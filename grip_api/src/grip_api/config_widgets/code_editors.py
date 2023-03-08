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

import re
import copy
from PyQt5 import Qsci
from grip_api.abstract_widgets.base_text_editor import BaseTextEditor


class YamlCodeEditor(BaseTextEditor):

    """
        QScintilla-based YAML code editor
    """

    def __init__(self):
        """
            Initialize the class by setting up the editor
        """
        super().__init__()
        self._initialize_margin()
        # Store the lexer to be used
        self.text_lexer = Qsci.QsciLexerYAML(self)
        # Will contain the parsed content
        self.parsed_content = {}
        # Each list of this list will contain the lines that are supposed to correspond to a top-level component
        self._sliced_root_components = []
        # List that will contain the index of each line wrongly formatted
        self.wrong_format_lines = []
        # Dictionary that contains the newly parsed dictionaries
        self._new_dict_lines = {}

    def _initialize_margin(self):
        """
            Initialize the margin with a symbol, that if clicked, helps with the definition of a new entry in the editor
        """
        # Give the ability to set symbols in the margin
        self.setMarginType(1, Qsci.QsciScintilla.SymbolMargin)
        # Make sure the margin does not become too large
        self.setMarginWidth(1, "00")
        # Define a plus marker, which is index with 1 since index 0 is used to highlight wrongly formatted lines
        self.markerDefine(Qsci.QsciScintilla.Plus, 1)
        # Make the margin clickable
        self.setMarginSensitivity(1, True)

    def set_autocompletion(self, words_to_autocomplete):
        """
            Allow the strings contained in the input list to be autocompleted after two characters

            @param words_to_autocomplete: List of strings corresponding to the words to be proposed for autocompletion
        """
        self.setAutoCompletionSource(Qsci.QsciScintilla.AcsAPIs)
        self.setAutoCompletionThreshold(2)
        api = Qsci.QsciAPIs(self.text_lexer)
        for word in words_to_autocomplete:
            api.add(word)
        # Need to call this method so that the added words get accounted for
        api.prepare()

    def turn_off_autocompletion(self):
        """
            Turn off autocompletion
        """
        self.setAutoCompletionSource(Qsci.QsciScintilla.AcsNone)

    def _parse_content(self):  # pylint: disable=R0912, R0914, R0915
        """
            Parse the editor's content and store ONLY the valid content in the parsed_content attribute
        """
        # Reinitialize the list of wrong lines
        self.wrong_format_lines = []
        editor_content = self.text()
        # If there's no text inside the editor, we can just stop the parsing here
        if not editor_content:
            self.parsed_content = {}
            # Emit a signal re. whether the text has been modified or not
            self.contentIsModified.emit(self._initial_content != self.parsed_content)
            return
        # Split the text line by line
        split_content = editor_content.split("\n")
        # Get indices of all lines starting without any space or \n (so lines potentially defining a new root component)
        zero_depth_indices = [line_index for line_index, line_value in enumerate(split_content)
                              if re.search(r"^(\w+)", line_value) is not None]
        # If nothing has been found (meaning that all the content is wrong), set all the lines in a single list
        if not zero_depth_indices:
            self._sliced_root_components = [split_content]
        # If at least one potential root component is found, create slices of lines that are supposed to correspond to
        # each component
        else:
            # Condition ensuring we don't forget any text that could be written above the first candidate root component
            self._sliced_root_components = [split_content[:zero_depth_indices[0]]] if zero_depth_indices[0] else []
            # For each potential root component, extract the following lines, up to a potential new root component
            self._sliced_root_components += [split_content[zero_depth_indices[i]:zero_depth_indices[i + 1]]
                                             for i in range(len(zero_depth_indices) - 1)]
            # Take all the lines after the last potential root component
            self._sliced_root_components += [split_content[zero_depth_indices[-1]:]]
        # Current line number
        line_number = 0
        # Will contain the YAML parsed content of all valid root components of the editor
        parsed_and_valid = {}
        # the following dictionary is used to recursively fill the other dictionaries.
        # Keys correspond to the depth of the parent of the current element and values correspond to the container of
        # such elements
        parent_dictionary = {-1: parsed_and_valid}
        # Make sure to start from a clean and empty dictionary
        # Make sure it includes of course the indices previously computed but also the dicts created at higher levels
        self._new_dict_lines = {}
        # For each slice corresponding to a root component (depth = 0)
        for slice_ in self._sliced_root_components:
            # Expected depth is used to detect wrong indentation
            expected_depth = 0
            # For each line of the given slice
            for line in slice_:
                # Search for spaces at the beginning of the line
                starting_spaces = re.search(r"(^\s{1,})", line)
                # Get the number of spaces out of the search
                number_space = len(starting_spaces.group(1)) if starting_spaces else 0
                # To be valid the number of space should be even. In other word, incrementing the depth by one is
                # equivalent to inserting two spaces. If it's not the case set depth to a higher value
                depth = number_space / 2 if number_space % 2 == 0 else expected_depth + 1
                # If depth is higher than expected_depth it means that the indentation is wrong
                if depth > expected_depth and line.strip():
                    self.wrong_format_lines.append(line_number)
                    line_number += 1
                    continue
                # If depth is smaller than the expected one, update the latter to detect potential future wrong depth
                if depth < expected_depth and depth:
                    expected_depth = depth
                # Use a regex to extract all kind of information that can be part of a line without any trailing space
                split_line = re.search(r"([^\#\:\s\-\{\}]*)(\s?\:\s?)?(?(2)([^\[\{\:\s\#]*)|)?(\-\s?)?"
                                       r"(?(4)([^\{\[\#]*)|)?(\{[^\#\[\{\(\]\}\)]*\})?(\[[^\#\:\[\{\]\}\(\)]*\])?"
                                       r"(\s*\#.*)?(.*)?", line.strip()).groups()
                # Unstack all the information contained in the line into different variables
                key_name, column, value, dash, list_element, condensed_dict, condensed_list, comment, trash = split_line
                # If there are unexpected stuff on the line then continue and go to the next line
                if trash:
                    self.wrong_format_lines.append(line_number)
                    line_number += 1
                    continue
                # Otherwise remove the last element as we know it's empty
                split_line = split_line[:-1]
                # If it's an empty line then skip to the next line
                if all(not element for element in split_line):
                    line_number += 1
                    continue
                # If we have only a comment on the line then go to the next line
                if comment and all(not element for element in split_line[:-1]):
                    line_number += 1
                    continue
                # If only text is present on the line (without :) then it is invalid
                if key_name and all(not element for element in split_line[1:-1]):
                    # Add the line to wrong format so that it appears in red-ish
                    self.wrong_format_lines.append(line_number)
                    line_number += 1
                    continue
                # If we have only a keyword and a column, mark the line as wrong as we don't know what's following
                # However get the corresponding line and record it into self._new_dict_lines
                if key_name and column and all(not element for element in split_line[2:-1]):
                    self.wrong_format_lines.append(line_number)
                    self._new_dict_lines[key_name] = line_number
                    # Since it might be the beginning of another sub-element, increment expected_depth
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

                # Parse potential condensed dictionary
                if condensed_dict:
                    content = re.search(r"\{(.*)\}", condensed_dict).group(1)
                    # Extract both keys and values from the dictionary as a list of tuples, i.e.
                    # [(key1, value1), (key2, value2), ...]
                    dict_args = re.findall(r"([^\:\s\{\,]*)\s?:\s?([^\:\s\}\,]*)", condensed_dict)
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
                    if any(not parsed_key or not parsed_value for parsed_key, parsed_value in dict_args):
                        self.wrong_format_lines.append(line_number)
                        line_number += 1
                        continue
                    # If we reach here, it means that the condensed dictionary is actually valid, and therefore store it
                    # in a dedicated dictionary
                    checked_condensed_dict = {self.to_format(parsed_key): self.to_format(parsed_value)
                                              for parsed_key, parsed_value in dict_args}

                # Parse potential condensed list
                if condensed_list:
                    content = re.search(r"\[(.*)\]", condensed_list).group(1)
                    elements = content.split(",") if content else []
                    checked_condensed_list = [self.to_format(element.strip()) for element in elements]

                # Get the name of the parent component and corresponding line
                parent_name = list(parent_dictionary[depth - 2])[-1] if depth else ""
                parent_line = self._new_dict_lines[parent_name] if depth else -1

                # If the line is at least composed of a key name and column
                if key_name and column:
                    # If a value is provided
                    if value:
                        parent_dictionary[depth - 1][key_name] = self.to_format(value)
                    # If a condensed dictionary is provided
                    elif condensed_dict:
                        parent_dictionary[depth - 1][key_name] = checked_condensed_dict
                    elif condensed_list:
                        parent_dictionary[depth - 1][key_name] = checked_condensed_list
                    # If no value is provided then create a new empty dictionary that is going to be filled by elements
                    # coming from higher depths
                    else:
                        new_empty_dict = {}
                        parent_dictionary[depth - 1][key_name] = new_empty_dict
                        parent_dictionary[depth] = new_empty_dict

                # If the line corresponds to a list
                elif dash:
                    # Get the object in which we should add the list
                    object_to_fill = list(
                        parent_dictionary[depth - 2].values())[-1]
                    # If some text is provided after the dash
                    if list_element:
                        element = list_element.strip()
                        # Look for a potential one element dictionary such as - a : b
                        one_elem_dict = re.search(r"([^\:\s]*)\s?:\s?([^\:\s]*)", element)
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
                            key, value_ = one_elem_dict.groups()
                            formatted_elements = [(self.to_format(key), self.to_format(value_))]
                        element_to_add = dict(formatted_elements) if one_elem_dict else self.to_format(element)
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
                        # Delete the dictionary that was created in the parents beforehand
                        del parent_dictionary[depth - 1]

                # Remove the parent line as wrong if possible
                if parent_line in self.wrong_format_lines:
                    self.wrong_format_lines.remove(parent_line)
                line_number += 1
        # Get the parsed content
        self.parsed_content = parsed_and_valid
        # Emit the signal if the parsed content is different than the initial
        self.contentIsModified.emit(self._initial_content != self.parsed_content)

    def reset_initial_content(self):
        """
            Reset the initial content
        """
        self._initial_content = copy.deepcopy(self.parsed_content) if self.parsed_content else {}

    def mark_component(self, component_name):
        """
            Mark all of the lines that belong to the component "component_name" as wrongly formatted
        """
        # Get the index of the list in which the component to be marked is
        slice_index = 0
        for sliced_root_lines in self._sliced_root_components:
            if sliced_root_lines[0].startswith(component_name):
                break
            slice_index += 1
        # Get the starting line
        begin_line = 0
        # If the element is part of new_dicts_index then get the beginning line from the dict
        if slice_index < len(self._sliced_root_components):
            potential_key = self._sliced_root_components[slice_index][0].strip(":").strip()
        else:
            potential_key = -1
        if potential_key in self._new_dict_lines:
            begin_line = self._new_dict_lines[potential_key]
        # Otherwise it means it is a complete element
        else:
            for index in range(slice_index):
                begin_line += len(self._sliced_root_components[index])
        # Remove all trailing spaces of each element of the slice
        striped_slice = list(map(lambda x: x.strip(), self._sliced_root_components[slice_index]))
        # Get the ending line
        end_line = begin_line + len(self._sliced_root_components[slice_index]) - striped_slice.count("")
        # For all these lines mark them as bad
        lines_indices = range(begin_line, end_line)
        for line_index in lines_indices:
            if line_index not in self.wrong_format_lines:
                self.wrong_format_lines.append(line_index)
        # Call an update of the background color (i.e to flag wrong lines as red)
        self._update_background()

    def set_margin_marker(self):
        """
            Make a symbol appears in the margin of the editor and update it when text is typed
        """
        self.textChanged.connect(self.display_margin_marker)

    def display_margin_marker(self):
        """
            Update the margin marker (the + symbol) that helps the user adding new elements to the editor
        """
        self.markerDeleteAll(1)
        self.markerAdd(0, 1)

    def remove_text(self):
        """
            Clear the editor (i.e. remove content and reset attributes) but keep is editable
        """
        super().remove_text()
        self.parsed_content = {}


class XmlCodeEditor(BaseTextEditor):

    """
        QScintilla-based XML code editor
    """

    def __init__(self):
        """
            Initialize the class by setting up the editor

            @param parent: parent of the widget
        """
        super().__init__()
        self.text_lexer = Qsci.QsciLexerXML(self)
        self._initial_content = None
        self.parsed_content = None
        self.wrong_format_lines = []

    def _parse_content(self):
        """
            Parse the XML file to capture correctly formatted arguments
        """
        self.wrong_format_lines = []
        editor_content = self.text()

        # If there's no text in the editor
        if not editor_content:
            self.parsed_content = None
            self.contentIsModified.emit(self._initial_content != self.parsed_content)
            return

        # As of now (03/23) the only XML editors used are for launch files so we are certain that the following
        # statement must be in the editor for the latter to be valid
        raw_arguments = re.search(r"\<include file=.*?\>(.*?)\<\/include\>", editor_content, re.DOTALL)
        # If no valid argument can be parsed from the above line
        if raw_arguments is None:
            self.parsed_content = None
            self.contentIsModified.emit(self._initial_content != self.parsed_content)
            return

        # Subtract the comment that is added to guide the user
        raw_arguments = re.sub("<!-- You can add any options you want to the file -->", "", raw_arguments.group(1))
        # Strip is used to remove possible spaces at the head and tail of the string
        arguments_list = re.split("\n", raw_arguments.strip())
        # Only extract non empty elements
        filtered_arguments = [argument.strip() for argument in arguments_list if argument]
        # Get all the lines (spaces are cleared) from the editors
        editor_list = re.split("\n", editor_content.strip())
        filtered_editor = [element.strip() for element in editor_list if element]

        self.parsed_content = []
        # For each argument that we have found
        for argument in filtered_arguments:
            # Parse the values from the known format that each argument should follow
            template_search = re.search(r"\<arg name\s?=\s?(.*?) value\s?=\s?(.*?)\s?\/\>", argument)
            if template_search is None:
                # Mark the line as wrong if nothing has been found
                self.wrong_format_lines.append(filtered_editor.index(argument))
            else:
                self.parsed_content.append(argument)
        self.contentIsModified.emit(self._initial_content != self.parsed_content)

    def _update_background(self):
        """
            Update the markers based on which lines are detected as wrong
        """
        super().update_background()
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
