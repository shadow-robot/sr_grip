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

from typing import Type, List
import pytest
from grip_api.abstract_widgets.base_text_editor import BaseTextEditor
from grip_api.config_widgets.code_editors import XmlCodeEditor

# Need to add these pylint disabler due to the nature of pytest fixtures and the mandatory qt's plugin fixture qtbot
# pylint: disable=W0621, W0613


@pytest.fixture
def xml_editor(qtbot):
    """
        Fixture returning a XML code editor

        @param qtbot: Fixture used to enable the library to run on Qt-based objects
    """
    return XmlCodeEditor()


def test_editor_initialization(xml_editor) -> None:
    """
        Make sure new instances of the class inherit from the parent class as the latter implements the core UI

        @param xml_editor: Fixture previously defined providing the method with an instance of the object to be tested
    """
    assert isinstance(xml_editor, BaseTextEditor)


def set_text_and_check_result(xml_code_editor: Type[XmlCodeEditor], text: str, wrong_line_indices: List[int]) -> None:
    """
        Convenience function that gathers the core, i.e. setting text to an editor and checking that some of its
        attributes are properly set

        @param yaml_code_editor: Instance of a YamlCodeEditor that is the subject of the test
        @param text: Text to set to the code editor and that will be parsed
        @param wrong_line_indices: List of the indices of the lines wrongly formatted
        @param parsed_text_as_dict: Dictionary only containing the parsed version of the properly formatted components
    """
    xml_code_editor.set_text_and_trigger_checks(text)
    # Get number of lines
    number_lines = xml_code_editor.lines()
    # Make sure the lines detected as wrongly formatted are correct
    assert xml_code_editor.wrong_format_lines == wrong_line_indices
    # Make sure only the background of the badly formatted lines has been properly updated
    assert [line for line in range(number_lines) if xml_code_editor.markersAtLine(line)] == wrong_line_indices


@pytest.mark.parametrize("input_text, wrong_lines", [("Random text", [0]), ("Random\ntext", [0, 1]),
                                                     ("key: value", [0]), ("3.5", [0])])
def test_editor_parsing_invalid_text(xml_editor, input_text: str, wrong_lines: List[int]) -> None:
    """
        Test that XML code editors reject all kind of text that is not placed inside the expected tags

        @param xml_editor: Fixture previously defined providing the method with an instance of the object to be tested
        @param input_text: Text to set to the code editor and that will be parsed
        @param wrong_lines: List of the indices of the lines wrongly formatted
    """
    set_text_and_check_result(xml_editor, input_text, wrong_lines)


@pytest.mark.parametrize("input_text, wrong_lines",
                         [('<include file="$(find ur10e_moveit_config)/launch/move_group.launch">\n</include>', []),
                          ('<include file="$(find ur10e_moveit_config)/launch/move_group.launch"/>', [])])
def test_editor_parsing_valid_file(xml_editor, input_text: str, wrong_lines: List[int]) -> None:
    """
        Test that XML code editors can properly parse files to be included

        @param xml_editor: Fixture previously defined providing the method with an instance of the object to be tested
        @param input_text: Text to set to the code editor and that will be parsed
        @param wrong_lines: List of the indices of the lines wrongly formatted
    """
    set_text_and_check_result(xml_editor, input_text, wrong_lines)


@pytest.mark.parametrize("input_text, wrong_lines",
                         [('<include file="$(find ur10e_moveit_config)/launch/move_group.launch">\n  '
                           '<!-- This is a comment that should not be triggering any issue -->\n</include>', []),
                          ('<include file="$(find ur10e_moveit_config)/launch/move_group.launch">\n  This is invalid'
                           '\n</include>', [1]),
                          ('<include file="$(find ur10e_moveit_config)/launch/move_group.launch">\n  '
                           '<arg name=test value=5>\n</include>', [1]),
                          ('<include file="$(find ur10e_moveit_config)/launch/move_group.launch">\n  '
                           '<arg name=test value=5/>\n</include>', [])])
def test_editor_parsing_arguments(xml_editor, input_text: str, wrong_lines: List[int]) -> None:
    """
        Test that XML code editors can properly parse arguments specified in the context of an included file

        @param xml_editor: Fixture previously defined providing the method with an instance of the object to be tested
        @param input_text: Text to set to the code editor and that will be parsed
        @param wrong_lines: List of the indices of the lines wrongly formatted
    """
    set_text_and_check_result(xml_editor, input_text, wrong_lines)
