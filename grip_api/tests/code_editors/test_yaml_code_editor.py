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


from typing import Type, List, Dict, Union
import pytest
from grip_api.abstract_widgets.base_text_editor import BaseTextEditor
from grip_api.config_widgets.code_editors import YamlCodeEditor

# Need to add these pylint disabler due to the nature of pytest fixtures and the mandatory qt's plugin fixture qtbot
# pylint: disable=W0621, W0613


@pytest.fixture
def yaml_editor(qtbot):
    """
        Fixture returning a YAML code editor

        @param qtbot: Fixture used to enable the library to run on Qt-based objects
    """
    return YamlCodeEditor()


def test_editor_initialization(yaml_editor) -> None:
    """
        Make sure new instances of the class inherit from the parent class as the latter implements the core UI

        @param yaml_editor: Fixture previously defined providing the method with an instance of the object to be tested
    """
    assert isinstance(yaml_editor, BaseTextEditor)


def set_text_and_check_parsing_result(yaml_code_editor: Type[YamlCodeEditor], text: str, wrong_line_indices: List[int],
                                      parsed_text_as_dict: Dict[str, Union[int, float, bool, str, List, Dict]]) -> None:
    """
        Convenience function that gathers the core, i.e. setting text to an editor and checking that some of its
        attributes are properly set

        @param yaml_code_editor: Instance of a YamlCodeEditor that is the subject of the test
        @param text: Text to set to the code editor and that will be parsed
        @param wrong_line_indices: List of the indices of the lines wrongly formatted
        @param parsed_text_as_dict: Dictionary only containing the parsed version of the properly formatted components
    """
    yaml_code_editor.set_text_and_trigger_checks(text)
    # Get number of lines
    number_lines = yaml_code_editor.lines()
    # Make sure the parsed content is the one expected
    assert yaml_code_editor.parsed_content == parsed_text_as_dict
    # Make sure the lines detected as wrongly formatted are correct
    assert yaml_code_editor.wrong_format_lines == wrong_line_indices
    # Make sure only the background of the badly formatted lines has been properly updated
    assert [line for line in range(number_lines) if yaml_code_editor.markersAtLine(line)] == wrong_line_indices


@pytest.mark.parametrize("input_text, wrong_lines, parsed_dict",
                         [("test", [0], {}), ("-5.4", [0], {}), ("3", [0], {}), ("{key: value}", [0], {}),
                          ("[G, 1, -8.3, True, false]", [0], {})])
def test_editor_parsing_single_element(yaml_editor, input_text: str, wrong_lines: List[int],
                                       parsed_dict: Dict[str, Union[int, float, bool, str, List, Dict]]) -> None:
    """
        Test the be behavior of the editor when a single-line element is set as text

        @param yaml_editor: Fixture previously defined providing the method with an instance of the object to be tested
        @param input_text: Text to set to the code editor and that will be parsed
        @param wrong_lines: List of the indices of the lines wrongly formatted
        @param parsed_dict: Dictionary only containing the parsed version of the properly formatted components
    """
    set_text_and_check_parsing_result(yaml_editor, input_text, wrong_lines, parsed_dict)


@pytest.mark.parametrize("input_text, wrong_lines, parsed_dict",
                         [("\ntest", [1], {}), ("\ntest\n", [1], {}), ("test\n", [0], {}), ("This \ntest", [0, 1], {}),
                          ("-5.4", [0], {}), ("-4\n.3", [0, 1], {}), ("-4.\n3", [0, 1], {}), ("[G,\n-8.3]", [0, 1], {}),
                          ("[G, \n-8.3]", [0, 1], {}), ("[\nG, -8.3]", [0, 1], {}), ("[G, -8.3\n]", [0, 1], {}),
                          ("{key:\nvalue}", [0, 1], {}), ("{key\n:value}", [0, 1], {}), ("{\nkey:value}", [0, 1], {}),
                          ("{key:value\n}", [0, 1], {})])
def test_editor_parsing_multi_line(yaml_editor, input_text: str, wrong_lines: List[int],
                                   parsed_dict: Dict[str, Union[int, float, bool, str, List, Dict]]) -> None:
    """
        Test the be behavior of the editor when parsing multi-line elements

        @param yaml_editor: Fixture previously defined providing the method with an instance of the object to be tested
        @param input_text: Text to set to the code editor and that will be parsed
        @param wrong_lines: List of the indices of the lines wrongly formatted
        @param parsed_dict: Dictionary only containing the parsed version of the properly formatted components
    """
    set_text_and_check_parsing_result(yaml_editor, input_text, wrong_lines, parsed_dict)


@pytest.mark.parametrize("input_text, wrong_lines, parsed_dict",
                         [("key: string_value", [], {"key": "string_value"}), ("key: -1.75", [], {"key": -1.75}),
                          ("key: 104.8", [], {"key": 104.8}), ("key: -3", [], {"key": -3}), ("key: 2", [], {"key": 2}),
                          ("key: True", [], {"key": True}), ("key: true", [], {"key": True}),
                          ("key: false", [], {"key": False}), ("key: False", [], {"key": False})
                          ])
def test_editor_parsing_format_value(yaml_editor, input_text: str, wrong_lines: List[int],
                                   parsed_dict: Dict[str, Union[int, float, bool, str, List, Dict]]) -> None:
    """
        Test whether the editor is capable of correctly parsing non-condensed values in dictionaries

        @param yaml_editor: Fixture previously defined providing the method with an instance of the object to be tested
        @param input_text: Text to set to the code editor and that will be parsed
        @param wrong_lines: List of the indices of the lines wrongly formatted
        @param parsed_dict: Dictionary only containing the parsed version of the properly formatted components
    """
    set_text_and_check_parsing_result(yaml_editor, input_text, wrong_lines, parsed_dict)


@pytest.mark.parametrize("input_text, wrong_lines, parsed_dict",
                         [("key: value", [], {"key": "value"}), ("3: value", [], {"3": "value"}),
                          ("True: value", [], {"True": "value"}), ("-1.4: value", [], {"-1.4": "value"}),
                          ("[1, a]: value", [], {"[1, a]": "value"})])
def test_editor_parsing_format_key(yaml_editor, input_text: str, wrong_lines: List[int],
                                   parsed_dict: Dict[str, Union[int, float, bool, str, List, Dict]]) -> None:
    """
        Test that code editors correctly parse keys of dictionaries, i.e. in our specific use case, cast them as strings

        @param yaml_editor: Fixture previously defined providing the method with an instance of the object to be tested
        @param input_text: Text to set to the code editor and that will be parsed
        @param wrong_lines: List of the indices of the lines wrongly formatted
        @param parsed_dict: Dictionary only containing the parsed version of the properly formatted components
    """
    set_text_and_check_parsing_result(yaml_editor, input_text, wrong_lines, parsed_dict)


@pytest.mark.parametrize("input_text, wrong_lines, parsed_dict",
                         [("- first_element\n- second_element", [0, 1], {}), ("-number_one\n-number_two", [0, 1], {}),
                          ("-number_one\n- 8", [0, 1], {})])
def test_editor_parsing_root_explicit_lists(yaml_editor, input_text: str, wrong_lines: List[int],
                                   parsed_dict: Dict[str, Union[int, float, bool, str, List, Dict]]) -> None:
    """
        Test if top level (i.e. root) lists come out as wrongly formatted (as they should) since not mapped to a key

        @param yaml_editor: Fixture previously defined providing the method with an instance of the object to be tested
        @param input_text: Text to set to the code editor and that will be parsed
        @param wrong_lines: List of the indices of the lines wrongly formatted
        @param parsed_dict: Dictionary only containing the parsed version of the properly formatted components
    """
    set_text_and_check_parsing_result(yaml_editor, input_text, wrong_lines, parsed_dict)


@pytest.mark.parametrize("input_text, wrong_lines, parsed_dict",
                         [("- key: value", [0], {}), ("- [True, 1, test]", [0], {})])
def test_editor_parsing_root_list_condensed(yaml_editor, input_text: str, wrong_lines: List[int],
                                            parsed_dict: Dict[str, Union[int, float, bool, str, List, Dict]]) -> None:
    """
        Test if top level (i.e. root) lists come out as wrongly formatted (as they should) when provided with condensed
        elements

        @param yaml_editor: Fixture previously defined providing the method with an instance of the object to be tested
        @param input_text: Text to set to the code editor and that will be parsed
        @param wrong_lines: List of the indices of the lines wrongly formatted
        @param parsed_dict: Dictionary only containing the parsed version of the properly formatted components
    """
    set_text_and_check_parsing_result(yaml_editor, input_text, wrong_lines, parsed_dict)


@pytest.mark.parametrize("input_text, wrong_lines, parsed_dict",
                         [("# The first line is a comment\nBut the second one is not", [1], {}),
                          ("# If the whole text is\n# made of comments", [], {}),
                          ("key: 12 # Adding a comment to a valid line must not change its parsing", [], {"key": 12}),
                          ("\ninvalid_line # A comment should not make an non valid line valid", [1], {})])
def test_editor_parsing_comments(yaml_editor, input_text: str, wrong_lines: List[int],
                                 parsed_dict: Dict[str, Union[int, float, bool, str, List, Dict]]) -> None:
    """
        Test that commented lines don't throw the parser off, i.e. don't change its behavior

        @param yaml_editor: Fixture previously defined providing the method with an instance of the object to be tested
        @param input_text: Text to set to the code editor and that will be parsed
        @param wrong_lines: List of the indices of the lines wrongly formatted
        @param parsed_dict: Dictionary only containing the parsed version of the properly formatted components
    """
    set_text_and_check_parsing_result(yaml_editor, input_text, wrong_lines, parsed_dict)


@pytest.mark.parametrize("input_text, wrong_lines, parsed_dict",
                         [("key: [1, True, -8.5, value]", [], {"key": [1, True, -8.5, "value"]}),
                          ("key: [1, True,\n-8.5, value]", [0, 1], {}),
                          ("key:\n  [1, True, -8.5, value]", [0, 1], {"key": {}}),
                          ("key:\n  -1\n  -True\n  --8.5\n  -value", [], {"key": [1, True, -8.5, "value"]}),
                          ("key:\n  - 1\n  - True\n  - -8.5\n  - value", [], {"key": [1, True, -8.5, "value"]})])
def test_editor_parsing_lists(yaml_editor, input_text: str, wrong_lines: List[int],
                              parsed_dict: Dict[str, Union[int, float, bool, str, List, Dict]]) -> None:
    """
        Test that lists are properly handled when set as values of a dictionary

        @param yaml_editor: Fixture previously defined providing the method with an instance of the object to be tested
        @param input_text: Text to set to the code editor and that will be parsed
        @param wrong_lines: List of the indices of the lines wrongly formatted
        @param parsed_dict: Dictionary only containing the parsed version of the properly formatted components
    """
    set_text_and_check_parsing_result(yaml_editor, input_text, wrong_lines, parsed_dict)


@pytest.mark.parametrize("input_text, wrong_lines, parsed_dict",
                         [("key: {subkey1 : value, subkey2 : -1.5, subkey3 : False , sub: [-1, 2.6, True], -8: test}",
                           [], {"key": {"subkey1": "value", "subkey2": -1.5, "subkey3": False, "sub": [-1, 2.6, True],
                                        -8: "test"}}),
                          ("key:\n  subkey1: value\n  subkey2: -1.5\n  subkey3: False\n  sub: [-1, 2.6, True]\n"
                           "  -8: test", [], {"key": {"subkey1": "value", "subkey2": -1.5, "subkey3": False,
                                                      "sub": [-1, 2.6, True], -8: "test"}}),
                          ("key:\n  subkey1: value\n  subkey2: -1.5\nsubkey3: False\n  sub: [-1, 2.6, True]\n"
                           "  -8: test", [4, 5], {"key": {"subkey1": "value", "subkey2": -1.5}, "subkey3": False}),
                          ("key:\n subkey1 : value\n  subkey2 : -1.5\n  subkey3 : False\n  sub: [-1, 2.6, True]\n"
                           "  8: test", [0, 1, 2, 3, 4, 5], {}),
                          ("key: {subkey1 : value subkey2 : -1.5, subkey3 : False , sub: [-1, 2.6, True], -8: test}",
                           [0], {})])
def test_editor_parsing_dictionaries(yaml_editor, input_text: str, wrong_lines: List[int],
                              parsed_dict: Dict[str, Union[int, float, bool, str, List, Dict]]) -> None:
    """
        Test that dictionary parsing works as expected, especially with wrong indentations or wrong format of the keys

        @param yaml_editor: Fixture previously defined providing the method with an instance of the object to be tested
        @param input_text: Text to set to the code editor and that will be parsed
        @param wrong_lines: List of the indices of the lines wrongly formatted
        @param parsed_dict: Dictionary only containing the parsed version of the properly formatted components
    """
    set_text_and_check_parsing_result(yaml_editor, input_text, wrong_lines, parsed_dict)


@pytest.mark.parametrize("pre_reset_text", ["Invalid text", "key: -1.5"])
@pytest.mark.parametrize("post_reset_text, wrong_lines, parsed_dict",
                         [("Invalid should remain invalid", [0], {}),
                          ("valid_key: False", [], {"valid_key": False})])
def test_editor_parsing_after_reset(yaml_editor, pre_reset_text: str, post_reset_text: str, wrong_lines: List[int],
                                    parsed_dict: Dict[str, Union[int, float, bool, str, List, Dict]]) -> None:
    """
        Method ensuring that removing text does not mess with the parsing of any potential new text

        @param yaml_editor: Fixture previously defined providing the method with an instance of the object to be tested
        @param pre_reset_text: Text set before the code editor goes through a reset
        @param post_reset_text: Text set after the code editor went through a reset
        @param wrong_lines: List of the indices of the lines wrongly formatted
        @param parsed_dict: Dictionary only containing the parsed version of the properly formatted components
    """
    yaml_editor.set_text_and_trigger_checks(pre_reset_text)
    yaml_editor.remove_text()
    set_text_and_check_parsing_result(yaml_editor, post_reset_text, wrong_lines, parsed_dict)
