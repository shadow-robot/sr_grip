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


import pytest
from grip_api.abstract_widgets.base_text_editor import BaseTextEditor
from grip_api.config_widgets.code_editors import YamlCodeEditor


@pytest.fixture
def yaml_editor(qtbot):
    """
        Fixture returning a YAML code editor
    """
    return YamlCodeEditor()


def test_editor_initialization(yaml_editor):
    """
        Make sure new instances of the object under test is valid if provided with different parameters
    """
    assert isinstance(yaml_editor, BaseTextEditor) and isinstance(yaml_editor, YamlCodeEditor)


def set_text_and_check_parsing_result(yaml_code_editor, text, wrong_line_indices, parsed_text_as_dict):
    """

    """
    yaml_code_editor.set_text_and_trigger_checks(text)
    assert yaml_code_editor.wrong_format_lines == wrong_line_indices
    assert yaml_code_editor.parsed_content == parsed_text_as_dict


@pytest.mark.parametrize("input_text,wrong_lines,parsed_dict",
                         [("test", [0], {}), ("\ntest", [1], {}), ("\ntest\n", [1], {}), ("test\n", [0], {}),
                          ("-5.4", [0], {}), ("3", [0], {})])
def test_editor_parsing_single_element(yaml_editor, input_text, wrong_lines, parsed_dict):
    """

    """
    set_text_and_check_parsing_result(yaml_editor, input_text, wrong_lines, parsed_dict)


def test_editor_parsing_multi_line_text(yaml_editor):
    """

    """
    set_text_and_check_parsing_result(yaml_editor, "This is a\nsingle line of text,\n  split over more than one line",
                                      [0, 1, 2], {})


@pytest.mark.parametrize("input_text,wrong_lines,parsed_dict",
                         [("key: string_value", [], {"key": "string_value"}), ("key: -1.75", [], {"key": -1.75}),
                          ("key: 104.8", [], {"key": 104.8}), ("key: -3", [], {"key": -3}), ("key: 2", [], {"key": 2}),
                          ("key: True", [], {"key": True}), ("key: true", [], {"key": True}),
                          ("key: false", [], {"key": False}), ("key: False", [], {"key": False})
                          ])
def test_editor_parsing_format_value(yaml_editor, input_text, wrong_lines, parsed_dict):
    """

    """
    set_text_and_check_parsing_result(yaml_editor, input_text, wrong_lines, parsed_dict)


@pytest.mark.parametrize("input_text,wrong_lines,parsed_dict",
                         [("key: value", [], {"key": "value"}), ("3: value", [], {"3": "value"}),
                          ("True: value", [], {"True": "value"}), ("-1.4: value", [], {"-1.4": "value"})])
def test_editor_parsing_format_key(yaml_editor, input_text, wrong_lines, parsed_dict):
    """

    """
    set_text_and_check_parsing_result(yaml_editor, input_text, wrong_lines, parsed_dict)


@pytest.mark.parametrize("input_text,wrong_lines,parsed_dict",
                         [("- first_element\n- second_element", [0, 1], {}), ("-number_one\n-number_two", [0, 1], {}),
                          ("-number_one\n- 8", [0, 1], {})])
def test_editor_parsing_root_explicit_lists(yaml_editor, input_text, wrong_lines, parsed_dict):
    """

    """
    set_text_and_check_parsing_result(yaml_editor, input_text, wrong_lines, parsed_dict)


@pytest.mark.parametrize("input_text,wrong_lines,parsed_dict",
                         [("[0, 1, a]", [0], {}), ("{key: value}", [0], {}), ("- key: value", [0], {}),
                          ("- [True, 1, test]", [0], {})])
def test_editor_parsing_root_condensed_elements(yaml_editor, input_text, wrong_lines, parsed_dict):
    """

    """
    set_text_and_check_parsing_result(yaml_editor, input_text, wrong_lines, parsed_dict)


@pytest.mark.parametrize("input_text,wrong_lines,parsed_dict",
                         [("# The first line is a comment\nBut the second one is not", [1], {}),
                          ("# If the whole text is\n# made of comments", [], {}),
                          ("key: 12 # Adding a comment to a valid line must not change its parsing", [], {"key": 12}),
                          ("\ninvalid_line # A comment should not make an non valid line valid", [1], {})])
def test_editor_parsing_comments(yaml_editor, input_text, wrong_lines, parsed_dict):
    """

    """
    set_text_and_check_parsing_result(yaml_editor, input_text, wrong_lines, parsed_dict)


@pytest.mark.parametrize("input_text,wrong_lines,parsed_dict",
                         [("key: [1, True, -8.5, value]", [], {"key": [1, True, -8.5, "value"]}),
                          ("key:\n  [1, True, -8.5, value]", [0, 1], {"key": {}}),
                          ("key:\n  -1\n  -True\n  --8.5\n  -value", [], {"key": [1, True, -8.5, "value"]}),
                          ("key:\n  - 1\n  - True\n  - -8.5\n  - value", [], {"key": [1, True, -8.5, "value"]})])
def test_editor_parsing_lists(yaml_editor, input_text, wrong_lines, parsed_dict):
    """

    """
    set_text_and_check_parsing_result(yaml_editor, input_text, wrong_lines, parsed_dict)


@pytest.mark.parametrize("input_text,wrong_lines,parsed_dict",
                         [("key: {subkey1 : value, subkey2 : -1.5, subkey3 : False , sub: [-1, 2.6, True], -8: test}",
                           [], {"key": {"subkey1": "value", "subkey2": -1.5, "subkey3": False, "sub": [-1, 2.6, True], -8: "test"}}),
                          ("key:\n  subkey1 : value\n  subkey2 : -1.5\n  subkey3 : False\n  sub: [-1, 2.6, True]\n  -8: test", [], {"key": {"subkey1": "value", "subkey2": -1.5, "subkey3": False, "sub": [-1, 2.6, True], -8: "test"}})])
def test_editor_parsing_dictionaries(yaml_editor, input_text, wrong_lines, parsed_dict):
    """

    """
    set_text_and_check_parsing_result(yaml_editor, input_text, wrong_lines, parsed_dict)

# Add same as above but with wrong indentations


@pytest.mark.parametrize("pre_reset_text", ["Invalid text", "key: -1.5"])
@pytest.mark.parametrize("post_reset_text,wrong_lines,parsed_dict",
                         [("Invalid should remain invalid", [0], {}),
                          ("valid_key: False", [], {"valid_key": False})])
def test_editor_parsing_after_reset(yaml_editor, pre_reset_text, post_reset_text, wrong_lines, parsed_dict):
    """

    """
    yaml_editor.set_text_and_trigger_checks(pre_reset_text)
    yaml_editor.remove_text()
    set_text_and_check_parsing_result(yaml_editor, post_reset_text, wrong_lines, parsed_dict)
