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

from enum import Enum


class Color(Enum):
    BLACK = '\033[30m'
    RED = '\033[31m'
    GREEN = '\033[32m'
    ORANGE = '\033[33m'
    BLUE = '\033[34m'
    PURPLE = '\033[35m'
    CYAN = '\033[36m'
    LIGHT_GREY = '\033[37m'
    DARK_GREY = '\033[90m'
    LIGHT_RED = '\033[91m'
    LIGHT_GREEN = '\033[92m'
    YELLOW = '\033[93m'
    LIGHT_BLUE = '\033[94m'
    PINK = '\033[95m'
    LIGHT_CYAN = '\033[96m'
    COLOR_RESET = '\033[0m'


def get_color_formatted_string(text, color: Color, message_type: str) -> str:
    """
        Return a coloured formatted string compatible with most terminals

        @param text: Text to be printed
        @param color: Element from the Color enum
        @param message_type: Text describing the type of message that will be displayed
        @return: Formatted string
    """
    if not isinstance(color, Color):
        raise TypeError(f"The input color {color} is not currently supported!")
    return f"{color.value}[{message_type}]: {text}{Color.COLOR_RESET.value}"


def print_error(text: str) -> None:
    """
        Print a red-coloured error message

        @param text: Text to be printed
    """
    print(get_color_formatted_string(text, Color.RED, "ERROR"))


def format_raise_string(text: str) -> str:
    """
        Return a coloured version of a given string so the user can directly understand that the given message relates
        to a raised error

        @param text: Text to be printed
        @return: Formatted string
    """
    return f"{Color.LIGHT_RED.value}{text}{Color.COLOR_RESET.value}"


def print_success(text: str) -> None:
    """
        Print a green-coloured error message

        @param text: Text to be printed
    """
    print(get_color_formatted_string(text, Color.GREEN, "SUCCESS"))


def print_progress(text: str) -> None:
    """
        Print a light cyan-coloured progress message

        @param text: Text to be printed
    """
    print(get_color_formatted_string(text, Color.LIGHT_CYAN, "PROGRESS"))


def print_warning(text: str) -> None:
    """
        Print a yellow-coloured warning message

        @param text: Text to be printed
    """
    print(get_color_formatted_string(text, Color.YELLOW, "WARNING"))


def interactive_dialog(text: str) -> str:
    """
        Return a light blue-coloured interactive input message

        @param text: Text to be printed
        @return: Formatted string corresponding to the user's input
    """
    response = input(get_color_formatted_string(text, Color.LIGHT_BLUE, "USER INPUT"))
    return response
