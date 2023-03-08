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


def get_color_formatted_string(text, color, message_type):
    """
        Return a string formatted in a way that it can be coloured when printed to a terminal

        @param text: String corresponding to the message to be printed
        @param color: Element from the Color enum
        @param message_type: String to display the type of message that will be displayed
        @return: Formatted string
    """
    if not isinstance(color, Color):
        raise TypeError(f"The input color {color} is not currently supported!")
    return f"{color.value}[{message_type}]: {text}{Color.COLOR_RESET.value}"


def print_error(text):
    """
        Print a red-coloured error message

        @param text: String corresponding to the message to be printed
    """
    print(get_color_formatted_string(text, Color.RED, "ERROR"))


def format_raise_string(text):
    """
        Return a coloured version of a given string so the user can directly understand a custom message associated to
        a raised error

        @param text: String corresponding to the message to be printed
    """
    return f"{Color.LIGHT_RED.value}{text}{Color.COLOR_RESET.value}"


def print_success(text):
    """
        Print a green-coloured error message

        @param text: String corresponding to the message to be printed
    """
    print(get_color_formatted_string(text, Color.GREEN, "SUCCESS"))


def print_progress(text):
    """
        Print a light cyan-coloured progress message

        @param text: String corresponding to the message to be printed
    """
    print(get_color_formatted_string(text, Color.LIGHT_CYAN, "PROGRESS"))


def print_warning(text):
    """
        Print a yellow-coloured warning message

        @param text: String corresponding to the message to be printed
    """
    print(get_color_formatted_string(text, Color.YELLOW, "WARNING"))


def interactive_dialog(text):
    """
        Print a light blue-coloured interactive input message

        @param text: String corresponding to the message to be printed
        @return: String corresponding to the user's input
    """
    response = input(get_color_formatted_string(text, Color.LIGHT_BLUE, "USER INPUT"))
    return response
