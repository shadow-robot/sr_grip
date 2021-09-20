#!/usr/bin/env python3

# Copyright 2020 Shadow Robot Company Ltd.
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

from PyQt5.QtWidgets import QMessageBox


def error_message(title, main_text, additional_text=None, parent=None):
    """
        Display an error message to the user on top of a parent widget

        @param title: Title of the message box
        @param main_text: Content of the message box
        @param additional_text: Additional text to display. Usually to specify why the error occurs.
        @param parent: Specify on top of which widget the message box should be spawned
    """
    error_message = QMessageBox(QMessageBox.Critical, title, main_text, parent=parent)
    if additional_text is not None:
        error_message.setInformativeText(additional_text)
    error_message.exec_()


def warning_message(title, main_text, additional_text=None, parent=None):
    """
        Display a warning message to the user on top of a parent widget

        @param title: Title of the message box
        @param main_text: Content of the message box
        @param additional_text: Additional text to display. Usually to specify why the warning occurs.
        @param parent: Specify on top of which widget the message box should be spawned
    """
    warning_message = QMessageBox(QMessageBox.Warning, title, main_text, parent=parent)
    if additional_text is not None:
        warning_message.setInformativeText(additional_text)
    warning_message.exec_()


def can_save_warning_message(title, main_text, additional_text=None, parent=None):
    """
        Display a waring message to the user on top of a parent widget to ask whether something needs to be saved

        @param title: Title of the message box
        @param main_text: Content of the message box
        @param additional_text: Additional text to display. Usually to specify why the warning occurs.
        @param parent: Specify on top which widget the message box should be spawned
        @return: True if "Save" is clicked, False if "Discard" is clicked and None otherwise
    """
    warning_message = QMessageBox(QMessageBox.Warning, title, main_text, parent=parent)
    if additional_text is not None:
        warning_message.setInformativeText(additional_text)
    warning_message.setStandardButtons(QMessageBox.Save | QMessageBox.Discard | QMessageBox.Cancel)
    choice = warning_message.exec_()
    if choice == QMessageBox.Cancel:
        return None
    return choice == QMessageBox.Save
