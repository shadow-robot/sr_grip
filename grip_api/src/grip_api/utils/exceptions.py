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


from grip_api.utils.formatted_print import format_raise_string


class ParentWidgetTypeError(TypeError):
    """
        Custom error raised when the type of the input parent of a QWidget is not valid
    """

    def __init__(self, parent_type):
        """
            Customize the message output by the exception

            @param parent_type: Type of the parent that was attempted to be set to a QWidget
        """
        super().__init__(format_raise_string("The parent of a QWidget should either be 'None' or be another QWidget! "
                                             f"Provided type: {parent_type}"))
