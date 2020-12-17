#!/usr/bin/env python

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


class Serializable(object):

    """
        Object that is going to be the base of most of the graphical editors widgets
    """

    def __init__(self):
        """
            Initialize the widget
        """
        self.id = id(self)

    def serialize(self):
        """
            Function allowing to save the content of a graphical editor widget
        """
        raise NotImplemented()

    def deserialize(self, data, hashmap={}):
        """
            Function allowing to load the content of a graphical editor widget

            @param data: Dictionary containign the data to load
            @param hashmap: Dictionary contianing the mapping betwwen id and objects
        """
        raise NotImplemented()
