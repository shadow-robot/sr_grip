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

from PyQt5.QtWidgets import QGraphicsProxyWidget


class GraphicsStateContent(QGraphicsProxyWidget):

    """
        QGraphicsProxyWidget that displays the content of the states
    """

    def __init__(self, widget, parent=None):
        """
            Initialize the widget

            @param widget: QWidget to be dispalyed in the view
            @param parent: Parent of this widget
        """
        super(GraphicsStateContent, self).__init__(parent=parent)
        # Store the parent in here
        self.parent = parent
        # Set the widget
        self.setWidget(widget)
        # Connect the signal coming from the parent stating that the widget's view should be changed
        self.parent.collapseBox.connect(self.update_visible)

    def update_visible(self, make_visible):
        """
            Make the widget visible or non visible depending on the value sent by the signal

            @param make_visible: Boolean signaling whether the widget should be visible or not
        """
        self.setVisible(make_visible)
