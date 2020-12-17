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

from PyQt5.QtWidgets import QGraphicsItem
from PyQt5.QtGui import QColor, QPen, QBrush
from PyQt5.QtCore import QRectF


class GraphicsSocket(QGraphicsItem):

    """
        Graphical representation of sockets that are set on State objects
    """

    def __init__(self, socket):
        """
            Initialize the widget

            @param socket: Socket object this QGraphicsItem relates to
        """
        # Store the socket
        self.socket = socket
        # Set the parent as the state's graphcial state
        super(GraphicsSocket, self).__init__(socket.state.graphics_state)
        # Depending on the current view, update the transform applied to this object
        self.socket.state.container.get_view().viewScaled.connect(self.update_transform)
        # Initialize all constants for a nice rendering
        self.initialize_visu_tools()
        # Set the tool tip of the widget so we can see its name
        self.setToolTip(self.socket.name)
        self.update_transform(self.socket.state.container.get_view().current_zoom)

    def initialize_visu_tools(self):
        """
            Set all the attributes required to have a nice representation of the socket
        """
        # Radius of the socket
        self.radius = 8
        # Outline width of the socket
        self.outline_width = 1.0
        # For now, 6 different colours are set. Ifa state has more than 6 outcomes, would need to add some colors here
        self.colors = [
            QColor("#FF00cb00"),
            QColor("#FFFF0021"),
            QColor("#FF0056a6"),
            QColor("#FFFF7700"),
            QColor("#FFa86db1"),
            QColor("#FFb54747"),
        ]
        # Input sockets always have the same colour
        if self.socket.index == 0 and self.socket.count_on_this_side == 1:
            self.color_background = QColor("#FF4599FF")
        # Output sockets have their color set depending on their index
        else:
            self.color_background = self.colors[self.socket.index]
        # Outline will be painted in black
        self.color_outline = QColor("#FF000000")
        # Create a pen to draw the outline
        self.pen = QPen(self.color_outline)
        self.pen.setWidthF(self.outline_width)
        # Create the brush to colour the background
        self.brush = QBrush(self.color_background)

    def update_transform(self, current_zoom):
        """
            Update the transform flag depending on the current zoom applied to the view

            @param current_zoom: Current zoom level applied to the view (integer)
        """
        # If we are zooming out, thene ignores all tranforms to keep a constant size
        self.setFlag(QGraphicsItem.ItemIgnoresTransformations, current_zoom < 0)
        # Recompute the pose the sockets
        self.setPos(*self.socket.get_position())

    def paint(self, painter, QStyleOptionGraphicsItem, widget=None):
        """
            Paint the content of the socket onto the QGraphicsView

            @param painter: QPainter that will render the widget
            @param QStyleOptionGraphicsItem: Options provividing style options for the item
            @param widget: Specify another QWidget on which this item will be painted on. Default to None
        """
        # Paint a circle
        painter.setBrush(self.brush)
        painter.setPen(self.pen)
        painter.drawEllipse(-self.radius, -self.radius, 2 * self.radius, 2 * self.radius)

    def boundingRect(self):
        """
            Return the bounding rectangle of the widget. Must be overriden to avoid issues when interacting with other
            widgets

            @return: QRectF containing the origin and bounding size of the circle (x,y,w,h)
        """
        return QRectF(
            - self.radius - self.outline_width,
            - self.radius - self.outline_width,
            2 * (self.radius + self.outline_width),
            2 * (self.radius + self.outline_width),
        ).normalized()
