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

from PyQt5.QtWidgets import QGraphicsItem, QGraphicsTextItem
from PyQt5.QtGui import QColor, QPen, QBrush, QFont
from PyQt5.QtCore import QRectF, Qt


class TerminalGraphicsSocket(QGraphicsItem):

    """
        Graphical representation of a terminal socket
    """

    def __init__(self, socket, parent=None):
        """
            Initialize the widget

            @param socket: Socket linked to this graphical representation
            @param parent: Parent of this widget
        """
        super(TerminalGraphicsSocket, self).__init__(parent=parent)
        self.socket = socket
        self.socket.container.get_view().viewScaled.connect(self.update_transform)
        # Check whether the socket is used as the input of a state or not
        self.is_input = not self.socket.name
        self.init_resources()
        self.init_ui()
        self.init_title()

    def update_transform(self, current_zoom):
        """
            Update the transform flag depending on the current zoom applied to the view

            @param current_zoom: Current zoom level applied to the view (integer)
        """
        self.setFlag(QGraphicsItem.ItemIgnoresTransformations, current_zoom < 0)

    def init_resources(self):
        """
            Define the resources that will be used to render the widget
        """
        # Radius of the socket
        self.radius = 10.0
        # Width of the outline
        self.outline_width = 2.0
        # Space between the socket and its name
        self.title_padding = 3.0
        # Set the font and color of the name
        self.title_color = Qt.white
        self.title_font = QFont("Ubuntu", 14)
        # Color for input sockets
        self.input_socket_color = QColor("#FF4599FF")
        # Color for outcomes
        self.socket_type_colors = [QColor("#FF00cb00"), QColor("#FFFF0021"), QColor("#FF0056a6"), QColor("#FFFF7700"),
                                   QColor("#FFa86db1"), QColor("#FFb54747")]
        # Get the color of the socket
        if self.socket.is_starting:
            self.socket_color = self.input_socket_color
        else:
            self.socket_color = self.socket_type_colors[self.socket.index]
        # Outline color
        self.color_outline = QColor("#FF000000")
        # Color of the outline when selected
        self.color_selected = QColor("#FFFFFFFF")
        # Define of the painter should draw the lines (= outlines)
        self.pen = QPen(self.color_outline)
        self.pen.setWidthF(self.outline_width)
        self.pen_selected = QPen(self.color_selected)
        self.pen_selected.setWidthF(self.outline_width)
        # Define the fill patern of the socket
        self.brush = QBrush(self.socket_color)

    def init_ui(self):
        """
            Initialize the UI of the widget
        """
        # Make the GraphicsSocket selectable and movable
        self.setFlag(QGraphicsItem.ItemIsSelectable)
        self.setFlag(QGraphicsItem.ItemIsMovable)
        self.setAcceptHoverEvents(True)

    def init_title(self):
        """
            Add a text below the terminal socket
        """
        self.title = QGraphicsTextItem(self)
        # Set the text
        self.title.setPlainText(self.socket.name)
        self.title.setDefaultTextColor(self.title_color)
        self.title.setFont(self.title_font)
        # Make sure the text takes the right amount of space
        self.title.adjustSize()
        # Center it below or above the socket
        # Below
        height_offset = self.radius + self.outline_width + self.title_padding
        # Above (need to add another offset corresponding to the title height)
        if self.socket.is_starting:
            height_offset = -height_offset - 30
        self.title.setPos(-self.title.textWidth() / 2., height_offset)

    def mouseMoveEvent(self, event):
        """
            Function triggered when this object is moved by the user

            @param event: QMouseEvent sent by PyQt5
        """
        super(TerminalGraphicsSocket, self).mouseMoveEvent(event)
        # Update all the connectors that are linked to this terminal socket
        for connector in self.socket.connectors:
            connector.update_positions()

    def paint(self, painter, QStyleOptionGraphicsItem, widget=None):
        """
            Paint the content of the terminal socket onto the QGraphicsView

            @param painter: QPainter that will render the widget
            @param QStyleOptionGraphicsItem: Options provividing style options for the item
            @param widget: Specify another QWidget on which this item will be painted on. Default to None
        """
        # Set the brush and pen
        painter.setBrush(self.brush)
        painter.setPen(self.pen if not self.isSelected() else self.pen_selected)
        # Paint the socket
        painter.drawEllipse(-self.radius, -self.radius, 2 * self.radius, 2 * self.radius)

    def boundingRect(self):
        """
            Return the bounding rectangle of the widget. Must be override to avoid issues when interacting with other
            widgets

            @return: QRectF containing the origin and bounding size of the circle (x,y,w,h)
        """
        return QRectF(- self.radius - self.outline_width, - self.radius - self.outline_width,
                      2 * (self.radius + self.outline_width), 2 * (self.radius + self.outline_width))
