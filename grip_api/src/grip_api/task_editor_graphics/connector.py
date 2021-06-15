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

from PyQt5.QtWidgets import QGraphicsPathItem, QGraphicsItem
from PyQt5.QtCore import Qt, QPointF, QLineF
from PyQt5.QtGui import QColor, QPen, QPainterPath, QBrush, QPainterPathStroker


class GraphicsConnector(QGraphicsPathItem):

    """
        Graphical representation of a conenctor between two states
    """

    def __init__(self, connector, parent=None):
        """
            Initialize the widget

            @param connector: Connector object corresponding to this graphical representation
            @param parent: Parent of the widget
        """
        super(GraphicsConnector, self).__init__(parent)

        self.connector = connector
        # Initialize the source and destination positions (values set here are arbitrary)
        self.source_position = [0, 0]
        self.destination_position = [200, 100]
        self.init_visu_tools()
        self.init_ui()
        self.connector.container.get_view().viewScaled.connect(self.update_settings)

    def init_visu_tools(self):
        """
            Create the different attributes required to have a nice rendering
        """
        self.color = QColor("#001000")
        self.color_selected = QColor("#00ff00")
        self.pen = QPen(self.color)
        self.pen_selected = QPen(self.color_selected)
        self.pen_dragging = QPen(self.color)
        self.pen_dragging.setStyle(Qt.DashLine)
        self.pen_width = 3.0
        # Roundness factor of the computed curve. The higher the rounder
        self.roundness_factor = 0.25
        # Compute the compensation factor required to always have a connector with a consistent proportion wrt states
        self.view = self.connector.container.get_view()
        if self.view.current_zoom < -7:
                self.pen_width_compensator = self.view.zoom_in_multiplier**(-7 - self.view.current_zoom)
        else:
            self.pen_width_compensator = 1

    def init_ui(self):
        """
            Initialize the flags of the widget
        """
        # Make it selectable and make sure it goes under states if it collides with them
        self.setFlag(QGraphicsItem.ItemIsSelectable)
        self.setZValue(-1)

    def update_settings(self, zoom_value):
        """
            Update both the compensation factor for the width and the source and destination positions according to the
            new zoom

            @param zoom_value: Current zoom value of the view
        """
        if zoom_value < -7:
                self.pen_width_compensator = self.view.zoom_in_multiplier**(-7 - self.view.current_zoom)
        else:
            self.pen_width_compensator = 1
        self.connector.update_positions()

    def set_source(self, x, y):
        """
            Set the starting position of the connector

            @param x: Float value corresponding to the x coordinate
            @param y: Float value corresponding to the y coordinate
        """
        self.source_position = [x, y]
        # Create an extra point to avoid in most cases to get the connector totally hidden behind the state
        self.after_source_position = [x, y + 10]

    def set_destination(self, x, y):
        """
            Set the end position of the connector

            @param x: Float value corresponding to the x coordinate
            @param y: Float value corresponding to the y coordinate
        """
        self.destination_position = [x, y]
        # Create an extra point to avoid in most cases to get the connector totally hidden behind the state
        self.before_destination_position = [x, y - 10]

    def paint(self, painter, QStyleOptionGraphicsItem, widget=None):
        """
            Paint the connector onto the QGraphicsView

            @param painter: QPainter that will render the widget
            @param QStyleOptionGraphicsItem: Options provividing style options for the item
            @param widget: Specify another QWidget on which this item will be painted on. Default to None
        """
        # Depending on the action that involves the connector, select the proper pen
        if self.connector.end_socket is None:
            pen = self.pen_dragging
        else:
            pen = self.pen if not self.isSelected() else self.pen_selected
        # Scale the width of the connector if needed
        pen.setWidthF(self.pen_width * self.pen_width_compensator)
        painter.setBrush(Qt.NoBrush)
        painter.setPen(pen)
        # If we don't add this line, clicks on this object won't be detected
        self.setPath(self.calculate_path())
        # Draw the path that we compute ourselves
        painter.drawPath(self.path())

    def calculate_path(self):
        """
            Compute the path between the source and destination positions

            @return: QPainterPath corresponding to what pixels the connectors will go through
        """
        # Format all the points to create the path
        points = map(lambda x: QPointF(*x), [self.source_position, self.after_source_position,
                                             self.before_destination_position, self.destination_position])
        # Start the path from the source
        path = QPainterPath(points[0])
        for index, current_point in enumerate(points[1:-1], 1):
            # Get the previous segment
            source = QLineF(points[index - 1], current_point)
            # Compute the next segment
            next_segment = QLineF(current_point, points[index + 1])
            # Compute the angle required to have a nice jointure
            next_segment_angle = next_segment.angleTo(source)
            if next_segment_angle > 180:
                angle = (source.angle() + source.angleTo(next_segment) / 2) % 360
            else:
                angle = (next_segment.angle() + next_segment.angleTo(source) / 2) % 360

            total_length = source.length() * self.roundness_factor
            reversed_next_segment = QLineF.fromPolar(total_length, angle + 180).translated(current_point)
            # Get the second control point from this reverse engineering
            control_point_2 = reversed_next_segment.p2()
            # Depending on where we are in the loop do not use the same extrapolation betwee nthe points
            if index == 1:
                path.quadTo(control_point_2, current_point)
            else:
                # Use the control point 1 set in the previous iteration of the loop
                path.cubicTo(control_point_1, control_point_2, current_point)
            # Use the same reverse engineering process to get the first control point
            total_length = next_segment.length() * self.roundness_factor
            reversed_source = QLineF.fromPolar(total_length, angle).translated(current_point)
            control_point_1 = reversed_source.p2()

        # The final curve, that joins the last point
        path.quadTo(control_point_1, points[-1])
        # Create a PainterStorker so collision area fits the shape of the path
        stroke = QPainterPathStroker()
        # The fillable area is set to 0
        stroke.setWidth(0)
        # Return the path
        return stroke.createStroke(path)
