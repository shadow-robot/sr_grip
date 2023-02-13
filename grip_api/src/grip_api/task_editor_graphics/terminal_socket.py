#!/usr/bin/env python3

# Copyright 2020, 2021, 2023 Shadow Robot Company Ltd.
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

from PyQt5.QtWidgets import QGraphicsItem, QGraphicsTextItem, QMenu
from PyQt5.QtGui import QColor, QPen, QBrush, QFont, QTextCursor
from PyQt5.QtCore import QRectF, Qt
from grip_api.utils.files_specifics import TERMINAL_SOCKET_COLORS


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
        super().__init__(parent=parent)
        self.socket = socket
        self.socket.container.get_view().viewScaled.connect(self.update_transform)
        # Check whether the socket is used as the input of a state or not
        self.is_input = not self.socket.name
        self.init_resources()
        self.init_ui()
        self.init_title()
        # Flag stating whether the object has been moved around
        self.has_moved = False

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
        # Color for input sockets
        self.input_socket_color = QColor("#FF4599FF")
        # Get the color of the socket
        if self.socket.is_starting:
            self.socket_color = self.input_socket_color
        else:
            self.socket_color = TERMINAL_SOCKET_COLORS[self.socket.index]
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
        # Add this flag so that deletable terminal sockets are directly created with proper transform
        self.setFlag(QGraphicsItem.ItemIgnoresTransformations, self.socket.container.get_view().current_zoom < 0)
        self.setAcceptHoverEvents(True)
        # Create the context menu that will appear when right click is executed on the object
        self.context_menu = QMenu()
        self.rename_action = self.context_menu.addAction("Rename")
        self.make_default_action = self.context_menu.addAction("Make default")

    def contextMenuEvent(self, event):
        """
            Function triggered when right click is pressed to make a context menu appear

            @param event: QContextMenuEvent sent by PyQt5
        """
        # Get the action selected by the user
        action = self.context_menu.exec_(event.screenPos())
        # If the selected action is "Rename" then trigger this functionality
        if action == self.rename_action:
            self.title.rename()
        # If this action is selected, update the default socket of the container
        elif action == self.make_default_action:
            self.socket.container.default_socket = self.socket

    def init_title(self):
        """
            Add a text below the terminal socket
        """
        self.title = OutcomeTitle(self)
        # Set the text
        self.title.setPlainText(self.socket.name)
        # Make sure the text takes the right amount of space
        self.title.adjustSize()
        # Center it below or above the socket
        # Below
        height_offset = self.radius + self.outline_width + self.title_padding
        # Above (need to add another offset corresponding to the title height)
        if self.socket.is_starting:
            height_offset = -height_offset - 30
        self.title.setPos(-self.title.textWidth() / 2., height_offset)

    def update_name(self):
        """
            Update the text associated to the object, correpsonding to its name
        """
        self.title.update_text()

    def get_total_height(self):
        """
            Get the total height of the socket + title

            @return: Integer corresponding to the height (in pixels) of the socket and title
        """
        socket_height = 2 * (self.radius + self.outline_width)
        return socket_height + self.title_padding + self.title.font().pointSize()

    def mouseMoveEvent(self, event):
        """
            Function triggered when this object is moved by the user

            @param event: QMouseEvent sent by PyQt5
        """
        super().mouseMoveEvent(event)
        # Socket is being moved
        self.has_moved = True
        # Update all the connectors that are linked to this terminal socket
        for connector in self.socket.connectors:
            connector.update_positions()

    def mouseReleaseEvent(self, event):
        """
            Function triggered when the mouse is released (click off) from this object

            @param event: QMouseEvent sent by PyQt5
        """
        super().mouseReleaseEvent(event)
        # If the object has been moved
        if self.has_moved:
            # Reset the flag
            self.has_moved = False
            # Store the history
            self.socket.container.history.store_current_history()

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


class OutcomeTitle(QGraphicsTextItem):

    """
        QGraphicsTextItem used to display and interact with the name of an outcome
    """

    def __init__(self, parent):
        """
            Initialize the widget

            @param parent: Parent of this widget
        """
        super().__init__(parent=parent)
        # Store the parent TerminalGraphicsSocket the title is linked to
        self.parent = parent
        # Set default visualisation parameters
        self.setDefaultTextColor(Qt.white)
        self.setFont(QFont("Ubuntu", 14))

    def rename(self):
        """
            Make the title interactive so the user can set a new name for the outcome
        """
        # Make the widget both selectable and editable
        self.setFlag(QGraphicsItem.ItemIsSelectable)
        self.setTextInteractionFlags(Qt.TextEditable)
        # Set the focus on this object
        self.setFocus()
        # Makes sure the user can see the whole name
        self.setPlainText(self.parent.socket.name)
        text_cursor = self.textCursor()
        # Make sure to send the cursor to the beginning
        text_cursor.movePosition(QTextCursor.Start, QTextCursor.MoveAnchor)
        # Select all the text
        text_cursor.movePosition(QTextCursor.End, QTextCursor.KeepAnchor)
        # Set the cursor
        self.setTextCursor(text_cursor)

    def keyPressEvent(self, event):
        """
            Function triggered when a key is hit when interacting with this widget

            @param event: QKeyEvent triggered by PyQt5
        """
        # Extract which keys and modifiers (Ctrl, Alt, Shit) have been pressed
        pressed_key = event.key()
        key_modifier = event.modifiers()

        # If Enter is pressed, then ends the text interaction phasis
        if event.key() == Qt.Key_Return:
            self.setFlag(QGraphicsItem.ItemIsSelectable, False)
            self.setTextInteractionFlags(Qt.NoTextInteraction)
            return

        # Get the current QTextCursor
        text_cursor = self.textCursor()
        # If Shift + directional arrows (right or left) is pressed, selects the text up to were the cursor is moved
        if pressed_key == Qt.Key_Left and key_modifier == Qt.ShiftModifier:
            text_cursor.movePosition(QTextCursor.PreviousCharacter, QTextCursor.KeepAnchor)
            self.setTextCursor(text_cursor)
        elif pressed_key == Qt.Key_Right and key_modifier == Qt.ShiftModifier:
            text_cursor.movePosition(QTextCursor.NextCharacter, QTextCursor.KeepAnchor)
            self.setTextCursor(text_cursor)
        # If Ctrl + A is pressed then slects all the text, regardless from where the cursor is
        elif pressed_key == Qt.Key_A and key_modifier == Qt.ControlModifier:
            text_cursor.movePosition(QTextCursor.Start, QTextCursor.MoveAnchor)
            text_cursor.movePosition(QTextCursor.End, QTextCursor.KeepAnchor)
            self.setTextCursor(text_cursor)
        # Move within the text
        elif pressed_key == Qt.Key_Right:
            text_cursor.movePosition(QTextCursor.NextCharacter, QTextCursor.MoveAnchor)
            self.setTextCursor(text_cursor)
        elif pressed_key == Qt.Key_Left:
            text_cursor.movePosition(QTextCursor.PreviousCharacter, QTextCursor.MoveAnchor)
            self.setTextCursor(text_cursor)
        # Otherwise just process the keys as usual
        else:
            super().keyPressEvent(event)

    def mouseDoubleClickEvent(self, event):
        """
            Function triggered when the user double clicks on the terminal socket

            @param event: QMouseEvent sent by PyQt5
        """
        # Make sure we cannot rename the starting terminal socket
        if not self.parent.socket.is_starting:
            self.rename()

    def focusOutEvent(self, event):
        """
            Function called when this widget loses the focus

            @param event: QFocusEvent sent by PyQt5
        """
        # Get the current QTextCursor
        text_editor = self.textCursor()
        # Make sure to send the cursor back to the beginning
        text_editor.movePosition(QTextCursor.Start, QTextCursor.MoveAnchor)
        self.setTextCursor(text_editor)
        # Make the item not selectable so we can drag the parent around by grabbing the title if not double clicked
        self.setFlag(QGraphicsItem.ItemIsSelectable, False)
        self.setTextInteractionFlags(Qt.NoTextInteraction)
        # Call the original behaviour
        super().focusOutEvent(event)
        # Make sure the object is not left with an empty name
        if not self.toPlainText():
            self.setPlainText(self.parent.socket.name)
        # Update the name of the outcome with the current text, making sure we don't have two items with the same name
        if self.parent.socket.name != self.toPlainText():
            self.parent.socket.update_name(self.toPlainText())
            # Update the display of the socket's name
            self.update_text()

    def update_text(self):
        """
            Update the name of outcome and make sure the text is aligned with the graphical socket
        """
        self.setPlainText(self.parent.socket.name)
        self.adjustSize()
        self.setX(-self.textWidth() / 2)
