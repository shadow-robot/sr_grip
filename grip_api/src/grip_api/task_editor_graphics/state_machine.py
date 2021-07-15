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

from PyQt5.QtWidgets import QGraphicsItem, QGraphicsTextItem, QGraphicsProxyWidget, QMenu
from PyQt5.QtCore import Qt, QRectF
from PyQt5.QtGui import QFont, QColor, QPen, QBrush, QPainterPath, QTransform, QTextCursor, QFontMetrics


class GraphicsStateMachine(QGraphicsItem):

    """
        Graphical representation of a state-like state machine
    """

    def __init__(self, state_machine, parent=None):
        """
            Initialize the widget

            @param state_machine: State machine linked to this graphical representation
            @param parent: Parent of this widget
        """
        super(GraphicsStateMachine, self).__init__(parent=parent)
        self.state_machine = state_machine
        self.init_visu_tools()
        self.init_dimensions()
        # Connect the signal coming from the view to a function that will update the behaviour
        self.state_machine.container.get_view().viewScaled.connect(self.update_scaling_factor)
        self.init_ui()
        # Flag to know if the state has been moved
        self.is_moved = False

    def init_dimensions(self):
        """
            Set initial dimensions and constant required to properly display the state
        """
        # Current zoom to be applied to this widget
        self.zoom = self.state_machine.container.get_view().current_zoom
        # Get the zoom in multiplier stored in the view
        self.zoom_multiplier = self.state_machine.container.get_view().zoom_in_multiplier
        # Get the current scaling factor when the object is being created
        self.scaling_factor = self.zoom_multiplier**self.zoom
        # Set the zoom from which the state will be collapsed
        self.zoom_threshold = -7
        # Height of the title (valid only for the font currently used, i.e. ubuntu 14)
        self.title_height = 30.0
        # Padding between edge and any element of the state
        self.edge_padding = 10.0
        # Roundness of the outline
        self.edge_roundness = 10.0
        # Get the content size. The 15 comes from the display size of the scroll bar.
        self.content_initial_width = self.state_machine.content.width() + 15
        self.content_initial_height = self.state_machine.content.height()
        # Compute the initial width and height of the widget. The 1 comes from half the outline width
        self.width = self.content_initial_width + 2 * (self.edge_padding + 1)
        self.height = self.content_initial_height + 2 * (self.edge_padding + 1) + self.title_height
        # Get the minimum height of the state when collapsed
        self.min_height = self.title_height + 2 * (self.edge_padding + 1) + self.edge_padding

    def init_visu_tools(self):
        """
            Initialize the different attributes responsible for the look of the state machine (brush colour, etc.)
        """
        # Black colour for the outline
        self.default_colour = QColor("#FF000000")
        # Orange outline when selected
        self.color_selected = QColor("#FFFFA637")
        # Create the different pens
        self.pen_default = QPen(self.default_colour)
        self.pen_default.setWidthF(2.0)
        self.pen_selected = QPen(self.color_selected)
        self.pen_selected.setWidthF(2.0)
        # Create the background brush
        self.brush_background = QBrush(QColor("#E6393939"))

    def correct_initial_dimensions(self):
        """
            Correct the dimensions and visibility of the content if the object is dropped on a scaled view
        """
        if self.zoom <= self.zoom_threshold:
            self.graphics_content.setVisible(False)
            self.height = self.min_height
            self.scaling_factor = self.zoom_multiplier**self.zoom_threshold

    def update_scaling_factor(self, current_zoom):
        """
            Store the current scaling factor and set attributes, even for children if required

            @param current_zoom: Integer corresponding to the current zoom
        """
        # Store the previous zoom
        previous_zoom = self.zoom
        # Compute the scaling factor from the zoom
        self.zoom = current_zoom
        self.scaling_factor = self.zoom_multiplier**current_zoom
        # Here we do all the comparisons based on the zoom and not scaling factor to avoid inacurracies coming from
        # working with floats or double
        if self.zoom == self.zoom_threshold and previous_zoom > current_zoom:
            self.graphics_content.set_visible(False)
            # When the content is set to be not visible, update the height of the box
            self.height = self.min_height
        if self.zoom == self.zoom_threshold + 1 and previous_zoom < current_zoom:
            # When the content is et to be visible, update the height of the box
            self.graphics_content.set_visible(True)
            self.height = self.content_initial_height + 2 * (self.edge_padding + 1) + self.title_height
        # Clamp the zooming factor that can be applied to this widget
        if self.zoom < self.zoom_threshold:
            self.scaling_factor = self.zoom_multiplier**self.zoom_threshold

    def init_ui(self):
        """
            Initialize this widget's UI
        """
        # Make sure we can grab and move the item around
        self.setFlag(QGraphicsItem.ItemIsSelectable)
        self.setFlag(QGraphicsItem.ItemIsMovable)
        # Store the name of the state machine as an attribute
        self.name = self.state_machine.name
        # Create the context menu that will apear when the state machine is right clicked
        self.context_menu = QMenu()
        self.rename_action = self.context_menu.addAction("Rename")
        self.go_to_action = self.context_menu.addAction("Go to definition")
        # Initialize the title
        self.title = StateMachineTitle(self)
        # Makes sure the title is not too stuck to the outline
        self.title.setPos(self.edge_padding / 2, 0)
        # Initialize the graphical component to display the content of the state machine
        self.state_machine.content.setGeometry(self.edge_padding + 1, self.title_height + self.edge_padding + 2,
                                               self.content_initial_width, self.content_initial_height)
        self.graphics_content = GraphicsStateMachineContent(self.state_machine.content, parent=self)
        # In case the state machines is dropped on a scaled view, makes sure dimensions and the content is updated
        self.correct_initial_dimensions()

    def mouseMoveEvent(self, event):
        """
            Function triggered when this object is moved by the user

            @param event: QMouseEvent sent by PyQt5
        """
        super(GraphicsStateMachine, self).mouseMoveEvent(event)
        # If the object is selected and is moved, update the connectors linked to this state machine
        if self.isSelected():
            self.state_machine.update_connectors()
        # Update the flag
        self.is_moved = True

    def mouseReleaseEvent(self, event):
        """
            Function triggered when the mouse is released (click off) from this object

            @param event: QMouseEvent sent by PyQt5
        """
        super(GraphicsStateMachine, self).mouseReleaseEvent(event)
        if self.is_moved:
            # Reset the flag
            self.is_moved = False
            # Store the current state of the container
            self.state_machine.container.history.store_current_history(set_modified=True)

    def mousePressEvent(self, event):
        """
            Function triggered when a click action is performed on this object

            @param event: QMouseEvent sent by PyQt5
        """
        # Make sure the clicked state is not overlapped by another one is selected
        self.state_machine.container.z_tracker += 1
        self.setZValue(self.state_machine.container.z_tracker)
        super(GraphicsStateMachine, self).mousePressEvent(event)

    def boundingRect(self):
        """
            Return the bounding rectangle of the widget. Must be overriden to avoid issues when interacting with other
            widgets

            @return: QRectF containing the origin and size of the rectangle (x,y,w,h)
        """
        # We need to account for when we zoom out even after the widget appeareance is not affected
        if self.zoom < self.zoom_threshold:
            compensation_zoom = self.zoom_threshold - self.zoom
            compensation_factor = self.zoom_multiplier**compensation_zoom
        else:
            compensation_factor = 1
        return QRectF(
            0,
            0,
            self.width * compensation_factor,
            self.height * compensation_factor
        ).normalized()

    def mouseDoubleClickEvent(self, event):
        """
            Function triggered when the user double clicks on the state machine

            @param event: QMouseEvent sent by PyQt5
        """
        # Check whether the double click is done on the title. If it's the case then activate the renaming behaviour
        # We need to do it here and not in StateTitle because the event is not always propagated to the children
        if self.title.contains(event.pos()):
            self.title.rename()
        # Otherwise go to the state machine definition tab
        else:
            state_machine_definition_tab = self.state_machine.def_container.editor_widget.parent()
            state_machine_definition_tab.mdiArea().setActiveSubWindow(state_machine_definition_tab)

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
        # If the action is to go to the definition, go to the corresponding tab
        elif action == self.go_to_action:
            state_machine_definition_tab = self.state_machine.def_container.editor_widget.parent()
            state_machine_definition_tab.mdiArea().setActiveSubWindow(state_machine_definition_tab)

    def paint(self, painter, QStyleOptionGraphicsItem, widget=None):
        """
            Paint the content of the state machine onto the QGraphicsView

            @param painter: QPainter that will render the widget
            @param QStyleOptionGraphicsItem: Options provividing style options for the item
            @param widget: Specify another QWidget on which this item will be painted on. Default to None
        """
        # Get the transform between the view and the local coordinates
        world_transform = painter.worldTransform()
        # If required, set the appropriate transform to have a fixed size vcollapsed view of the state
        if self.zoom < self.zoom_threshold:
            t = QTransform(self.scaling_factor, world_transform.m12(), world_transform.m13(),
                           world_transform.m21(), self.scaling_factor, world_transform.m23(),
                           world_transform.m31(), world_transform.m32(), world_transform.m33())
            painter.setTransform(t)
        # Draw the background
        path_title = QPainterPath()
        path_title.setFillRule(Qt.WindingFill)
        path_title.addRoundedRect(0, 0, self.width, self.height, self.edge_roundness, self.edge_roundness)
        painter.setPen(Qt.NoPen)
        painter.setBrush(self.brush_background)
        painter.drawPath(path_title.simplified())

        # Draw the outline
        path_outline = QPainterPath()
        path_outline.addRoundedRect(0, 0, self.width, self.height, self.edge_roundness, self.edge_roundness)
        painter.setBrush(Qt.NoBrush)
        painter.setPen(self.pen_default if not self.isSelected() else self.pen_selected)
        painter.drawPath(path_outline.simplified())

    @staticmethod
    def create_unscaled_transform(transform_matrix):
        """
            Static method that creates an version of the input transform matrix for which the scaling factor is 1

            @param transform_matrix: QTransform to modify
            @return: New QTransform matrix with scaling factor set to 1
        """
        return QTransform(1, transform_matrix.m12(), transform_matrix.m13(),
                          transform_matrix.m21(), 1, transform_matrix.m23(),
                          transform_matrix.m31(), transform_matrix.m32(), transform_matrix.m33())


class StateMachineTitle(QGraphicsTextItem):

    """
        QGraphicsTextItem used to display and interact with the name of state machines
    """

    def __init__(self, parent):
        """
            Initialize the widget

            @param parent: Parent of this widget
        """
        super(StateMachineTitle, self).__init__(parent=parent)
        # Store the parent GraphicsStateMachine the title is linked to
        self.parent = parent
        # Set default visualisation parameters
        self.setDefaultTextColor(Qt.white)
        self.setFont(QFont("Ubuntu", 14))
        # Get the corresponding QFontMetrics required to run some adjustments
        self.font_metrics = QFontMetrics(self.font())
        # Make sure the title does not go over the parent's width
        self.adapt_text_length()
        # Initialize the tooltip of the state
        self.parent.setToolTip(self.parent.state_machine.name)

    def adapt_text_length(self):
        """
            Make sure the text disaplyed does not go over the GraphicsState maximum width
        """
        # Since we want to reason on the view reference, we need to apply some transforms
        # Get transform to go from the item's local coordinates to the view (what the user sees) coordinates
        view_transform = self.parent.state_machine.container.get_view().viewportTransform()
        # Apply the transform to the parent's bounding box and extract the actual width
        mapped_parent_width = view_transform.mapRect(self.parent.boundingRect()).width()
        # Get the text to be displayed
        text = self.parent.state_machine.name
        # If we zoom out, since we keep the text unscaled, get the proper transform matrix
        if self.parent.zoom < 0:
            transform_to_apply = self.parent.create_unscaled_transform(view_transform)
        else:
            transform_to_apply = view_transform
        # Max width we want the text to fit in. The 15 comes from empiric tests
        max_width = mapped_parent_width - 15
        # Get a rough estimate of how many chars would fit in the defined width
        nb_char = int(max_width / self.font_metrics.averageCharWidth())
        # While the bounding box the cropped text is larger than the max width in the view coordinates, remove one char
        # Note that we are using cropped text + ...
        while transform_to_apply.mapRect(self.font_metrics.boundingRect(text[:nb_char] + "...")).width() > max_width:
            nb_char -= 1
        # If the number of chars is smaller than the text then add the ... to show that the actual name is larger
        if nb_char < len(text):
            self.setPlainText(text[:nb_char] + "...")
        # otherwise fit in the whole name
        else:
            self.setPlainText(text)

    def rename(self):
        """
            Make the title interactive so the user can set a new name for the state
        """
        # Make the widget both selectable and editable
        self.setFlag(QGraphicsItem.ItemIsSelectable)
        self.setTextInteractionFlags(Qt.TextEditable)
        # Set the focus on this object
        self.setFocus()
        # Makes sure the user can see the whole name
        self.setPlainText(self.parent.state_machine.name)
        text_cursor = self.textCursor()
        # Make sure to send the cursor to the beginning
        text_cursor.movePosition(QTextCursor.Start, QTextCursor.MoveAnchor)
        # Select all the text
        text_cursor.movePosition(QTextCursor.End, QTextCursor.KeepAnchor)
        # Set the cursor
        self.setTextCursor(text_cursor)

    def contains(self, point):
        """
            Override the native function that tests if a given point is over the widget

            @param point: QPointF reprsenting the x,y coordinates
            @return: Boolean stating whether the point is part of the item's bounding box
        """
        # Transform the point to the view coordinates
        view_transform = self.parent.state_machine.container.get_view().viewportTransform()
        mapped_point = view_transform.map(point)
        # Depending on the current zoom, pick the proper transform that is applied to the item
        if self.parent.zoom < 0:
            unscaled_transform = self.parent.create_unscaled_transform(view_transform)
        else:
            unscaled_transform = view_transform
        # Get the mapped bounding text
        mapped_bounding_rect = unscaled_transform.mapRect(self.boundingRect())
        return mapped_bounding_rect.contains(mapped_point)

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
            super(StateMachineTitle, self).keyPressEvent(event)

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
        super(StateMachineTitle, self).focusOutEvent(event)
        # Update the name of the state machine with the current text
        self.parent.state_machine.def_container.editor_widget.set_name(self.toPlainText())
        # Make sure the text fits in the given width
        self.adapt_text_length()
        # Update the parent's tooltip
        self.parent.setToolTip(self.parent.state_machine.name)

    def paint(self, painter, QStyleOptionGraphicsItem, widget=None):
        """
            Paint the content of the state machine onto the QGraphicsView

            @param painter: QPainter that will render the widget
            @param QStyleOptionGraphicsItem: Options provividing style options for the item
            @param widget: Specify another QWidget on which this item will be painted on. Default to None
        """
        # Get the transform between the view and the local coordinates
        world_transform = painter.worldTransform()
        # Make sure the text is not scaled out, while keeping the possibility to scaling it up
        if self.parent.scaling_factor < 1:
            painter.setTransform(self.parent.create_unscaled_transform(world_transform))
        # Call the original painter with the updated (or not) painter
        super(StateMachineTitle, self).paint(painter, QStyleOptionGraphicsItem, widget)
        if not self.hasFocus():
            self.adapt_text_length()


class GraphicsStateMachineContent(QGraphicsProxyWidget):

    """
        QGraphicsProxyWidget that displays the content of the state machines
    """

    def __init__(self, widget, parent=None):
        """
            Initialize the widget

            @param widget: QWidget to be dispalyed in the view
            @param parent: Parent of this widget
        """
        super(GraphicsStateMachineContent, self).__init__(parent=parent)
        # Set the widget
        self.setWidget(widget)

    def set_visible(self, make_visible):
        """
            Make the widget visible or not, depending on the value

            @param make_visible: Boolean signaling whether the widget should be visible or not
        """
        self.setVisible(make_visible)
