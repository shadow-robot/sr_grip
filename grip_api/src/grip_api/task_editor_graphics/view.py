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

from PyQt5.QtWidgets import QGraphicsView
from PyQt5.QtCore import Qt, QEvent, pyqtSignal
from PyQt5.QtGui import QPainter, QMouseEvent
from state_content import GraphicsStateContent
from socket import GraphicsSocket
from terminal_socket import TerminalGraphicsSocket
from grip_api.task_editor_widgets.connector import Connector
from connector import GraphicsConnector
from state import GraphicsState
from state_machine import GraphicsStateMachine


class TaskEditorView(QGraphicsView):

    """
        Widget allowing to visualize the content of a container
    """
    # Signal triggered when the view is scaled and gives the level of zoom
    viewScaled = pyqtSignal(int)

    def __init__(self, graphics_scene, parent=None):
        """
            Initialize the widget

            @param graphics_scene: QGraphicsScene object linked to the widget
            @param parent: Parent of the widget
        """
        super(TaskEditorView, self).__init__(parent)
        # Store the QGraphicsScene
        self.graphics_scene = graphics_scene
        self.init_ui()
        # Create lists that will contain functions to be executed when drag enter and drop events occur
        self.drag_enter_listeners = list()
        self.drop_listeners = list()
        # Set the QGraphicsScene
        self.setScene(self.graphics_scene)
        # Set the zooming parameters
        self.zoom_in_multiplier = 1.1
        self.zoom_out_multiplier = 1 / 1.1
        self.current_zoom = 0
        self.zoom_range = [-15, 15]
        # Indicates whether the user is dragging an edge
        self.is_dragging = False

    def init_ui(self):
        """
            Initialize the UI of the widget
        """
        # Bunch of options to make things look nice (not pixelized, etc.)
        self.setRenderHints(QPainter.Antialiasing | QPainter.HighQualityAntialiasing |
                            QPainter.TextAntialiasing | QPainter.SmoothPixmapTransform)
        self.setViewportUpdateMode(QGraphicsView.FullViewportUpdate)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setDragMode(QGraphicsView.RubberBandDrag)
        self.setAcceptDrops(True)
        self.rubberBandDraggingRectangle = False

    def add_drag_enter_listener(self, callback):
        """
            Add a function (callback) to be executed when a dragged object enters the view

            @param callback: Function or method to be executed
        """
        self.drag_enter_listeners.append(callback)

    def add_drop_listener(self, callback):
        """
            Add a function (callback) to be executed when an object is dropped onto the view

            @param callback: Function or method to be executed
        """
        self.drop_listeners.append(callback)

    def connector_drag_start(self, graphics_socket):
        """
            Function called when the user starts dragging a connector from a socket to another

            @param graphics_socket: GraphicsSocket object set as the source of the connector to be created
        """
        self.drag_start_socket = graphics_socket.socket
        # Dummy connector used for display purpose
        self.drag_connector = Connector(self.graphics_scene.container, graphics_socket.socket, None)

    def connector_drag_end(self, item):
        """
            Function called when the user releases the previous dragged conenctor

            @param item: Object on which the connector has been released
            @return: Boolean stating if a connector has been created
        """
        # Switch off the dragging flag
        self.is_dragging = False
        # Remove the connector set until that point (the dashed one)
        self.drag_connector.remove()
        self.drag_connector = None
        # If the dummy connector is released on an object capable of handling it, then create the final connector
        if isinstance(item, GraphicsSocket) or isinstance(item, TerminalGraphicsSocket):
            # Make sure the dummy connector has not been released on its origin
            if item.socket != self.drag_start_socket:
                # We want to keep all the connectors coming from target socket, if it's a multi connected one
                if not item.socket.is_multi_connected:
                    item.socket.remove_all_connectors()

                # We want to keep all the connectors coming from source socket, if it's a multi connected one
                if not self.drag_start_socket.is_multi_connected:
                    self.drag_start_socket.remove_all_connectors()
                # Make sure we cannot link two terminal sockets
                if self.drag_start_socket.is_terminal and item.socket.is_terminal:
                    return False

                # Make sure we cannot create a connector from input to input socket or output to output. We also need to
                # deal with the case of the starting terminal socket for concurrent state machines.
                is_start_socket_terminal = self.drag_start_socket.is_terminal and self.drag_start_socket.is_starting
                is_target_socket_input = item.socket.is_multi_connected and not item.socket.is_terminal

                if self.drag_start_socket.is_multi_connected ^ item.socket.is_multi_connected:
                    Connector(self.graphics_scene.container, self.drag_start_socket, item.socket)
                    return True
                elif is_start_socket_terminal and is_target_socket_input:
                    Connector(self.graphics_scene.container, self.drag_start_socket, item.socket)
                    return True

        return False

    def is_distance_between_click_and_release_enough(self, event):
        """
            Check if the the click and release events occured at least 100 pixels apart

            @param event: QMouseEvent sent by PyQt5
            @return: Boolean stating whether the distance of the two events is bigger than 100 pixels
        """
        # Get the release event position
        new_left_mouse_release_scene_pos = self.mapToScene(event.pos())
        dist_scene = new_left_mouse_release_scene_pos - self.last_left_mouse_click_scene_pos
        # Compute the distance in the view
        threshold = 100
        return (dist_scene.x() * dist_scene.x() + dist_scene.y() * dist_scene.y()) > threshold

    def delete_selected(self):
        """
            Delete all the valid items selected in the graphics view
        """
        # For each item that is currently selected, only remove states and connectors (but not the terminal sockets)
        for item in self.graphics_scene.selectedItems():
            if isinstance(item, GraphicsConnector):
                item.connector.remove()
            elif isinstance(item, GraphicsState):
                item.state.remove()
            elif isinstance(item, GraphicsStateMachine):
                item.state_machine.remove()

    def mousePressEvent(self, event):
        """
            Override the function handling mouse press events

            @oaram event: QMouseEvent sent by PyQt5
        """
        # Depending on which mouse button has been pressed, direct toward the corresponding function
        if event.button() == Qt.MiddleButton:
            self.middleMouseButtonPress(event)
        elif event.button() == Qt.LeftButton:
            self.leftMouseButtonPress(event)
        elif event.button() == Qt.RightButton:
            self.rightMouseButtonPress(event)
        else:
            super(TaskEditorView, self).mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        """
            Override the function handling mouse release events

            @param event: QMouseEvent sent by PyQt5
        """
        # Depending on which mouse button has been released, direct toward the corresponding function
        if event.button() == Qt.MiddleButton:
            self.middleMouseButtonRelease(event)
        elif event.button() == Qt.LeftButton:
            self.leftMouseButtonRelease(event)
        elif event.button() == Qt.RightButton:
            self.rightMouseButtonRelease(event)
        else:
            super(TaskEditorView, self).mouseReleaseEvent(event)

    def middleMouseButtonPress(self, event):
        """
            Override the function handling the middle mouse button press event

            @param event: QMouseEvent sent by PyQt5
        """
        # Make sure we release any mouse button event that might have been triggered before by the user
        release_event = QMouseEvent(QEvent.MouseButtonRelease, event.localPos(), event.screenPos(),
                                    Qt.LeftButton, Qt.NoButton, event.modifiers())
        super(TaskEditorView, self).mouseReleaseEvent(release_event)
        # Create the dragging mode
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        # Emulate the event that would normally be triggered by a left mouse press
        fake_event = QMouseEvent(event.type(), event.localPos(), event.screenPos(),
                                 Qt.LeftButton, event.buttons() | Qt.LeftButton, event.modifiers())
        super(TaskEditorView, self).mousePressEvent(fake_event)

    def middleMouseButtonRelease(self, event):
        """
            Override the function handling the middle mouse button release event

            @param event: QMouseEvent sent by PyQt5
        """
        # Make the event of release the left click to drag
        fake_event = QMouseEvent(event.type(), event.localPos(), event.screenPos(),
                                 Qt.LeftButton, event.buttons() & ~Qt.LeftButton, event.modifiers())
        super(TaskEditorView, self).mouseReleaseEvent(fake_event)
        # Set the dragging
        self.setDragMode(QGraphicsView.RubberBandDrag)

    def leftMouseButtonPress(self, event):
        """
            Override the function handling the left mouse button press event

            @param event: QMouseEvent sent by PyQt5
        """
        # Get the item that is being clicked
        item = self.itemAt(event.pos())
        # Store the position of the event
        self.last_left_mouse_click_scene_pos = self.mapToScene(event.pos())
        # If we press the left button on a GraphicsSocket, then start the dragging of a connector
        if isinstance(item, GraphicsSocket):
            if not self.is_dragging:
                self.is_dragging = True
                self.connector_drag_start(item)
                return
        # If we are already dragging, make sure to update teh end of the dummy connector so the user can see it
        if self.is_dragging:
            is_connector_created = self.connector_drag_end(item)
            # If the connector is created then don't go through the normal left click behaviour
            if is_connector_created:
                return
        # Make sure to execute the normal behaviour if required
        super(TaskEditorView, self).mousePressEvent(event)

    def leftMouseButtonRelease(self, event):
        """
            Override the function handling the left mouse button release event

            @param event: QMouseEvent sent by PyQt5
        """
        # Get the item that is being clicked
        item = self.itemAt(event.pos())
        # If we are dragging an edge and the release action is not too far from the previous click
        if self.is_dragging and self.is_distance_between_click_and_release_enough(event):
            # Drag the connector there
            is_connector_created = self.connector_drag_end(item)
            # If the connector is created then don't run the original behaviour
            if is_connector_created:
                return
        super(TaskEditorView, self).mouseReleaseEvent(event)

    def rightMouseButtonPress(self, event):
        """
            Override the function handling the right mouse button press event

            @param event: QMouseEvent sent by PyQt5
        """
        # Do nothing special, but if not overriden, then the right mouse does not answer properly
        super(TaskEditorView, self).mousePressEvent(event)

    def rightMouseButtonRelease(self, event):
        """
            Override the function handling the right mouse button release event

            @param event: QMouseEvent sent by PyQt5
        """
        # Do nothing special, but if not overriden, then the right mouse does not answer properly
        super(TaskEditorView, self).mouseReleaseEvent(event)

    def mouseMoveEvent(self, event):
        """
            Function triggered whe the mouse is moved in the view

            @param event: QMouseEvent
        """
        # If the user is dragging a connector, update its destination with the mouse position
        if self.is_dragging:
            pos = self.mapToScene(event.pos())
            self.drag_connector.graphics_connector.set_destination(pos.x(), pos.y())
            self.drag_connector.graphics_connector.update()
        super(TaskEditorView, self).mouseMoveEvent(event)

    def mouseDoubleClickEvent(self, event):
        """
            Function triggered when the user double clicks in the graphics view

            @param event: QMouseEvent sent by PyQt5
        """
        # Get the item the event occured on
        item = self.itemAt(event.pos())
        # If it's the starting socket, then starts the connector dragging
        if isinstance(item, TerminalGraphicsSocket) and item.socket.is_starting:
            if not self.is_dragging:
                self.is_dragging = True
                self.connector_drag_start(item)
                return
        super(TaskEditorView, self).mouseDoubleClickEvent(event)

    def keyPressEvent(self, event):
        """
            Override the function handling the key press event

            @param event: QKeyEvent sent by PyQt5
        """
        # Do nothing special, but if not overriden, then the event realted to the keyboard does not worf properly
        super(TaskEditorView, self).keyPressEvent(event)

    def dragEnterEvent(self, event):
        """
            Function called when an item is dragged in this widget

            @param event: QDragEvent
        """
        for callback in self.drag_enter_listeners:
            callback(event)

    def dropEvent(self, event):
        """
            Function called when an item is dropped onto this widget

            @param event: QDropEvent
        """
        for callback in self.drop_listeners:
            callback(event)

    def wheelEvent(self, event):
        """
            Function triggered when the wheel of a mouse is activated

            @param event: QWheelEvent triggered by PyQt5
        """
        # Get the item under the mouse when the wheel event is triggered
        pointed_item = self.itemAt(event.pos())
        # Make sure to be able to scroll the content of the state with the wheel
        if isinstance(pointed_item, GraphicsStateContent):
            super(TaskEditorView, self).wheelEvent(event)
            return

        # Calculate zoom
        if event.angleDelta().y() > 0:
            zoom_to_apply = self.zoom_in_multiplier
            self.current_zoom += 1
        else:
            zoom_to_apply = self.zoom_out_multiplier
            self.current_zoom -= 1

        # Clamp the zoom if required
        clamped = False
        if self.current_zoom < self.zoom_range[0]:
            self.current_zoom, clamped = self.zoom_range[0], True
        if self.current_zoom > self.zoom_range[1]:
            self.current_zoom, clamped = self.zoom_range[1], True

        # Set view scale
        if not clamped:
            # Emit the signal giving the current zoom
            self.viewScaled.emit(self.current_zoom)
            self.scale(zoom_to_apply, zoom_to_apply)
