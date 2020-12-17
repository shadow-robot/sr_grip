# !/usr/bin/env python

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

from PyQt5.QtWidgets import QMdiArea, QMdiSubWindow, QMenu
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QIcon, QPixmap
from grip_core.utils.common_paths import RED_CIRCLE, GREEN_CIRCLE
from graphical_editor_widget import GraphicalEditorWidget


class TaskEditorMDIArea(QMdiArea):

    """
        Widget managing the Multi Document Interface (MDI) required to design and execute tasks
    """

    def __init__(self, parent=None):
        """
            Initialise the MDI Area and automatically create the root state machine

            @param parent: Parent of the widget
        """
        super(TaskEditorMDIArea, self).__init__(parent=parent)
        self.init_ui()
        # Update which subwindow has currently the focus
        self.subWindowActivated.connect(self.track_activated_subwindow)
        # Add a subwindow containing the base of all state machines
        self.add_subwindow("root", "base")
        # Initialize the focused subwindow
        self.focused_subwindow = self.subWindowList()[0]

    def init_ui(self):
        """
            Initialize the UI of the widget
        """
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.setViewMode(QMdiArea.TabbedView)
        self.setDocumentMode(True)
        # Make sure the user can close the different tabs
        self.setTabsClosable(True)
        self.setTabsMovable(True)
        self.setWindowFlags(Qt.FramelessWindowHint)

    def track_activated_subwindow(self, sub_window):
        """
            Function called everytime a subwindow is activated. Updates the internal variable that stores which
            sub-window the user is working on

            @param sub_window: QMdiSubWindow object that is being activated
        """
        self.focused_subwindow = sub_window

    def add_subwindow(self, state_machine_name, state_machine_type):
        """
            Add a new custom made subwindow containing a GraphicalEditorWidget

            @param state_machine_name: Name of the state machine that will be contained in the subwindow
            @param state_machine_type: Type of the state machine that will be contained in the subwindow
        """
        # Create the subwindow
        subwindow = TaskEditorSubWindow(state_machine_name, state_machine_type, parent=self)
        # Add it to the MDI area
        self.addSubWindow(subwindow)
        self.setActiveSubWindow(subwindow)
        # Make sure to get a nice visualization
        subwindow.showMaximized()


class TaskEditorSubWindow(QMdiSubWindow):

    """
        Subwindow that will contain a GraphicalEditorWidget in which a state machine can be configured
    """

    def __init__(self, state_machine_name, state_machine_type, parent=None):
        """
            Initialize the class by adding a state machine to the the GraphicalEditorWidget

            @param state_machine_name: Name of the state machine that will be set to the GraphicalEditorWidget
            @param state_machine_type: Type of the state machine that will be loaded to the GraphicalEditorWidget
            @param parent: Parent of the widget
        """
        super(TaskEditorSubWindow, self).__init__(parent=parent)
        self.init_ui()
        # Set the subwindow icon
        self.setWindowIcon(self.red_icon)
        # Create the widget
        widget_to_set = GraphicalEditorWidget(state_machine_name, state_machine_type, parent=self)
        # Set the widget
        self.setWidget(widget_to_set)
        # If the subwindow is the main, remove the possibility of removing it
        if state_machine_type == "base":
            self.setSystemMenu(QMenu(self))

    def init_ui(self):
        """
            Initialize the subwindow
        """
        # Define resources notifying whether the state machine is valid or not
        self.red_icon = QIcon(QPixmap(RED_CIRCLE))
        self.green_icon = QIcon(QPixmap(GREEN_CIRCLE))
        # This command makes sure that when the tab is removed, the subwindow is as well
        self.setAttribute(Qt.WA_DeleteOnClose)

    def change_icon(self, is_valid):
        """
            Change the color of the icon of the tab to signal the user the state machine is correct

            @param is_valid: Boolean corresponding to the current validity of the corresponding scene
        """
        icon_to_set = self.green_icon if is_valid else self.red_icon
        self.setWindowIcon(icon_to_set)

    def remove(self):
        """
            Remove the window from the associated MDI area
        """
        self.mdiArea().removeSubWindow(self)

    def closeEvent(self, event):
        """
            Function called when the user closes a subwindow

            @param event: QCloseEvent sent by PyQt5
        """
        # Make sure the user cannot close the base state machine (can't hide the close button for a specific subwindow)
        if self.widget().container.type == "root":
            event.ignore()
        # Otherwise proceed as usual
        else:
            # Remove the state-like representation from the parent graphical editor as well
            self.widget().container.state_machine.remove(is_window_closed=True)
            event.accept()
