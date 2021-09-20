#!/usr/bin/env python3

# Copyright 2020, 2021 Shadow Robot Company Ltd.
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

from PyQt5.QtWidgets import QHBoxLayout, QWidget, QVBoxLayout
from PyQt5.QtCore import pyqtSignal
from .interface_config_widgets import RobotInterfaceConfig, SimulationConfig, MoveitConfig


class RobotInterfaceWidget(QWidget):

    """
        Widget containing other widgets allowing to interface a robot
    """
    # Create a signal parametrized by a boolean specifying if the interface is in a different state as its initial
    interfaceChanged = pyqtSignal(bool)
    # Signal stating whether a custon launch file has been provided
    isCustomLaunchProvided = pyqtSignal(bool)

    def __init__(self, parent=None):
        """
            Initialize the class by creating the different widgets

            @param parent: parent of the widget
        """
        super(RobotInterfaceWidget, self).__init__(parent=parent)
        self.setObjectName("Robot interface widget")
        self.init_ui()
        self.create_widgets()
        self.modifiers = dict()
        self.connect_slots()

    def init_ui(self):
        """
            Create the layout of the widget
        """
        self.layout = QHBoxLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)

    def create_widgets(self):
        """
            Create the different widgets and add them to the layout.
        """
        # Create a vertical layout first
        left_hand_side_layout = QVBoxLayout()
        left_hand_side_layout.setContentsMargins(0, 0, 0, 0)

        # Create the widgets that will be contained in this sub-layout
        self.robot_config = RobotInterfaceConfig(parent=self)
        self.simulation_config = SimulationConfig(parent=self)
        # Add both widgets to the sub-layout
        left_hand_side_layout.addWidget(self.robot_config)
        left_hand_side_layout.addWidget(self.simulation_config)
        # Update the interface view according to the simulation checkbox state
        self.simulation_config.check_box.toggled.connect(self.update_interface_simulation)

        # Create the widget composing the right hand side of the layout
        self.moveit_config = MoveitConfig(parent=self)
        # Add the two components of the layout so they can be displayed
        self.layout.addLayout(left_hand_side_layout, 1)
        self.layout.addWidget(self.moveit_config, 1)
        # Set the layout otherwise nothing will be displayed
        self.setLayout(self.layout)

    def connect_slots(self):
        """
            Remap signals coming from all this widget's children
        """
        self.robot_config.robot_urdf_entry_widget.canBeSaved.connect(self.handle_signals)
        self.robot_config.urdf_args_entry_widget.canBeSaved.connect(self.handle_signals)
        self.robot_config.launch_file_entry_widget.canBeSaved.connect(self.handle_signals)
        self.robot_config.collision_scene_entry_widget.canBeSaved.connect(self.handle_signals)
        self.robot_config.arm_spin_box.inputChanged.connect(self.handle_signals)
        self.robot_config.hand_spin_box.inputChanged.connect(self.handle_signals)
        self.robot_config.sensor_spin_box.inputChanged.connect(self.handle_signals)
        self.simulation_config.simuModeChanged.connect(self.handle_signals)
        self.simulation_config.gazebo_file_entry_widget.canBeSaved.connect(self.handle_signals)
        self.simulation_config.gazebo_folder_entry_widget.canBeSaved.connect(self.handle_signals)
        self.simulation_config.starting_pose_entry_widget.canBeSaved.connect(self.handle_signals)
        self.moveit_config.moveit_package_entry_widget.canBeSaved.connect(self.handle_signals)
        # Signals coming from editors
        self.robot_config.launch_file_editor.canBeSaved.connect(self.handle_signals)
        self.moveit_config.move_group_editor.canBeSaved.connect(self.handle_signals)
        self.moveit_config.rviz_editor.canBeSaved.connect(self.handle_signals)
        # Signal coming from a potential custom launch file
        self.robot_config.launch_file_entry_widget.inputChanged.connect(self.trigger_launch_file_signal)

    def handle_signals(self, has_widget_changed):
        """
            Emit a signal stating whether the robot interface is in a different state than its original

            @param has_widget_changed: Boolean stating whether the widget is in a different state as its original
        """
        # Since each object has got an unique name, store it in a dictionary
        self.modifiers[self.sender().objectName()] = has_widget_changed
        # Emits the signal. If any of the children widgets has been changed then it tells that the interface has changed
        self.interfaceChanged.emit(any(self.modifiers.values()))

    def update_interface_simulation(self, is_checked):
        """
            Update interface view regarding to the state of the simulation checkebox

            @param is_checked: Boolean provided by the signal stating whether the checkbox is ticked or not
        """
        self.simulation_config.gazebo_file_entry_widget.setEnabled(is_checked)
        self.simulation_config.gazebo_folder_entry_widget.setEnabled(is_checked)
        self.simulation_config.starting_pose_entry_widget.setEnabled(is_checked)
        self.robot_config.collision_scene_entry_widget.setEnabled(not is_checked)

    def trigger_launch_file_signal(self):
        """
            Update the whole simulation section according to the custom launch file provided
        """
        is_launch_file_empty = not self.sender().entry_edit_line.text() or self.sender().valid_input is None
        self.isCustomLaunchProvided.emit(not is_launch_file_empty)

    def reset(self):
        """
            Reset the state of this widget to its initial, i.e. empty without any field configured
        """
        for widget in self.children():
            if not isinstance(widget, QHBoxLayout):
                widget.reset()
        self.modifiers = dict()

    def save_config(self, settings):
        """
            Store the state of this widget and its children into settings

            @param settings: QSettings object in which widgets' information are stored
        """
        settings.beginGroup(self.objectName())
        class_name = self.metaObject().className()
        settings.setValue("type", class_name)
        for widget in self.children():
            if not isinstance(widget, QHBoxLayout):
                widget.save_config(settings)
        settings.endGroup()

    def restore_config(self, settings):
        """
            Restore the children's widget from the configuration saved in settings

            @param settings: QSettings object that contains information of the widgets to restore
        """
        settings.beginGroup(self.objectName())
        for widget in self.children():
            if not isinstance(widget, QHBoxLayout):
                widget.restore_config(settings)
        settings.endGroup()
