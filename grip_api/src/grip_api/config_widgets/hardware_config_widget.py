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

from PyQt5.QtWidgets import QWidget, QGridLayout
from PyQt5.QtCore import pyqtSignal
from plain_editor_widgets import YAMLEditorWidget
from component_editor_widgets import ComponentEditorWidget, RosControllersEditorWidget, MoveItPlannerEditorWidget
from grip_api.utils.files_specifics import ARM_CONFIG, HAND_CONFIG


class HardwareConfigWidget(QWidget):

    """
        Widget allowing the user to configure a robot arm or hand and integrate it to the framework
    """
    # Create a signal parametrized by a boolean specifying if the hardware config is in a different state as its initial
    hardwareChanged = pyqtSignal(bool)

    def __init__(self, hardware_part, parent=None):
        """
            Initialize the class by creating the layout and initializing the widgets

            @param hardware_part: String stating whether the widget is meant for an "Arm" or a "Hand"
            @param parent: parent of the widget
        """
        super(HardwareConfigWidget, self).__init__(parent=parent)
        self.hardware_part = hardware_part
        self.setObjectName("{} config widget".format(hardware_part))
        # Configuration of the hardware config
        self.configuration = ARM_CONFIG if hardware_part == "Arm" else HAND_CONFIG
        # By default it is not valid
        self.is_config_valid = False
        self.init_ui()
        self.create_widgets()
        self.editor_content_changed = dict()
        self.connect_update()

    def init_ui(self):
        """
            Set the widget's layout
        """
        self.layout = QGridLayout()
        self.layout.setContentsMargins(5, 10, 5, 5)

    def create_widgets(self):
        """
            Initialize the editors allowing to configure the arm
        """
        self.hardware_connection_config = YAMLEditorWidget("{} hardware connection".format(self.hardware_part),
                                                           parent=self)
        self.ros_controllers = RosControllersEditorWidget("ROS controllers", parent=self)
        self.moveit_planners_config = MoveItPlannerEditorWidget("MoveIt! planners", parent=self)
        self.kinematic_libraries_config = ComponentEditorWidget("External kinematics", parent=self)
        self.external_controller = ComponentEditorWidget("External controllers", parent=self)
        self.external_motion_planner = ComponentEditorWidget("External Motion Planners", parent=self)

        self.layout.addWidget(self.hardware_connection_config, 0, 0)
        self.layout.addWidget(self.ros_controllers, 0, 1)
        self.layout.addWidget(self.moveit_planners_config, 0, 2)
        self.layout.addWidget(self.kinematic_libraries_config, 1, 0)
        self.layout.addWidget(self.external_controller, 1, 1)
        self.layout.addWidget(self.external_motion_planner, 1, 2)
        self.setLayout(self.layout)

    def handle_editor_content_signal(self, has_widget_changed):
        """
            Emit a signal stating whether the content of one of the editors of hardware configuration has changed

            @param has_widget_changed: Boolean stating whether the content of the sender has changed
        """
        # Since each object has got an unique name, store it in a dictionary
        self.editor_content_changed[self.sender().objectName()] = has_widget_changed
        # Emits the signal. If any of the children widgets has been changed then it means the configuration has changed
        self.hardwareChanged.emit(any(self.editor_content_changed.values()))

    def set_default_enabled(self):
        """
            Enable/Disable the different widgets corresponding to the "default" configuration
        """
        is_widget_enabled = self.isEnabled()
        self.ros_controllers.setEnabled(is_widget_enabled)
        self.external_controller.setEnabled(is_widget_enabled)
        self.external_motion_planner.setEnabled(is_widget_enabled)
        self.kinematic_libraries_config.setEnabled(is_widget_enabled)

    def connect_update(self):
        """
            Connect signals to a slot allowing to update the hardware configuration
        """
        self.hardware_connection_config.canBeSaved.connect(self.update_config)
        self.external_controller.canBeSaved.connect(self.update_config)
        self.kinematic_libraries_config.canBeSaved.connect(self.update_config)
        self.external_motion_planner.canBeSaved.connect(self.update_config)
        self.ros_controllers.canBeSaved.connect(self.update_config)
        self.moveit_planners_config.canBeSaved.connect(self.update_config)

    def update_config(self, test):
        """
            Update the current hardware configuration
        """
        self.configuration[self.sender().objectName()] = self.sender().valid_input
        self.update_validity()
        # Emit signal stating whether the widget has changed
        self.handle_editor_content_signal(test)

    def update_validity(self):
        """
            Update the attribute specifying whether the current configuration is valid
        """
        hardware_connection = self.configuration["Editor {} hardware connection".format(self.hardware_part)]
        ros_controllers = self.configuration["Editor ROS controllers"]
        moveit_planners = self.configuration["Editor MoveIt! planners"]
        external_kinematics = self.configuration["Editor External kinematics"]
        external_controllers = self.configuration["Editor External controllers"]
        external_planner = self.configuration["Editor External Motion Planners"]
        # If any field has an invalid input
        if any(x is None for x in (hardware_connection, ros_controllers, moveit_planners, external_kinematics,
                                   external_controllers, external_planner)):
            self.is_config_valid = False
            return
        # If no controller has been defined
        if not ros_controllers and not external_controllers:
            self.is_config_valid = False
            return
        # If Moveit planners are set but no ROS controllers are provided
        if moveit_planners and not ros_controllers:
            self.is_config_valid = False
            return
        self.is_config_valid = True

    def save_config(self, settings):
        """
            Store the state of this widget and its children into settings

            @param settings: QSettings object in which widgets' information are stored
        """
        widget_name = self.objectName()
        settings.beginGroup(widget_name)
        class_name = self.metaObject().className()
        settings.setValue("type", class_name)
        for widget in self.children():
            # We don't consider the layout
            if not isinstance(widget, QGridLayout):
                widget.save_config(settings)
        settings.endGroup()

    def restore_config(self, settings):
        """
            Restore the children's widget from the configuration saved in settings

            @param settings: QSettings object that contains information of the widgets to restore
        """
        settings.beginGroup(self.objectName())
        for widget in self.children():
            if not isinstance(widget, QGridLayout):
                widget.restore_config(settings)
        settings.endGroup()
