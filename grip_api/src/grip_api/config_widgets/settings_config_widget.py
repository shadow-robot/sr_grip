#!/usr/bin/env python

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

from PyQt5.QtWidgets import QWidget, QGridLayout
from PyQt5.QtCore import pyqtSignal
from plain_editor_widgets import YAMLEditorWidget
from grip_api.utils.files_specifics import SETTINGS_CONFIG
from component_editor_widgets import (SensorEditorWidget, JointStateEditorWidget, ComponentEditorWidget,
                                      PoseEditorWidget, TrajectoryEditorWidget)


class SettingsConfigWidget(QWidget):

    """
        Widget allowing the user to configure the settings (recorded joint states, sensor configs, etc.)
    """
    settingsChanged = pyqtSignal(bool)

    def __init__(self, parent=None):
        """
            Initialize the class by creating the layout and initializing the widgets

            @param parent: parent of the widget
        """
        super(SettingsConfigWidget, self).__init__(parent=parent)
        self.setObjectName("Setup config widget")
        # Configuration of the settings config
        self.configuration = SETTINGS_CONFIG.copy()
        # By default it is not valid
        self.is_config_valid = True
        self.init_ui()
        self.create_widgets()
        self.editor_content_changed = dict()
        self.connect_slots()
        self.connect_update()

    def init_ui(self):
        """
            Set the widget's layout
        """
        self.layout = QGridLayout()
        self.layout.setContentsMargins(5, 10, 5, 5)
        self.setLayout(self.layout)

    def create_widgets(self):
        """
            Initialize the editors configuring the settings of the experiment
        """
        self.named_joint_states = JointStateEditorWidget("Named joint states", parent=self)
        self.layout.addWidget(self.named_joint_states, 0, 0)
        self.named_poses = PoseEditorWidget("Named poses", parent=self)
        self.layout.addWidget(self.named_poses, 0, 1)
        self.named_trajectories = TrajectoryEditorWidget("Named trajectories", parent=self)
        self.layout.addWidget(self.named_trajectories, 0, 2)
        self.sensor_configs = SensorEditorWidget("Sensors config", parent=self)
        self.layout.addWidget(self.sensor_configs, 1, 0)
        self.sensor_plugins = YAMLEditorWidget("Sensor plugins", parent=self)
        self.layout.addWidget(self.sensor_plugins, 1, 1)
        self.external_methods = ComponentEditorWidget("High level methods", enabled=True, parent=self)
        self.layout.addWidget(self.external_methods, 1, 2)

    def connect_slots(self):
        """
            Remap signals coming from all this widget's children
        """
        # Remap signals coming from changes in editors' content
        self.named_joint_states.canBeSaved.connect(self.handle_editor_content_signal)
        self.named_poses.canBeSaved.connect(self.handle_editor_content_signal)
        self.named_trajectories.canBeSaved.connect(self.handle_editor_content_signal)
        self.sensor_configs.canBeSaved.connect(self.handle_editor_content_signal)
        self.sensor_plugins.canBeSaved.connect(self.handle_editor_content_signal)
        self.external_methods.canBeSaved.connect(self.handle_editor_content_signal)
        # Update the pool of elements that can be used in other editors
        self.named_joint_states.canBeSaved.connect(self.update_new_checkpoints)
        self.named_poses.canBeSaved.connect(self.update_new_poses)

    def handle_editor_content_signal(self, has_widget_changed):
        """
            Emit a signal stating whether the content of one of the editors of the setting tab has changed

            @param has_widget_changed: Boolean stating whether the widget is in a different state as its original
        """
        # Since each object has got an unique name, store it in a dictionary
        self.editor_content_changed[self.sender().objectName()] = has_widget_changed
        # Emits the signal. If any of the children widgets has been changed then it tells that settings have changed
        self.settingsChanged.emit(any(self.editor_content_changed.values()))

    def update_new_checkpoints(self):
        """
            Set the available joint states defined in the corresponding editor
        """
        known_checkpoints = list() if not self.sender().valid_input else self.sender().valid_input.keys()
        self.named_trajectories.set_known_checkpoints(known_checkpoints)

    def update_new_poses(self):
        """
            Set the available poses defined in the corresponding editor
        """
        self.sensor_configs.set_known_poses(self.sender().poses.keys())

    def connect_update(self):
        """
            Connect signals related to the update the settings configuration
        """
        self.named_joint_states.canBeSaved.connect(self.update_config)
        self.named_poses.canBeSaved.connect(self.update_config)
        self.named_trajectories.canBeSaved.connect(self.update_config)
        self.sensor_configs.canBeSaved.connect(self.update_config)
        self.sensor_plugins.canBeSaved.connect(self.update_config)
        self.external_methods.canBeSaved.connect(self.update_config)

    def disconnect_update(self):
        """
            Disconnect signals related to the settings configuration
        """
        self.named_joint_states.canBeSaved.disconnect()
        self.named_poses.canBeSaved.disconnect()
        self.named_trajectories.canBeSaved.disconnect()
        self.sensor_configs.canBeSaved.disconnect()
        self.sensor_plugins.canBeSaved.disconnect()
        self.external_methods.canBeSaved.disconnect()

    def update_config(self, test):
        """
            Update the current settings configuration
        """
        self.configuration[self.sender().objectName()] = self.sender().valid_input
        self.update_validity()
        self.handle_editor_content_signal(test)

    def update_validity(self):
        """
            Update the attribute specifying whether the current configuration is valid
        """
        joint_states, poses = self.configuration["Editor Named joint states"], self.configuration["Editor Named poses"]
        trajectories = self.configuration["Editor Named trajectories"]
        sensor_conf = self.configuration["Editor Sensors config"]
        sensor_plugin = self.configuration["Editor Sensor plugins"]
        methods = self.configuration["Editor High level methods"]
        # If any field has an invalid input
        if any(x is None for x in (joint_states, poses, trajectories, sensor_conf, sensor_plugin, methods)):
            self.is_config_valid = False
            return
        self.is_config_valid = True

    def reset(self):
        """
            Reset the state of this widget to its initial, i.e. all editors closed
        """
        # Make sure we don't have signals conflict at some points
        self.disconnect_update()
        # By default it is valid
        self.is_config_valid = True
        # Reset all the editors
        self.named_joint_states.reset()
        self.named_poses.reset()
        self.named_trajectories.reset()
        self.sensor_configs.reset()
        self.sensor_plugins.reset()
        self.external_methods.reset()
        # Reconnect the udpate signals
        self.connect_update()
        # Configuration of the settings config, set ot its initial values
        self.configuration = SETTINGS_CONFIG.copy()

    def save_config(self, settings):
        """
            Store the state of this widget and its children into settings

            @param settings: QSettings object in which widgets' information are stored
        """
        test = self.objectName()
        settings.beginGroup(test)
        class_name = self.metaObject().className()
        settings.setValue("type", class_name)
        for widget in self.children():
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
