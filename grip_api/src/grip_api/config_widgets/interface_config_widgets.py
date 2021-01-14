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

from PyQt5.QtWidgets import QWidget, QGridLayout, QLabel, QSpinBox, QHBoxLayout, QCheckBox
from PyQt5.QtCore import pyqtSignal
from plain_editor_widgets import XMLEditorWidget
import user_entry_widgets as uew
from grip_api.utils.files_specifics import SIMU_CONFIG, INTERFACE_CONFIG, MOVEIT_CONFIG
import os
import re
import yaml


class GenericInterfaceConfigWidget(QWidget):

    """
        Generic widget gathering common properties of widgets used to interface a robot
    """

    def __init__(self, name, parent=None):
        """
            Initialize the class by setting up the layout and initializing the different attributes

            @param name: Name given to the object. Used to look it up when restoring a configuration
            @param parent: parent of the widget
        """
        super(GenericInterfaceConfigWidget, self).__init__(parent=parent)
        self.setObjectName(name)
        self.init_ui()

    def init_ui(self):
        """
            Set up the layout
        """
        self.layout = QGridLayout()
        # These values prevent the different elements not to be stuck to the tab
        self.layout.setContentsMargins(5, 15, 5, 5)
        self.setLayout(self.layout)


class SimulationConfig(GenericInterfaceConfigWidget):

    """
        Widget allowing to set up the simulation parameters
    """
    simuModeChanged = pyqtSignal(bool)

    def __init__(self, parent=None):
        """
            Initialize the class by creating the different user entries

            @param parent: parent of the widget
        """
        super(SimulationConfig, self).__init__("Simulation parameters", parent=parent)
        self.initial_checked = True
        # Configuration of the simulation
        self.configuration = SIMU_CONFIG
        # By default it is not valid
        self.is_config_valid = False
        self.initialize_content()
        self.connect_update()

    def initialize_content(self):
        """
            Create and add the user entries allowing to set up the simulation mode
        """
        # Add a check box to specify whether the simulation mode should be activated
        self.check_box = QCheckBox("Simulation", objectName="simu checkbox")
        self.check_box.setChecked(self.initial_checked)
        self.layout.addWidget(self.check_box, 0, 0)
        self.gazebo_file_entry_widget = uew.GazeboWorldEntryWidget(parent=self)
        self.gazebo_folder_entry_widget = uew.GazeboFolderEntryWidget(parent=self)
        self.starting_pose_entry_widget = uew.StartingPoseEntryWidget(browser_button=False,
                                                                      placeholder_text="-J shoulder_pan_joint 0.5 "
                                                                      "-J shoulder_lift_joint 1.5 -J ...", parent=self)
        self.layout.addWidget(self.gazebo_file_entry_widget)
        self.layout.addWidget(self.gazebo_folder_entry_widget)
        self.layout.addWidget(self.starting_pose_entry_widget)

    def state_changed(self, checked):
        """
            Triggers a signal if the checkbox ends up in a different state than the initial one

            @param checked: Boolean stating whether the box is checked or not
        """
        self.simuModeChanged.emit(checked != self.initial_checked)

    def connect_update(self):
        """
            Connect signals to a slot allowing to update the simulation configuration
        """
        self.check_box.toggled.connect(self.update_config)
        self.gazebo_file_entry_widget.canBeSaved.connect(self.update_config)
        self.gazebo_folder_entry_widget.canBeSaved.connect(self.update_config)
        self.starting_pose_entry_widget.canBeSaved.connect(self.update_config)

    def update_config(self, is_checked):
        """
            Update the current simulation configuration

            @param is_checked: Boolean stating whether the box is checked or not
        """
        valid_input = self.sender().isChecked() if self.sender() is self.check_box else self.sender().valid_input
        self.configuration[self.sender().objectName()] = valid_input
        self.update_validity()
        if isinstance(valid_input, bool):
            self.state_changed(is_checked)

    def update_validity(self):
        """
            Update the attribute specifying whether the current configuration is valid
        """
        simu, world = self.configuration["simu checkbox"], self.configuration["UE Gazebo world file"]
        folder, pose = self.configuration["UE Gazebo model folder"], self.configuration["UE Starting pose"]
        if simu and self.check_box.isEnabled() and any(x is None for x in (world, folder, pose)):
            self.is_config_valid = False
            return
        self.is_config_valid = True

    def save_config(self, settings):
        """
            Store the state of this widget and its children into settings

            @param settings: QSettings object in which widgets' information are stored
        """
        settings.beginGroup(self.objectName())
        current_state = self.check_box.isChecked()
        settings.setValue("is_checked", current_state)
        self.initial_checked = current_state
        for widget in (self.gazebo_file_entry_widget, self.gazebo_folder_entry_widget, self.starting_pose_entry_widget):
            widget.save_config(settings)
        settings.endGroup()

    def restore_config(self, settings):
        """
            Restore the children's widget from the configuration saved in settings

            @param settings: QSettings object that contains information of the widgets to restore
        """
        settings.beginGroup(self.objectName())
        state_to_set = settings.value("is_checked", type=bool)
        self.initial_checked = state_to_set
        self.check_box.setChecked(state_to_set)
        for widget in (self.gazebo_file_entry_widget, self.gazebo_folder_entry_widget, self.starting_pose_entry_widget):
            widget.restore_config(settings)
        settings.endGroup()


class MoveitConfig(GenericInterfaceConfigWidget):

    """
        Widget allowing the user to specify the optional MoveIt! configuration of a robot
    """

    def __init__(self, parent=None):
        """
            Initialize the class by creating the different user entries

            @param parent: parent of the widget
        """
        super(MoveitConfig, self).__init__("Moveit parameters", parent=parent)
        # Configuration of the moveit interface
        self.configuration = MOVEIT_CONFIG
        # By default it is not valid
        self.is_config_valid = True
        self.initialize_content()
        self.connect_update()
        self.moveit_package_entry_widget.inputChanged.connect(self.update_xml_editors)

    def initialize_content(self):
        """
            Create and add the user entries allowing to set up the MoveIt! configuration
        """
        self.moveit_package_entry_widget = uew.MoveitPackageEntryWidget("Moveit package", parent=self)
        self.move_group_editor = XMLEditorWidget("Move group arguments (optional)", enabled=False, parent=self)
        self.rviz_editor = XMLEditorWidget("RViz arguments (optional)", enabled=False, parent=self)
        self.layout.addWidget(self.moveit_package_entry_widget)
        self.layout.addWidget(self.move_group_editor)
        self.layout.addWidget(self.rviz_editor)

    def setup_editors(self):
        """
            Setup the editors allowing to modify the move group and rviz launch files
        """
        # If the provided package is valid then make the editors enabled
        self.move_group_editor.setEnabled(True)
        self.rviz_editor.setEnabled(True)
        # Display the skeleton helping the user to change options of some of the moveit launch files
        self.move_group_editor.set_editor_content(self.get_moveit_config("move_group"))
        self.rviz_editor.set_editor_content(self.get_moveit_config("moveit_rviz"))

    def update_xml_editors(self):
        """
            Method changing the editors' content according to provided MoveIt! package
        """
        if self.moveit_package_entry_widget.valid_input:
            self.setup_editors()
        else:
            self.move_group_editor.code_editor.reinitialize()
            self.rviz_editor.code_editor.reinitialize()
            self.move_group_editor.setEnabled(False)
            self.rviz_editor.setEnabled(False)

    def get_moveit_config(self, filename):
        """
            Returns the configuration for a given editor according to its filename

            @param filename: String specifying which editor it refers to
            @return: String corresponding to what will be added to the generated launch file
        """
        if not self.moveit_package_entry_widget.valid_input:
            return self.moveit_package_entry_widget.valid_input

        package_name = self.moveit_package_entry_widget.valid_input[-1]

        if filename == "move_group":
            arguments = self.move_group_editor.get_formated_arguments()
        elif filename == "moveit_rviz":
            arguments = self.rviz_editor.get_formated_arguments()
        else:
            return

        if arguments is None:
            arguments = ""
        else:
            arguments = "  " + arguments

        return "<include file=\"$(find {})/launch/{}.launch\">\n  "\
               "<!-- You can add any options you want to the file -->\n{}</include>".format(package_name, filename,
                                                                                            arguments)

    def get_parsed_info(self):
        """
            Returns information about MoveIt! controllers and planners

            @return: Two dictionaries containing information about controllers and planners set in the MoveIt! package
        """
        # Just makes sure we can proceed
        if self.moveit_package_entry_widget.valid_input is None:
            return

        package_path = self.moveit_package_entry_widget.valid_input[0]
        # Initialize a dictionary with an empty string as key. It is useful to let the user input another controller
        controllers_info = {"": ""}
        assumed_moveit_controller_file_path = os.path.join(package_path, "config", "controllers.yaml")
        # Parse the name of the controllers used by MoveIt! (must be ROS controllers), if part of the provided package
        if os.path.exists(assumed_moveit_controller_file_path):
            with open(assumed_moveit_controller_file_path, "r") as file_:
                moveit_controllers = yaml.safe_load(file_)
            for controller in moveit_controllers["controller_list"]:
                controllers_info[controller["name"]] = controller["joints"]

        planning_groups_info = dict()
        assumed_available_planners_file = os.path.join(package_path, "config", "ompl_planning.yaml")
        # Get the name of the planning groups as well as their list of possible planners, if the correpsonding file is
        # part of the package
        if os.path.exists(assumed_available_planners_file):
            with open(assumed_available_planners_file, "r") as file_:
                planning_groups = yaml.safe_load(file_)
            for group_name, config in planning_groups.items():
                if group_name != "planner_configs":
                    planning_groups_info[group_name] = config["planner_configs"]

        return controllers_info, planning_groups_info

    def connect_update(self):
        """
            Connect signals to a slot allowing to update the MoveIt! configuration
        """
        self.moveit_package_entry_widget.canBeSaved.connect(self.update_config)
        self.move_group_editor.canBeSaved.connect(self.update_config)
        self.rviz_editor.canBeSaved.connect(self.update_config)

    def update_config(self):
        """
            Update the current MoveIt! interface configuration
        """
        self.configuration[self.sender().objectName()] = self.sender().valid_input
        self.update_validity()

    def update_validity(self):
        """
            Update the attribute specifying whether the current configuration is valid
        """
        package = self.configuration["UE Moveit package"]
        move_group = self.configuration["Editor Move group arguments (optional)"]
        rviz = self.configuration["Editor RViz arguments (optional)"]
        # If the user entry has an invalid input
        if package is None:
            self.is_config_valid = False
            return
        # If arguments provided in the editor are wrong
        if package and (move_group is None or rviz is None):
            self.is_config_valid = False
            return
        self.is_config_valid = True

    def save_config(self, settings):
        """
            Store the state of this widget into settings

            @param settings: QSettings object in which widgets' information are stored
        """
        settings.beginGroup(self.objectName())
        for widget in self.children():
            if not (isinstance(widget, QLabel) or isinstance(widget, QGridLayout)):
                widget.save_config(settings)
        settings.endGroup()

    def restore_config(self, settings):
        """
            Restore the children's widget from the configuration saved in settings

            @param settings: QSettings object that contains information of the widgets to restore
        """
        settings.beginGroup(self.objectName())
        for widget in self.children():
            if not (isinstance(widget, QLabel) or isinstance(widget, QGridLayout)):
                widget.restore_config(settings)
        settings.endGroup()


class RobotInterfaceConfig(GenericInterfaceConfigWidget):

    """
        Widget allowing the user to interface robot
    """

    def __init__(self, parent=None):
        """
            Initialize the class by creating the different user entries

            @param parent: parent of the widget
        """
        super(RobotInterfaceConfig, self).__init__("Robot interface", parent=parent)
        # Configuration of the interface
        self.configuration = INTERFACE_CONFIG
        # By default it is not valid
        self.is_config_valid = False
        self.initialize_content()
        self.connect_update()
        self.launch_file_entry_widget.inputChanged.connect(self.update_xml_editor)

    def initialize_content(self):
        """
            Create and set the different entries composing this widget
        """
        self.robot_urdf_entry_widget = uew.UrdfEntryWidget(parent=self)
        self.urdf_args_entry_widget = uew.UrdfArgumentsEntryWidget(browser_button=False, parent=self)
        self.launch_file_entry_widget = uew.LaunchFileEntryWidget(browser_button=True, parent=self)
        self.launch_file_editor = XMLEditorWidget("Launch file arguments (optional)", parent=self)
        self.collision_scene_entry_widget = uew.CollisionFileEntryWidget("Collision scene", enabled=False, parent=self)
        self.layout.addWidget(self.robot_urdf_entry_widget)
        self.layout.addWidget(self.urdf_args_entry_widget)
        self.layout.addWidget(self.launch_file_entry_widget)
        self.layout.addWidget(self.launch_file_editor)
        self.layout.addWidget(self.collision_scene_entry_widget)
        self.add_robot_composition()

    def add_robot_composition(self):
        """
            Add three custom made spin boxes allowing to specify what the robot is composed of
        """
        horizontal_layout = QHBoxLayout()
        horizontal_layout.addWidget(QLabel("The robot is composed of: "))
        self.arm_spin_box = HardwareSpinBox("arm", self)
        self.hand_spin_box = HardwareSpinBox("hand", self)
        self.sensor_spin_box = HardwareSpinBox("sensor", self)
        horizontal_layout.addWidget(self.arm_spin_box)
        horizontal_layout.addWidget(self.hand_spin_box)
        horizontal_layout.addWidget(self.sensor_spin_box)
        self.layout.addLayout(horizontal_layout, 5, 0, 1, 1)

    def setup_editor(self):
        """
            Setup the editor allowing to modify the provided launch file
        """

        self.launch_file_editor.setEnabled(True)
        self.launch_file_editor.set_editor_content(self.get_launch_config())

    def update_xml_editor(self):
        """
            Method changing the editor's content according to provided launch file path
        """
        if self.launch_file_entry_widget.valid_input:
            self.setup_editor()
        else:
            self.launch_file_editor.code_editor.reinitialize()
            self.launch_file_editor.setEnabled(False)

    def get_robot_name(self):
        """
            Extract the robot name out of the provided URDF file

            @return: String containing the name of the robot
        """
        with open(self.robot_urdf_entry_widget.valid_input, "r") as f:
            urdf_file_content = "".join(f.readlines())
        robot_name = re.search("<robot .*? name=\"(.*?)\">", urdf_file_content, re.DOTALL).group(1)
        return robot_name

    def get_launch_config(self):
        """
            Returns the configuration according to the editor's content

            @return: String corresponding to what will be added to the generated launch file
        """
        if not self.launch_file_entry_widget.valid_input:
            return self.launch_file_entry_widget.valid_input

        launch_file_path, package_name = self.launch_file_entry_widget.valid_input
        arguments = self.launch_file_editor.get_formated_arguments()
        ind = launch_file_path.split("/").index(package_name)
        # Not sure the strip is needed
        test = "/" + "/".join(launch_file_path.split("/")[ind + 1:]).strip("/")
        if arguments is None:
            arguments = ""
        else:
            arguments = "  " + arguments

        return "<include file=\"$(find {}){}\">\n  "\
               "<!-- You can add any options you want to the file -->\n{}</include>".format(package_name,
                                                                                            test,
                                                                                            arguments)

    def connect_update(self):
        """
            Connect signals to a slot allowing to update the robot interface configuration
        """
        self.robot_urdf_entry_widget.canBeSaved.connect(self.update_config)
        self.urdf_args_entry_widget.canBeSaved.connect(self.update_config)
        self.launch_file_entry_widget.canBeSaved.connect(self.update_config)
        self.launch_file_editor.canBeSaved.connect(self.update_config)
        self.collision_scene_entry_widget.canBeSaved.connect(self.update_config)
        self.arm_spin_box.inputChanged.connect(self.update_config)
        self.hand_spin_box.inputChanged.connect(self.update_config)
        self.sensor_spin_box.inputChanged.connect(self.update_config)

    def update_config(self):
        """
            Update the current robot interface configuration
        """
        sender = self.sender()
        valid_input = sender.get_value() if isinstance(sender, HardwareSpinBox) else sender.valid_input
        self.configuration[self.sender().objectName()] = valid_input
        self.update_validity()

    def update_validity(self):
        """
            Update the attribute specifying whether the current configuration is valid
        """
        urdf, urdf_args = self.configuration["UE Robot's URDF file"], self.configuration["UE URDF arguments (optional)"]
        launch = self.configuration["UE Custom launch file"]
        launch_args = self.configuration["Editor Launch file arguments (optional)"]
        collision_scene = self.configuration["UE Collision scene"]
        arm, hand = self.configuration["spin arm"], self.configuration["spin hand"]
        sensor = self.configuration["spin sensor"]
        # If any user entry has an invalid input
        if any(x is None for x in (urdf, urdf_args, launch, collision_scene)):
            self.is_config_valid = False
            return
        # If arguments provided in the editor are wrong
        if launch and launch_args is None:
            self.is_config_valid = False
            return
        # If no part have been added
        if arm + hand + sensor == 0:
            self.is_config_valid = False
            return
        self.is_config_valid = True

    def save_config(self, settings):
        """
            Store the state of this widget and its children into settings

            @param settings: QSettings object in which widgets' information are stored
        """
        settings.beginGroup(self.objectName())
        for widget in self.children():
            if not (isinstance(widget, QLabel) or isinstance(widget, QHBoxLayout) or isinstance(widget, QGridLayout)):
                widget.save_config(settings)
        settings.endGroup()

    def restore_config(self, settings):
        """
            Restore the children's widget from the configuration saved in settings

            @param settings: QSettings object that contains information of the widgets to restore
        """
        settings.beginGroup(self.objectName())
        for widget in self.children():
            if not (isinstance(widget, QLabel) or isinstance(widget, QHBoxLayout) or isinstance(widget, QGridLayout)):
                widget.restore_config(settings)
        settings.endGroup()


class HardwareSpinBox(QWidget):

    """
        Widget containing a label and a spin box
    """
    inputChanged = pyqtSignal(bool)

    def __init__(self, name, parent=None):
        """
            Initialize the class by creating the layout and wisgets

            @param name: text to write after the spin box
            @param parent: parent of the widget
        """
        super(HardwareSpinBox, self).__init__(parent=parent)
        self.setObjectName("spin {}".format(name))
        self.init_ui()
        self.original_text = name
        self.initial_value = 0
        self.create_widgets()

    def init_ui(self):
        """
            Set up the layout
        """
        self.layout = QHBoxLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(self.layout)

    def create_widgets(self):
        """
            Creates and adds to the layout both the spin box and label
        """
        self.spin_box = QSpinBox(self, objectName=self.original_text)
        self.spin_box.setMaximumWidth(45)
        self.label_text = QLabel(self.original_text)
        self.spin_box.valueChanged.connect(self.make_plural)
        self.spin_box.valueChanged.connect(self.signal_if_different)
        self.layout.addWidget(self.spin_box)
        self.layout.addWidget(self.label_text)

    def make_plural(self):
        """
            Make the label plural if the value of the spin box is greater than 1
        """
        if self.spin_box.value() > 1:
            self.label_text.setText(self.original_text + "s")
        else:
            self.label_text.setText(self.original_text)

    def signal_if_different(self):
        """
            Emit a signal if the current spin box value is different than the initial
        """
        self.inputChanged.emit(self.spin_box.value() != self.initial_value)

    def get_value(self):
        """
            Returns the value of the spin boxes

            @return: Value (int) of the spin box
        """
        return self.spin_box.value()

    def set_value(self, value):
        """
            Sets the value of the spin box

            @param value: Value to set to the spin box (int)
        """
        self.spin_box.setValue(value)

    def save_config(self, settings):
        """
            Store the state of this widget into settings

            @param settings: QSettings object in which widgets' information are stored
        """
        settings.beginGroup(self.objectName())
        settings.setValue("spin_value", self.get_value())
        settings.endGroup()
        self.initial_value = self.get_value()

    def restore_config(self, settings):
        """
            Restore this widget's configuration from settings

            @param settings: QSettings object that contains information of the widgets to restore
        """
        settings.beginGroup(self.objectName())
        value = settings.value("spin_value", type=int)
        self.initial_value = value
        self.set_value(value)
        settings.endGroup()
