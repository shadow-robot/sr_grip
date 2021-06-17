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

from PyQt5.QtWidgets import QTabWidget, QPushButton
from PyQt5.QtCore import pyqtSignal
from grip_api.config_widgets.robot_interface_widget import RobotInterfaceWidget
from grip_api.config_widgets.hardware_config_widget import HardwareConfigWidget
from grip_api.config_widgets.settings_config_widget import SettingsConfigWidget
from grip_api.utils.common_dialog_boxes import error_message
from grip_api.utils.common_checks import create_yaml_file
from grip_core.launch_file_generator.launch_templater import LaunchFileTemplater
from grip_core.utils.common_paths import API_PATH
import subprocess
import os
from collections import OrderedDict


class RobotIntegrationArea(QTabWidget):

    """
        Widget gathering all the widgets required to integrate a robot to the framework
    """
    # Signal stating whether the robot can be launched or not
    robotCanBeLaunched = pyqtSignal(bool)
    # Signal stating whether the robot can be stopped or not
    robotCanBeStopped = pyqtSignal(bool)
    # Signal stating whether the task editor can be accessible or not
    enableTaskEditor = pyqtSignal(bool)
    # Signal sent when the configuration of commanders have changed
    commanderUpdated = pyqtSignal()

    def __init__(self, parent=None):
        """
            Initialize the class by creating the different widgets

            @param parent: parent of the widget
        """
        super(RobotIntegrationArea, self).__init__(parent=parent)
        # Set the object name to be able to look it up and restore it
        self.setObjectName("Robot integration area")
        self.config_changed = dict()
        self.can_be_saved = False
        self.launch_parameters = dict()
        # Get the configuration of the commanders
        self.commander_config = None
        self.launch_process = None
        self.launch_templater = LaunchFileTemplater()
        self.init_ui()
        self.connect_slots()

    def init_ui(self):
        """
            Create the different widgets and store them into tabs
        """
        # Widget taking care of robot interfacing
        self.robot_interface = RobotInterfaceWidget(self)
        # Widget taking care of configuring the potential robot arm
        self.arm_config_widget = HardwareConfigWidget("Arm", self)
        # Widget taking care of configuring the potential robot hand
        self.hand_config_widget = HardwareConfigWidget("Hand", self)
        # Widget taking care of configuring the experimental environemnt
        self.settings_config_widget = SettingsConfigWidget(self)
        self.addTab(self.robot_interface, "Robot interface")
        self.addTab(self.arm_config_widget, "Arm configuration")
        self.addTab(self.hand_config_widget, "Hand configuration")
        self.addTab(self.settings_config_widget, "Settings")
        # By default the hand and arm tabs are disabled since we don't know the robot's composure
        self.setTabEnabled(1, False)
        self.setTabEnabled(2, False)
        self.setTabEnabled(3, False)
        # Create a button (disabled by default) allowing to launch the configured robot
        self.launch_button = QPushButton("Launch robot", enabled=False)
        # Compute the max height of the widget to get nice visual
        maximum_height = self.launch_button.fontMetrics().boundingRect(self.launch_button.text()).height() + 5
        self.launch_button.setMaximumHeight(maximum_height)
        self.launch_button.clicked.connect(self.launch_robot)
        self.setCornerWidget(self.launch_button)

    def connect_slots(self):
        """
            Remap signals coming from different widgets to have a reactive behavior
        """
        # Update the display according to the simulation check box
        self.robot_interface.simulation_config.check_box.toggled.connect(self.update_simulation_availability)
        # Update the tab view according to the robot_interface's spin box value
        self.robot_interface.robot_config.arm_spin_box.spin_box.valueChanged.connect(self.update_view)
        self.robot_interface.robot_config.hand_spin_box.spin_box.valueChanged.connect(self.update_view)
        self.robot_interface.robot_config.sensor_spin_box.spin_box.valueChanged.connect(self.update_view)
        # Change the content and availability of the MoveIt! related widgets in hardware configuration
        self.robot_interface.moveit_config.moveit_package_entry_widget.inputChanged.connect(self.update_widgets)
        # Update whether something has changed within the widget's children or not
        self.robot_interface.interfaceChanged.connect(self.update_savable)
        self.arm_config_widget.hardwareChanged.connect(self.update_savable)
        self.hand_config_widget.hardwareChanged.connect(self.update_savable)
        self.settings_config_widget.settingsChanged.connect(self.update_savable)
        # Update the display according to the custom launch file user entry
        self.robot_interface.isCustomLaunchProvided.connect(self.update_custom_display)

    def update_savable(self, is_savable=True):
        """
            Change the attribute stating whether children widgets can be saved

            @param is_savable: Boolean specifying whether the widget has been saved
        """
        # Since each object has got an unique name, store it in a dictionary
        self.config_changed[self.sender().objectName()] = is_savable
        # Take into account all the different configs
        self.can_be_saved = any(self.config_changed.values())
        self.update_launch_availability()

    def update_launch_availability(self):
        """
            Make sure the robot can be launched with the current configurations. If everything is good enable the launch
            button otherwise disable it
        """
        is_robot_ok = self.robot_interface.robot_config.is_config_valid
        is_moveit_ok = self.robot_interface.moveit_config.is_config_valid
        is_simu_ok = self.robot_interface.simulation_config.is_config_valid
        # Check that each part of the robot interface config is valid
        if any(not x for x in (is_robot_ok, is_moveit_ok, is_simu_ok)):
            self.launch_button.setEnabled(False)
            self.robotCanBeLaunched.emit(False)
            self.enableTaskEditor.emit(False)
            return
        # Now extract the current configuration and specific fields that will be used to further tests
        robot_config = self.robot_interface.robot_config.configuration
        simulation_config = self.robot_interface.simulation_config.configuration
        has_arm = robot_config["spin arm"]
        has_hand = robot_config["spin hand"]
        has_sensor = robot_config["spin sensor"]
        custom_launch_file = robot_config["UE Custom launch file"]
        is_simu = simulation_config["simu checkbox"]
        is_arm_ok = self.arm_config_widget.is_config_valid
        is_hand_ok = self.hand_config_widget.is_config_valid
        is_settings_ok = self.settings_config_widget.is_config_valid
        # Check that the tabs corresponding to the robot's composition are valid
        if not is_settings_ok or (has_arm and not is_arm_ok) or (has_hand and not is_hand_ok):
            self.launch_button.setEnabled(False)
            self.robotCanBeLaunched.emit(False)
            self.enableTaskEditor.emit(False)
            return
        # Extract the configurations
        arm_config = self.arm_config_widget.configuration
        hand_config = self.hand_config_widget.configuration
        settings_config = self.settings_config_widget.configuration
        # Extract information about the hardware config
        arm_ros_controllers = arm_config["Editor ROS controllers"]
        hand_ros_controllers = hand_config["Editor ROS controllers"]
        arm_ext_control = arm_config["Editor External controllers"]
        hand_ext_control = hand_config["Editor External controllers"]
        arm_connection = arm_config["Editor Arm hardware connection"]
        hand_connection = hand_config["Editor Hand hardware connection"]
        # If a sensor is declared but not configured then disable the button
        if has_sensor and not settings_config["Editor Sensors config"]:
            self.launch_button.setEnabled(False)
            self.robotCanBeLaunched.emit(False)
            self.enableTaskEditor.emit(False)
            return

        # If the robot is meant to be launched in simulation then a ROS controller must be defined for each part
        if is_simu and not custom_launch_file and (has_arm and not arm_ros_controllers or
                                                   has_hand and not hand_ros_controllers):
            self.launch_button.setEnabled(False)
            self.robotCanBeLaunched.emit(False)
            self.enableTaskEditor.emit(False)
            return

        # If the robot is not simulated, then makes sure that we have a way to communicate with it
        if not is_simu and not custom_launch_file and (has_arm and not arm_ext_control and not arm_connection or
                                                       has_hand and not hand_ext_control and not hand_connection):
            self.launch_button.setEnabled(False)
            self.robotCanBeLaunched.emit(False)
            self.enableTaskEditor.emit(False)
            return

        # If everything is good then enable the button
        self.send_commanders_config()
        self.launch_button.setEnabled(True)
        self.robotCanBeLaunched.emit(True)
        self.enableTaskEditor.emit(True)

    def update_widgets(self):
        """
            Update the availability and attributes of all the widgets related to MoveIt!
        """
        # Check whether a custom launch file is being used
        launchfile_not_used = not self.robot_interface.robot_config.configuration["UE Custom launch file"]
        # Check if a valid moveit package has been provided
        should_enable = self.sender().valid_input is not None and self.sender().valid_input != ""
        # Check whether a sensor is part of the setup
        has_sensor = self.robot_interface.robot_config.sensor_spin_box.get_value()
        # Update availability of moveit planners widgets
        self.arm_config_widget.moveit_planners_config.setEnabled(should_enable or not launchfile_not_used)
        self.hand_config_widget.moveit_planners_config.setEnabled(should_enable or not launchfile_not_used)
        # Update the sensor plugin availability
        self.settings_config_widget.sensor_plugins.setEnabled(has_sensor and (should_enable or not launchfile_not_used))
        # If a moveit config package is provided
        if should_enable:
            # Get information from the moveit package
            parsed_controllers, parsed_planners = self.robot_interface.moveit_config.get_parsed_info()
        else:
            # Otherwise turn off the autocompletion and exit the method
            self.settings_config_widget.named_joint_states.code_editor.turn_off_autocompletion()
            return
        # Set the parsed infromation to the proper widgets
        self.arm_config_widget.moveit_planners_config.set_planners_information(parsed_planners)
        self.arm_config_widget.ros_controllers.set_controllers_information(parsed_controllers)
        self.hand_config_widget.moveit_planners_config.set_planners_information(parsed_planners)
        self.hand_config_widget.ros_controllers.set_controllers_information(parsed_controllers)
        # Stack all the joints from the parsed controllers to set the autocompletion for the named joint states
        stacked_joints = []
        for joints in parsed_controllers.values():
            stacked_joints += joints
        # Get unique instance of each joints
        stacked_joints = set(stacked_joints)
        # Set the autocompletion
        self.settings_config_widget.named_joint_states.code_editor.set_autocompletion(stacked_joints)

    def update_simulation_availability(self, is_checked):
        """
            Enables or disables some widgets according to the selected mode (simulation or physical platform)

            @param is_checked: Boolean provided by the signal stating whether the checkbox is ticked or not
        """
        self.arm_config_widget.hardware_connection_config.setEnabled(not is_checked)
        self.hand_config_widget.hardware_connection_config.setEnabled(not is_checked)

    def update_view(self):
        """
            Enables/disables the widgets and tabs according to the state of the spin boxes
        """
        # Modify only the hardware tab view
        # Get the widget that triggers this method
        triggering_widget = self.sender()
        widget_name = triggering_widget.objectName()
        # If the input of the spin box is > 0 then at least tab needs to be enabled
        enable_tab = triggering_widget.value() > 0
        # If the triggering widget is not the sensor's spin box, enables/disables the corresponding tab view
        if widget_name != "sensor":
            tab_index = 1 if widget_name == "arm" else 2
            self.setTabEnabled(tab_index, enable_tab)

        # Is the robot composed of at least one hand or one arm
        one_hardware_is_on = self.isTabEnabled(1) or self.isTabEnabled(2)
        # Is sensor's spin box is not set to 0
        sensor_is_on = self.robot_interface.robot_config.sensor_spin_box.spin_box.value() > 0
        # Here we make sure that regardless of which spin box has been triggered we modify the settings tab
        if one_hardware_is_on or (widget_name == "sensor" and enable_tab):
            self.setTabEnabled(3, True)
        # Equivalent to no spin box activated
        elif not (one_hardware_is_on or sensor_is_on):
            self.setTabEnabled(3, False)
        # Get whether a Moveit config has been provided
        moveit_is_off = not self.robot_interface.moveit_config.moveit_package_entry_widget.valid_input
        # Check whether a custom launch file is being used
        launchfile_not_used = not self.robot_interface.robot_config.configuration["UE Custom launch file"]
        # Here we make sure that regardless of which spin box has been triggered we modify the widgets
        # inside the different tabs
        self.arm_config_widget.set_default_enabled()
        self.hand_config_widget.set_default_enabled()
        self.settings_config_widget.named_poses.setEnabled(one_hardware_is_on)
        self.settings_config_widget.named_joint_states.setEnabled(one_hardware_is_on)
        self.settings_config_widget.named_trajectories.setEnabled(one_hardware_is_on)
        self.settings_config_widget.sensor_configs.setEnabled(sensor_is_on)
        self.settings_config_widget.sensor_plugins.setEnabled(sensor_is_on and (not moveit_is_off or
                                                                                not launchfile_not_used))
        self.settings_config_widget.external_methods.setEnabled(sensor_is_on or one_hardware_is_on)

    def update_custom_display(self, should_enable):
        """
            Update the widget displayed when a custom launch file is provided

            @param should_enable: Boolean stating if a valid launch file has been set by the user
        """
        # Enable/disable the user entries related to simulation
        self.sender().simulation_config.gazebo_file_entry_widget.setEnabled(not should_enable)
        self.sender().simulation_config.gazebo_folder_entry_widget.setEnabled(not should_enable)
        self.sender().simulation_config.starting_pose_entry_widget.setEnabled(not should_enable)
        self.sender().simulation_config.check_box.setEnabled(not should_enable)
        # Update the validity of the simulation section
        self.sender().simulation_config.update_validity()
        # If a launch file is provided
        if should_enable:
            # Enable all the widgets that can be edited in this mode
            self.arm_config_widget.hardware_connection_config.setEnabled(should_enable)
            self.hand_config_widget.hardware_connection_config.setEnabled(should_enable)
            self.arm_config_widget.moveit_planners_config.setEnabled(should_enable)
            self.hand_config_widget.moveit_planners_config.setEnabled(should_enable)
            self.sender().robot_config.collision_scene_entry_widget.setEnabled(should_enable)
            self.settings_config_widget.sensor_plugins.setEnabled(should_enable)
        # If the launch file has been removed
        else:
            # Get the current configurations
            simulation_on = self.sender().simulation_config.check_box.isChecked()
            moveit_off = not self.sender().moveit_config.configuration["UE Moveit package"]
            sensor_on = self.sender().robot_config.sensor_spin_box.get_value() > 0
            # Depending on the current configuration update the proper state of the different widgets
            self.arm_config_widget.hardware_connection_config.setEnabled(not simulation_on)
            self.hand_config_widget.hardware_connection_config.setEnabled(not simulation_on)
            self.sender().robot_config.collision_scene_entry_widget.setEnabled(not simulation_on)
            self.settings_config_widget.sensor_plugins.setEnabled(not moveit_off and sensor_on)
            self.arm_config_widget.moveit_planners_config.setEnabled(not moveit_off)
            self.hand_config_widget.moveit_planners_config.setEnabled(not moveit_off)

    def launch_robot(self):
        """
            Extract all required information from the configuration and launch the robot
        """
        # Make sure we don't already have a robot running
        if self.launch_process is not None:
            error_message("Cannot launch the robot", "A robot is already running!\n",
                          additional_text="Please first stop the robot", parent=self)
            return
        # Extract the paraneters from the configuration
        self.extract_launch_parameters()
        # Generate the launch file
        self.launch_templater.generate_launch_file(self.launch_parameters)
        # Use a different thread to launch the robot
        try:
            self.launch_process = subprocess.Popen(
                ['roslaunch grip_core generated_framework_launch.launch'], shell=True)
            self.launch_button.setEnabled(False)
            self.robotCanBeLaunched.emit(False)
            self.robotCanBeStopped.emit(True)
        except Exception as exception:
            print("Exception is {}".format(exception))
            return

    def stop_robot(self):
        """
            If a robot is running, terminates the thread but keep the GUI alive
        """
        if self.launch_process is not None:
            self.launch_process.terminate()
            self.launch_process = None
            self.robotCanBeStopped.emit(False)
            self.update_launch_availability()

    def extract_launch_parameters(self):
        """
            Extract the different values required to fill the launch file template and store them in a dictionary
        """
        self.launch_parameters["robot_name"] = self.robot_interface.robot_config.get_robot_name()
        self.launch_parameters["urdf_file"] = self.robot_interface.robot_config.configuration["UE Robot's URDF file"]
        custom_launch_file = self.robot_interface.robot_config.configuration["UE Custom launch file"]
        self.launch_parameters["launch_file_path"] = custom_launch_file
        self.launch_parameters["launch_file_configuration"] = self.robot_interface.robot_config.get_launch_config()
        self.launch_parameters["simulation"] = self.robot_interface.simulation_config.configuration["simu checkbox"]
        moveit_package = self.robot_interface.moveit_config.configuration["UE Moveit package"]
        self.launch_parameters["is_using_moveit"] = moveit_package
        # Update the format just to be prettier once the template is rendered
        moveit_config = self.robot_interface.moveit_config.get_moveit_config("move_group").replace("\n", "\n  ")
        rviz_config = self.robot_interface.moveit_config.get_moveit_config("moveit_rviz").replace("\n", "\n  ")
        self.launch_parameters["moveit_configuration"] = moveit_config
        self.launch_parameters["rviz_configuration"] = rviz_config
        self.launch_parameters["ros_controllers"] = self.get_fused_ros_controllers()
        self.launch_parameters["sensors_config_path"] = self.settings_config_widget.sensor_configs.file_path
        self.launch_parameters["sensor_launch_files"] = self.extract_sensor_launch_files()
        # Get the paths of pre-recoded elements
        recorded_joint_states_file_path = self.settings_config_widget.named_joint_states.file_path
        recorded_joint_states = "" if not recorded_joint_states_file_path else recorded_joint_states_file_path
        recorded_trajectories_file_path = self.settings_config_widget.named_trajectories.file_path
        recorded_trajectories = "" if not recorded_trajectories_file_path else recorded_trajectories_file_path
        recorded_poses_file_path = self.settings_config_widget.named_poses.file_path
        recorded_poses = "" if not recorded_poses_file_path else recorded_poses_file_path
        self.launch_parameters["recorded_joint_state_path"] = recorded_joint_states
        self.launch_parameters["recorded_trajectories_path"] = recorded_trajectories
        self.launch_parameters["recorded_poses"] = recorded_poses
        # Get the valid input of each external component
        for hardware_widget in (self.arm_config_widget, self.hand_config_widget):
            hardware_part = hardware_widget.hardware_part.lower()
            controller = hardware_widget.external_controller.components
            motion_planner = hardware_widget.external_motion_planner.components
            kinematics = hardware_widget.kinematic_libraries_config.components
            self.launch_parameters["{}_external_controller".format(hardware_part)] = controller
            self.launch_parameters["{}_external_motion_planner".format(hardware_part)] = motion_planner
            self.launch_parameters["{}_external_kinematics".format(hardware_part)] = kinematics
        # Get the valid input of integrated external methods
        self.launch_parameters["external_high_level_methods"] = self.settings_config_widget.external_methods.components
        # Get the path of the MoveIt! sensor plugins
        plugins_path = self.settings_config_widget.sensor_plugins.file_path
        plugins_path = "" if not plugins_path else plugins_path
        self.launch_parameters["sensor_plugin"] = plugins_path

        # Simulation/real robot specific parameters
        if self.launch_parameters["simulation"] and not custom_launch_file:
            gazebo_world_file = self.robot_interface.simulation_config.configuration["UE Gazebo world file"]
            self.launch_parameters["world_file"] = gazebo_world_file
            model_path = self.robot_interface.simulation_config.configuration["UE Gazebo model folder"]
            # Makes sure to get the required trailing "/"
            self.launch_parameters["gazebo_model_path"] = os.path.join(model_path, "")
            simu_starting_pose = self.robot_interface.simulation_config.configuration["UE Starting pose"]
            self.launch_parameters["starting_pose"] = simu_starting_pose
        elif custom_launch_file or not self.launch_parameters["simulation"]:
            # Get parameters related to non simulated robots
            self.launch_parameters["hardware_connection"] = self.get_fused_hardware_connection()
            self.launch_parameters["scene"] = self.robot_interface.robot_config.configuration["UE Collision scene"]

    def extract_sensor_launch_files(self):
        """
            Get the path of the launch files to include in the generated launch file to run the configured sensors
        """
        # Initialize the list that is going to be returned
        sensor_launch_files = list()
        # Get the configuration
        configured_sensors = self.settings_config_widget.sensor_configs.valid_input
        # If at least a sensor has been configured
        if configured_sensors:
            # Go over all the configured sensors
            for sensor_params in configured_sensors.values():
                # Store the "launch_file" field if present
                if "launch_file" in sensor_params:
                    sensor_launch_files.append(os.path.abspath(sensor_params["launch_file"]))
        # Return the list
        return sensor_launch_files

    def get_fused_ros_controllers(self):
        """
            Generate the final version of the ROS controller file

            @return: Path of the yaml file containing all the ROS controllers to be loaded
        """
        # Extract both ROS controllers
        arm_widget = self.arm_config_widget
        hand_widget = self.hand_config_widget
        # Will contain the final ros controllers
        fused_ros_controllers = ""
        # Merge both ROS controllers
        controller_index = 0
        for controller in (arm_widget, hand_widget):
            # If not ROS controller is defined then go to the next controller
            if not controller.configuration["Editor ROS controllers"]:
                continue
            controller_text = controller.ros_controllers.code_editor.text()
            fused_ros_controllers += "\n\n" + controller_text if controller_index else controller_text
            controller_index += 1
        # Defined the path where the fused controllers will be saved
        fused_ros_controllers_file_path = os.path.join(API_PATH, "config", "merged_ros_controllers.yaml")
        # Create the file
        with open(fused_ros_controllers_file_path, "w") as file_:
            file_.write(fused_ros_controllers)
        # Return the path to the file
        return fused_ros_controllers_file_path

    def get_fused_hardware_connection(self):
        """
            Generate the final version of the ROS robot hardware connection file
        """
        # Extract hardware config widgets
        arm_widget = self.arm_config_widget
        hand_widget = self.hand_config_widget
        # Get the hardware connection valid input
        arm_connection = arm_widget.configuration["Editor Arm hardware connection"]
        hand_connection = hand_widget.configuration["Editor Hand hardware connection"]
        # If no hardware connection is set then stop here
        if all(not x for x in (arm_connection, hand_connection)):
            return
        # Will contain the final hardware connection
        fused_hardware_connection = OrderedDict()
        # Set the robot_hardware field
        fused_hardware_connection["robot_hardware"] = list()
        # Merge both files
        for connection in (arm_connection, hand_connection):
            # If no hardware connection is set for the arm go for the hand
            if not connection:
                continue
            # Gather in the final dictionary every field
            for element_name, element_value in connection.items():
                if element_name == "robot_hardware":
                    # "robot_hardware" should be a list of all the robot hw to run
                    fused_hardware_connection["robot_hardware"].extend(element_value)
                else:
                    fused_hardware_connection[element_name] = element_value

        # Defined the path where the fused hardware connection will be saved
        fused_ros_connection_file_path = os.path.join(API_PATH, "config", "merged_ros_hardware_connection.yaml")
        # Create the file
        create_yaml_file(fused_hardware_connection, fused_ros_connection_file_path)
        # Return the path to the file
        return fused_ros_connection_file_path

    def send_commanders_config(self):
        """
            Extract the commander configuration and emit a signal when it's done
        """
        # If not moveit config package is provided then no commander config should be set
        if not self.robot_interface.moveit_config.moveit_package_entry_widget.valid_input:
            updated_commander_config = None
        else:
            updated_commander_config = dict()
            arm_config = self.arm_config_widget.configuration
            hand_config = self.hand_config_widget.configuration
            # For each piece of hardware get the MoveIt planner config
            for hw_config in (arm_config, hand_config):
                planner_config = hw_config["Editor MoveIt! planners"]
                # If it contains at least one planner, go over all of them and get the configuration
                if not not planner_config:
                    for group_name, commander_config in planner_config.items():
                        updated_commander_config[str(group_name)] = commander_config
        # Update the attribute to know if the commander has been updated or not
        if updated_commander_config != self.commander_config:
            self.commander_config = updated_commander_config
            self.commanderUpdated.emit()

    def save_config(self, settings):
        """
            Store the state of this widget and its children into settings

            @param settings: QSettings object in which widgets' information are stored
        """
        settings.beginGroup(self.objectName())
        class_name = self.metaObject().className()
        settings.setValue("type", class_name)
        for tab_index in range(self.count()):
            widget_in_tab = self.widget(tab_index)
            widget_in_tab.save_config(settings)
        settings.endGroup()
        self.can_be_saved = False

    def restore_config(self, settings):
        """
            Restore the children's widget from the configuration saved in settings

            @param settings: QSettings object that contains information of the widgets to restore
        """
        settings.beginGroup(self.objectName())
        for tab_index in range(self.count()):
            widget_in_tab = self.widget(tab_index)
            widget_in_tab.restore_config(settings)
        settings.endGroup()
        # Make sure to update the commanders config
        self.send_commanders_config()
        # If removed then will always ask to save even though nothing has been modified
        self.config_changed = dict()
        self.can_be_saved = False
