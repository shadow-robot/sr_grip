#!/usr/bin/env python3

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

import rospkg
import os

# Framework packages
API_PATH = rospkg.RosPack().get_path("grip_api")
CORE_PATH = rospkg.RosPack().get_path("grip_core")

# Built in template folders
BUILT_IN_TEMPLATES_FOLDER = os.path.join(CORE_PATH, "templates")
FOLDER_TEMPLATE_LAUNCH_FILE = os.path.join(BUILT_IN_TEMPLATES_FOLDER, "launch_file")
STATE_MACHINES_TEMPLATES_FOLDER = os.path.join(BUILT_IN_TEMPLATES_FOLDER, "state_machines")
BASE_STATE_MACHINE_FOLDER = os.path.join(BUILT_IN_TEMPLATES_FOLDER, "base")
TASK_EDITOR_ROOT_TEMPLATE = os.path.join(BASE_STATE_MACHINE_FOLDER, "base_state_machine.template")
STATE_TEMPLATES_FOLDER = os.path.join(BUILT_IN_TEMPLATES_FOLDER, "states")

# GUI configuration
GUI_CONFIGS_FOLDER = os.path.join(API_PATH, "gui_configs")
MAIN_CONFIG_FILE = os.path.join(GUI_CONFIGS_FOLDER, "gui.ini")
ROBOT_CONFIG_FOLDER = os.path.join(GUI_CONFIGS_FOLDER, "robot_config")
TASK_CONFIG_FOLDER = os.path.join(GUI_CONFIGS_FOLDER, "task_config")
DEFAULT_ROBOT_CONFIG_FILE = os.path.join(ROBOT_CONFIG_FOLDER, "robot_config.ini")
DEFAULT_TASK_CONFIG_FILE = os.path.join(TASK_CONFIG_FOLDER, "task_config.ini")

# Catkin workspace
CATKIN_WS = "/home/user/projects/shadow_robot/base/src"

# Robot launch
LAUNCH_CORE_FOLDER = os.path.join(CORE_PATH, "launch")

# Icon resources
IMAGES = os.path.join(API_PATH, "resources")
RED_CIRCLE = os.path.join(IMAGES, "red_icon.png")
GREEN_CIRCLE = os.path.join(IMAGES, "green_icon.png")
STATE_MACHINE_ICON = os.path.join(IMAGES, "state_machine_icon.png")
STATE_ICON = os.path.join(IMAGES, "state_icon.png")

# States provided by the framework
INTERNAL_SRC_CORE = os.path.join(CORE_PATH, "src", "grip_core")
STATES_FOLDER = os.path.join(INTERNAL_SRC_CORE, "states")
COMMANDER_FOLDER = os.path.join(STATES_FOLDER, "commander")

# Generated state machines
GENERATED_STATE_MACHINE_FOLDER = os.path.join(CORE_PATH, "src", "grip_core", "generated_state_machines")
GENERATED_STATES_FOLDER = os.path.join(CORE_PATH, "src", "grip_core", "generated_states")

EXTERNAL_COMPONENT_TEMPLATE = os.path.join(STATE_TEMPLATES_FOLDER, "external_component.template")
SENSOR_TEMPLATE = os.path.join(STATE_TEMPLATES_FOLDER, "sensor.template")
