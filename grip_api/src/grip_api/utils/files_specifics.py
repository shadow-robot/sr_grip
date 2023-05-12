#!/usr/bin/env python3

# Copyright 2020, 2023 Shadow Robot Company Ltd.
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

from PyQt5.QtGui import QColor

FILE_TO_EXTENSION = {"Gazebo world file": ["gazebo world", "world", ".world"],
                     "Robot's URDF file": ["robot's urdf", "urdf", ".urdf.xacro"],
                     "Controller file": ["controller", "YAML", ".yaml"], "Moveit package": [],
                     "Gazebo model folder": [], "Collision scene": ["collision", "scene", ".scene"],
                     "Custom launch file": ["launch", "launch", ".launch"]}

INTERFACE_CONFIG = {"UE Robot's URDF file": None, "UE URDF arguments (optional)": "", "UE Custom launch file": "",
                    "Editor Launch file arguments (optional)": None, "UE Collision scene": "", "spin arm": 0,
                    "spin hand": 0, "spin sensor": 0}

SIMU_CONFIG = {"simu checkbox": True, "UE Gazebo world file": None, "UE Gazebo model folder": None,
               "UE Starting pose": ""}

MOVEIT_CONFIG = {"UE Moveit package": "", "Editor Move group arguments (optional)": None,
                 "Editor RViz arguments (optional)": None}

ARM_CONFIG = {"Editor Arm hardware connection": dict(), "Editor ROS controllers": dict(),
              "Editor MoveIt! planners": dict(), "Editor External kinematics": dict(),
              "Editor External controllers": dict(), "Editor External Motion Planners": dict()}

HAND_CONFIG = {"Editor Hand hardware connection": dict(), "Editor ROS controllers": dict(),
               "Editor MoveIt! planners": dict(), "Editor External kinematics": dict(),
               "Editor External controllers": dict(), "Editor External Motion Planners": dict()}

SETTINGS_CONFIG = {"Editor Named joint states": dict(), "Editor Named poses": dict(),
                   "Editor Named trajectories": dict(), "Editor Sensors config": dict(),
                   "Editor Sensor plugins": dict(), "Editor High level methods": dict()}

LISTITEM_MIMETYPE = "application/x-item"

EDITOR_TO_DESCRIPTION = {"Editor External controllers": "Run an external controller",
                         "Editor External kinematics": "Run an external kinematics solver",
                         "Editor External Motion Planners": "Run an external motion planner",
                         "Editor High level methods": "Run an external high level component",
                         "Editor Sensors config": "Capture data from a configured sensor"}

COMMANDER_DATA_TYPE_CHOICE = ["", "joint state", "pose"]

ALL_MANAGER_TYPE_CHOICE = ["", "joint state", "plan", "pose", "trajectory"]

# Blue for input (index 0), Green for success related outcomes (index 1) and Red for failure related things (index 2)
DEDICATED_SOCKET_COLORS = [QColor("#FF4599FF"), QColor("#FF00cb00"), QColor("#FFFF0021")]

SOCKET_COLORS = [QColor("#FFFF8c00"), QColor("#FFa86db1"), QColor("#FFfc9a9a"),
                 QColor("#FF82eda2"), QColor("#FFa8e9ff"), QColor("#FFf6fa8e"), QColor("#FFFF0021")]

# Create a set of 10 colors uniformly distributed on the hue scale
TERMINAL_SOCKET_COLORS = [QColor().fromHsvF(i/10., 1., 1.) for i in range(10)]
# Make sure the two first colors correspond to green and red
TERMINAL_SOCKET_COLORS[1], TERMINAL_SOCKET_COLORS[3] = TERMINAL_SOCKET_COLORS[3], TERMINAL_SOCKET_COLORS[1]
TERMINAL_SOCKET_COLORS[0], TERMINAL_SOCKET_COLORS[1] = TERMINAL_SOCKET_COLORS[1], TERMINAL_SOCKET_COLORS[0]
