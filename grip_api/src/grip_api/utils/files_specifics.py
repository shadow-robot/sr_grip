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

from collections import OrderedDict

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

ARM_CONFIG = {"Editor Arm hardware connection": OrderedDict(), "Editor ROS controllers": OrderedDict(),
              "Editor MoveIt! planners": OrderedDict(), "Editor External kinematics": OrderedDict(),
              "Editor External controllers": OrderedDict(), "Editor External Motion Planners": OrderedDict()}

HAND_CONFIG = {"Editor Hand hardware connection": OrderedDict(), "Editor ROS controllers": OrderedDict(),
               "Editor MoveIt! planners": OrderedDict(), "Editor External kinematics": OrderedDict(),
               "Editor External controllers": OrderedDict(), "Editor External Motion Planners": OrderedDict()}

SETTINGS_CONFIG = {"Editor Named joint states": OrderedDict(), "Editor Named poses": OrderedDict(),
                   "Editor Named trajectories": OrderedDict(), "Editor Sensors config": OrderedDict(),
                   "Editor Sensor plugins": OrderedDict(), "Editor High level methods": OrderedDict()}

LISTITEM_MIMETYPE = "application/x-item"

EDITOR_TO_DESCRIPTION = {"Editor External controllers": "Run an external controller",
                         "Editor External kinematics": "Run an external kinematics solver",
                         "Editor External Motion Planners": "Run an external motion planner",
                         "Editor High level methods": "Run an external high level component",
                         "Editor Sensors config": "Capture data from a configured sensor"}

DATA_TYPE_CHOICE = ["", "joint state", "pose"]
