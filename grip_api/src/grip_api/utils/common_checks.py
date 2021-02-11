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
import rospkg
import ruamel.yaml
from ruamel.yaml.comments import CommentedMap
from ruamel.yaml.scalarstring import DoubleQuotedScalarString


def is_pose_valid(dictionary, add_frame_id=False, add_reference_frame=True):
    """
        Method allowing to check that a dictionary is properly formatted i.e contains the proper keys and
        have appropriate values type to represent a pose

        @param dictionary: Dictionary containing the pose to check
        @param add_frame_id: Boolean stating whether the pose should contain the field "frame_id" or not
        @param add_reference_frame: Boolean stating whether the pose should contain the field "reference_frame" or not
        @return: True of the dictionary is properly formatted, False otherwise
    """
    keys_to_check = ["position", "orientation"]
    if add_frame_id:
        keys_to_check.append("frame_id")
    if add_reference_frame:
        keys_to_check.append("reference_frame")

    if not set(keys_to_check) == set(dictionary):
        return False
    # If the position does not have the proper keys
    if set(dictionary["position"].keys()) != set(["x", "y", "z"]):
        return False
    # If the orientation doesn't have appropriate keys
    is_quaternion_orientation = set(dictionary["orientation"].keys()) == set(["x", "y", "z", "w"])
    is_rpy_orientation = set(dictionary["orientation"].keys()) == set(["r", "p", "y"])
    if not (is_quaternion_orientation or is_rpy_orientation):
        return False
    # Check that values of both orientation and position are either int or float
    valid_position_value = all(isinstance(x, float) or isinstance(x, int) for x in dictionary["position"].values())
    valid_orient_value = all(isinstance(x, float) or isinstance(x, int) for x in dictionary["orientation"].values())
    # Check values provided for the frames are strings
    valid_reference_frame = True if not add_reference_frame else isinstance(dictionary["reference_frame"], str)
    valid_frame_id = True if not add_frame_id else isinstance(dictionary["frame_id"], str)
    # If all are good return True
    if valid_orient_value and valid_position_value and valid_frame_id and valid_reference_frame:
        return True

    return False


def is_topic_valid(topic_info):
    """
        Check that the information provided as topic info is valid

        @param topic_info: Value provided by the user
        @return: True if the value is properly formatted, False otherwise
    """
    if not isinstance(topic_info, OrderedDict):
        return False

    if any(not isinstance(topic_name, str) for topic_type, topic_name in topic_info.items()):
        return False
    return True


def is_moveit_planner_valid(moveit_planner):
    """
        Check that the information provided to use a moveit planner is correct

        @param moveit_planner: Dictionary provided by the user
        @return: True if the dictionary is properly formatted, False otherwise
    """
    planner_name_value = moveit_planner["planner_name"]
    robot_speed_value = moveit_planner["robot_speed_factor"]
    number_plan_value = moveit_planner["number_plan_attempt"]
    planning_time_value = moveit_planner["planning_max_time"]
    if not isinstance(planner_name_value, str):
        return False
    if (not (isinstance(robot_speed_value, float) or isinstance(robot_speed_value, int)) or
       (robot_speed_value <= 0 or robot_speed_value > 1)):
        return False
    if not isinstance(number_plan_value, int) or number_plan_value <= 0:
        return False
    if not (isinstance(planning_time_value, float) or isinstance(planning_time_value, int)) or planning_time_value <= 0:
        return False

    return all(x for x in moveit_planner.values())


def create_yaml_file(dictionary, file_path):
    """
        Given a dictionary (or OrderedDict), write a YAML file

        @param dictionary: Dictionary (or OrderedDict) containing inforamtion to be written in a YAML file
        @param file_path: Path specifying where to store the file
    """
    yaml_conf = ruamel.yaml.YAML(typ='rt')
    yaml_conf.indent(mapping=2, sequence=4, offset=2)
    yaml_conf.preserve_quotes = True
    with open(file_path, "w") as file_:
        yaml_conf.dump(to_ruamel_format(dictionary), file_)


def to_ruamel_format(element):
    """
        Make a given element to the proper format so it can be properly written in a YAML file

        @param element: String, integer or dictionary that needs to be stored in a YAML file
        @return: Properly formatted element
    """
    # If it's a dictionary or OrderedDict
    if isinstance(element, dict):
        # This is the equivalent of an OrderedDict
        commented_map = CommentedMap()
        # Call recursively this function on values of the dictionary
        for key, value in element.items():
            commented_map[key] = to_ruamel_format(value)
        return commented_map
    # If it's an empty string
    elif element == "\"\"" or element == "\'\'":
        return DoubleQuotedScalarString('')
    # Otherwise just return the element as it is
    return element


def is_launchfile_valid(file_path):
    """
        Check whether a provided path to a launch file is valid

        @param file_path: Path to a given file (that should correspond to a launch file)
        @return: True if the launch file is valid, False otherwise
    """
    # If the input is not a string return False
    if not isinstance(file_path, str):
        return False
    # If the path does not end with .launch return False
    if not file_path.endswith(".launch"):
        return False
    # If it is not part of a ros package return False as well
    ros_pkg = rospkg.get_package_name(file_path)
    if not ros_pkg:
        return False
    # Otherwise retrun True
    return True
