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

import os
import re
from collections import OrderedDict
from grip_core.utils.common_paths import COMMANDER_FOLDER

AVAILABLE_STATES = OrderedDict()
AVAILABLE_STATEMACHINES = OrderedDict()


def extract_init_from_file(file_path):
    """
        Extract and format the content of the __init__ function contained in a file

        @param file_path: Path to the file
        @return: List of strings
    """
    # Get a single string with the whole file inside
    with open(file_path, "r") as file_:
        file_content = "\n".join(file_.readlines())
    init_command = re.search("def __init__\((.*?)\):", file_content, re.DOTALL).group(1)
    init_command = re.sub("\n", "", init_command)
    init_command.replace(",", ", ")
    init_command = re.sub("\s{2}", "", init_command)
    init_command = re.split('\,\s*(?![^\[)]*\])', init_command)
    # Remove the self from the arguments
    return init_command[1:]


def extract_state_parameters_from_file(file_path):
    """
        Extract the parameters required to initialize a state

        @param file_path: Path to the file to parse
        @return: OrderedDict containing parameters name and potential default values
    """
    parameters = OrderedDict()
    split_parameters = extract_init_from_file(file_path)

    for parameter in split_parameters:
        if "=" not in parameter:
            parameters[parameter] = ""
        else:
            name, value = parameter.split("=")
            parameters[name] = value.strip("\"")
    return parameters


def extract_state_machine_parameters_from_file(file_path):
    """
        Extract the parameters required to initialize a state machine

        @param file_path: Path to the file to parse
        @return: OrderedDict containing parameters name and potential default values
    """
    parameters = OrderedDict()
    split_parameters = extract_init_from_file(file_path)
    # For each argument takes care of the jinja commands
    for parameter in split_parameters:
        parameter = re.sub("{%(.*?)%}", "", parameter)
        if "=" not in parameter:
            parameters[parameter] = ""
        else:
            name, value = parameter.split("=")
            if "{{" not in value and "%" not in value:
                parameters[name] = value
            else:
                value = re.sub("{{(.*?)}}", "", value)
                value = re.findall("[\'\"](.*?)[\'\"]", value)
                parameters[name] = value[0] if len(value) == 1 else value
    return parameters


def extract_description_from_file(file_path):
    """
        Extract the description (docstring) from the class contianed in the file

        @param file_path: Path to the file the description should be extracted
        @return: Description (string) if defined otherwise an empty string
    """
    with open(file_path, "r") as file_:
        file_content = "\n".join(file_.readlines())
    description = re.findall("class(?:[^\:]*)\:(?:[^\"]*)\"{3}(.*?)\"{3}", file_content, re.DOTALL)
    if not description:
        return "No description available"
    description = re.sub("\n", "", description[0])
    description = re.sub("\s{2}", "", description)
    return description


def fill_available_state_machines(path_folders):
    """
        Load all the state machine templates from the different paths contained in path_folders

        @param path_folders: List of paths pointing to directories containing templates to load
        @return: True if one of the path is not correct
    """
    not_a_path = False
    for path_folder in path_folders:
        if not os.path.isdir(path_folder):
            not_a_path = True
            continue
        for root, dirs, files in os.walk(path_folder):
            for file in files:
                if file.endswith(".template"):
                    file_path = os.path.join(root, file)
                    name = "".join([word.capitalize() for word in file.replace(".template", "").split("_")])
                    description = extract_description_from_file(file_path)
                    parameters = extract_state_machine_parameters_from_file(file_path)
                    if name not in AVAILABLE_STATEMACHINES.keys():
                        AVAILABLE_STATEMACHINES[name] = {"source": file_path, "parameters": parameters,
                                                         "description": description}
                    else:
                        print("Multiple file with the same name have been found. Ignoring the others.")
    if not_a_path:
        return True


def fill_available_states(path_folders):
    """
        Load all the states from the different paths contained in path_folders

        @param path_folders: List of paths pointing to directories containing the states to load
        @return: True if one of the path is not correct
    """
    not_a_path = False
    for path_folder in path_folders:
        if not os.path.isdir(path_folder):
            not_a_path = True
            continue

        for root, dirs, files in os.walk(path_folder):
            # When getting states associated to commanders, create a new dictionary
            if root == COMMANDER_FOLDER:
                dict_to_fill = OrderedDict()
            else:
                dict_to_fill = AVAILABLE_STATES

            for file in files:
                if file.endswith(".py") and file != "__init__.py":
                    file_path = os.path.join(root, file)
                    name = "".join([word.capitalize() for word in file.replace(".py", "").split("_")])
                    description = extract_description_from_file(file_path)
                    parameters = extract_state_parameters_from_file(file_path)
                    import_statement = get_import_statement(file_path)
                    # Add the state to the proper dictionary
                    if name not in dict_to_fill.keys():
                        dict_to_fill[name] = {"source": file_path, "parameters": parameters,
                                              "description": description, "import_statement": import_statement}
                    else:
                        print("The state named {} already exists. Ignoring the others.".format(name))
            # If the loaded states are related to commanders, then add the associated dictionary to AVAILABLE_STATES
            if root == COMMANDER_FOLDER:
                AVAILABLE_STATES["Commander"] = dict_to_fill

    if not_a_path:
        return True


def get_import_statement(file_path):
    """
        Return the correct import statement to use the python file located at file_path (/!\ Only valid for Python 2)

        @param file_path: Path to the python file (string)
        @return: String corresponding to what must be ater the from in an import statement (e.g. from xxx import yyy)
    """
    # Initialize the final import import_statement
    state_import_statement = None
    # Get the absolute path to avoid any possible mistake
    absolute_path = os.path.abspath(file_path)
    # Retrieve the correct python import statement by finding the correct package in which the states are stored
    # List gathering all the folders. Discarding the first element because it is empty (absolute path starts with /)
    split_path = os.path.dirname(absolute_path).split("/")[1:]
    # Variable that will contain the progressive paths
    tested_path = ""
    # For each root of the path tree, try to find if it contains an __init__.py (i.e the diretory is a package)
    for root_level, directory_name in enumerate(split_path):
        tested_path += "/{}".format(directory_name)
        # If we find the package name, we take everything after this to create the proper import path
        if os.path.exists(tested_path + "/__init__.py"):
            state_import_statement = ".".join(split_path[root_level:])
            break

    return state_import_statement


def is_def_file_valid(file_path):
    """
        Check whether an input file (.action or .srv) has the expected fields to be integrated to GRIP

        @param file_path: Path of the file (string)
        @return: True if the definition file (.action or .srv) is valid, False otherwise
    """
    # Get a single string with the whole file inside
    with open(file_path, "r") as file_:
        file_content = "\n".join(file_.readlines())
    # Get request/reply (srv file) or goal/result/feedback (action file)
    parts = re.split('---', file_content)
    # Double check that we have the proper number of parts
    if len(parts) < 2 or len(parts) > 3:
        return False
    # We are just interested in the two first elements anyway
    # We must have the input field in the first element (space is very important)
    is_first_part_ok = " input" in parts[0]
    # In the second part we must have both int8 outcome and returned_object
    is_second_part_ok = "int8 outcome" in parts[1] and " returned_object" in parts[1]
    # Return whether the file is correct or not
    return is_first_part_ok and is_second_part_ok
