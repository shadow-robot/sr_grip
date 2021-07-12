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

import re
from PyQt5.QtWidgets import QGridLayout, QLineEdit, QGroupBox, QLabel, QComboBox
from grip_api.utils.files_specifics import COMMANDER_DATA_TYPE_CHOICE, ALL_MANAGER_TYPE_CHOICE
from grip_api.utils.common_dialog_boxes import warning_message


class GenericConfigBoxWidget(QGroupBox):

    """
        QGroupBox that contains configuration slots for a state
    """

    def __init__(self, type, parent=None):
        """
            Initialize the widget for a given state type

            @param type: Type of the state (string)
            @param parent: Parent of the widget (QWidget)
        """
        super(GenericConfigBoxWidget, self).__init__(type, parent=parent)
        self.init_ui()
        # Number of configuration slots
        self.number_rows = 0
        # Store the slots
        self.registered_keys = []
        # Configuration slots we don't want to display
        self.slots_to_discard = ("outcomes", "input_keys", "output_keys", "io_keys")

    def init_ui(self):
        """
            Create the layout of the widget
        """
        self.layout = QGridLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(self.layout)

    def add_configuration_slot(self, slot_name, placeholder_text=None):
        """
            Add a line to the layout containing both a label and a QLineEdit

            @param slot_name: Name of the parameter to configure
            @param placeholder_text: Text that can be used as a placeholder to indicate examples or default values
        """
        # Create the label
        slot_title = QLabel(slot_name + ":", objectName="slot {}".format(slot_name))
        # Configure the QLine Edit
        line = QLineEdit(objectName="line {}".format(slot_name))
        line.setClearButtonEnabled(True)
        # If provided, set the placeholder text
        if placeholder_text is not None:
            line.setPlaceholderText(placeholder_text)
        # Add both widgets in the layout
        self.layout.addWidget(slot_title, self.number_rows, 0)
        self.layout.addWidget(line, self.number_rows, 1)
        # Update the class' attributes
        self.number_rows += 1
        self.registered_keys.append(slot_name)

    def add_choice_slot(self, slot_name, choices):
        """
            Add a line to the layout containing both a label and a QComboBox (i.e. dropdown list)

            @param slot_name: Name of the parameter to configure
            @param choices: List of strings the user can select from
        """
        # Create the label
        slot_title = QLabel(slot_name + ":", objectName="slot {}".format(slot_name))
        # Configure the QComboBox
        list_choice = QComboBox(objectName="choice {}".format(slot_name))
        # Set the different choices
        list_choice.addItems(choices)
        # Add both widgets in the layout
        self.layout.addWidget(slot_title, self.number_rows, 0)
        self.layout.addWidget(list_choice, self.number_rows, 1)
        # Update the class' attributes
        self.number_rows += 1
        self.registered_keys.append(slot_name)

    def get_slot_config(self, slot_name):
        """
            Get the current config of a given slot

            @param slot_name: String specifying from which slot we want to extract the config from
            @return: String, list, int or float corresponding to the current config
        """
        # If the slot name corresponds to a QCombo box then return its text
        combo_widget = self.findChild(QComboBox, "choice {}".format(slot_name))
        if combo_widget:
            raw_text = combo_widget.currentText()
        else:
            # Otherwise it means it comes from a QLineEdit, and get the text from it
            raw_text = self.findChild(QLineEdit, "line {}".format(slot_name)).text()
        # If the slot is empty
        if not raw_text:
            return raw_text

        # If the text does not correspond to a list or tuple
        if "[" not in raw_text and "]" not in raw_text:
            return self.to_format(raw_text)

        # If the config is a list or tuple, remove the corresponding brackets.
        raw_text = re.search("[^\[\(].*[^\]\)]", raw_text).group(0)
        # Split the text according to "," and store what it finds in a list
        split_text = re.split("\,\s*", raw_text)
        # Generate the list with all the elements parsed
        final_list = list()
        for text in split_text:
            final_list.append(self.to_format(text))
        return final_list

    def set_slot_value(self, slot_name, value):
        """
            Set the current config of a given slot

            @param slot_name: String specifying for which slot we want to set the content
            @param value: String to be displayed in the given slot
        """
        # If the value to set is a list, transform it back to a string so it can be displayed
        if isinstance(value, list):
            value = map(lambda x: str(x), value)
            value = "[" + ", ".join(value) + "]"
        # Transform booleans to strings
        elif isinstance(value, bool):
            value = str(value)
        # Check if the slot corresponds to a QComboBox
        combo_widget = self.findChild(QComboBox, "choice {}".format(slot_name))
        # If that's the case
        if combo_widget:
            # And the widget is editable, set the current text
            if combo_widget.isEditable():
                combo_widget.setCurrentText(value)
            # Otherwise, it means that we can only select pre-determined options.
            else:
                # Get all registered choices for the specific QComboBox
                all_items = [combo_widget.itemText(i) for i in range(combo_widget.count())]
                try:
                    index_choice = all_items.index(value)
                    combo_widget.setCurrentIndex(index_choice)
                except ValueError:
                    warning_message("Error configuring a state", "For state of type {}:".format(self.title()),
                                    additional_text="The provided value {} for slot {} is not valid.".format(value,
                                                                                                             slot_name))
        # Otherwise it should be a QLineEdit
        else:
            widget = self.findChild(QLineEdit, "line {}".format(slot_name))
            # If it is indeed a QLineEdit then we just need to set the provided input
            if widget:
                widget.setText(value)
            elif not (isinstance(self, CommanderStateConfigBox) and slot_name == "group_name" and
                      len(self.commander_choice) == 2):
                warning_message("Error configuring a state", "For state of type {}:".format(self.title()),
                                additional_text="The parameter {} cannot be found".format(slot_name))

    @staticmethod
    def to_format(input):
        """
            Turn the string input to the intended format (string, int, float or boolean)

            @param input: String to convert
            @return: Either a string, an int, a float or a boolean
        """
        try:
            return int(input)
        except (TypeError, ValueError):
            pass
        try:
            return float(input)
        except (TypeError, ValueError):
            pass
        if input in ("true", "True"):
            return True
        elif input in ("false", "False"):
            return False
        else:
            return str(input)


class StateConfigBox(GenericConfigBoxWidget):

    """
        Class derived from GenericConfigBoxWidget that creates the configuration area for a State
    """

    def __init__(self, source, state_parameters, parent=None):
        """
            Intialize the widget

            @param source: String corresponding to the type of the state
            @param state_parameters: Dictionary containing the parameters of the given state type
            @param parent: Parent (QWidget) of this widget
        """
        super(StateConfigBox, self).__init__("type: {}".format(source), parent=parent)
        # Initialize the content
        self.initialize_content(state_parameters)

    def initialize_content(self, state_parameters):
        """
            Add configuration slots for every parameters parsed from the state source

            @param state_parameters: Dictionary containing all the parameters to display
        """
        # For each parameter coming from the state
        for key, value in state_parameters.items():
            # Make sure to ignore some slots as we already take care of them internally
            if key in self.slots_to_discard:
                continue
            else:
                # Get any default value
                default_value = re.findall("\"(.*?)\"", value)
                if len(default_value) > 1:
                    default_value = ", ".join(default_value)
                else:
                    default_value = value
                # Add the configuration slot
                self.add_configuration_slot(key, placeholder_text=default_value)


class CommanderStateConfigBox(GenericConfigBoxWidget):

    """
        Class derived from GenericConfigBoxWidget that creates the configuration area for a commander state
    """

    def __init__(self, source, state_parameters, parent=None):
        """
            Intialize the widget

            @param source: String corresponding to the type of the state
            @param state_parameters: Dictionary containing the parameters of the given state type
            @param parent: Parent (QWidget) of this widget
        """
        super(CommanderStateConfigBox, self).__init__("type: {}".format(source), parent=parent)
        # Get a pointer to the task editor area
        task_editor_area = self.parent().state.container.editor_widget.parent().parent().parent().parent()
        # To access the robot integration area
        robot_integration_area = task_editor_area.framework_gui.robot_integration_area
        # Get access to the settings configuration
        settings_config = robot_integration_area.settings_config_widget
        # Initialize the choice for commander
        self.commander_choice = [""] + sorted(robot_integration_area.commander_config.keys())
        # Initialize the list of registered poses and joint states
        self.known_states = {"pose": settings_config.named_poses.poses.keys(),
                             "joint state": settings_config.named_joint_states.valid_input.keys()}
        # Connect the signal coming from the robot integration area re. the commander configs
        robot_integration_area.commanderUpdated.connect(self.update_commander_choice)
        # Connect the signals coming from the poses and joint states editors
        settings_config.named_poses.canBeSaved.connect(self.update_known_poses)
        settings_config.named_joint_states.canBeSaved.connect(self.update_known_joint_states)
        # Initialize the content
        self.initialize_content(state_parameters)

    def update_known_poses(self):
        """
            Update available poses defined in the corresponding editor
        """
        self.known_states["pose"] = self.sender().poses.keys()

    def update_known_joint_states(self):
        """
            Update the available joint states defined in the corresponding editor
        """
        known_js = list() if not self.sender().valid_input else self.sender().valid_input.keys()
        self.known_states["joint state"] = known_js

    def update_commander_choice(self):
        """
            Update the list of available commanders
        """
        # Update the proper attribute
        self.commander_choice = [""]
        if self.sender().commander_config is not None:
            self.commander_choice += sorted(self.sender().commander_config)
        # Get the corresponding drop down list
        combo_box = self.findChild(QComboBox, "choice {}".format("group_name"))
        # Update its content
        if combo_box is not None:
            combo_box.clear()
            combo_box.addItems(self.commander_choice)

    def add_choice_slot(self, slot_name, choices, is_editable=False):
        """
            Add a line to the layout containing both a label and a QComboBox

            @param slot_name: Name of the parameter to configure
            @param choices: List of strings the user can select
            @param is_editable: Boolean stating whether the user can edit its content or not. Default is False
        """
        # Create the label
        slot_title = QLabel(slot_name + ":", objectName="slot {}".format(slot_name))
        # Configure the QComboBox
        list_choice = QComboBox(objectName="choice {}".format(slot_name))
        # Set the different choices
        list_choice.addItems(choices)
        list_choice.setEditable(is_editable)
        if "type" in slot_name:
            list_choice.currentTextChanged.connect(self.update_choice_content)
        # Add both widgets in the layout
        self.layout.addWidget(slot_title, self.number_rows, 0)
        self.layout.addWidget(list_choice, self.number_rows, 1)
        # Update the class' attributes
        self.number_rows += 1
        self.registered_keys.append(slot_name)

    def update_choice_content(self, current_text):
        """
            Update the choices given in the combo box associated to the sender

            @param current_text: New value of the text from the QCombobox emitting the signal
        """
        # Get the other (associated) QComboBox
        widget = self.findChild(QComboBox, "{}".format(self.sender().objectName().replace("type", "name")))
        widget.clear()
        # Depending on the current text of the sender, update the possible choices
        if not current_text:
            widget.addItem("")
        else:
            widget.addItems([""] + self.known_states[current_text])

    def initialize_content(self, state_parameters):
        """
            Add configuration slots for every parameters parsed from the state source

            @param state_parameters: Dictionary containing all the parameters to display
        """
        # For each parameter coming from the state
        for key, value in state_parameters.items():
            # Make sure to ignore some slots as we already take care of them internally, and remove the group_name slot
            # if we only have one planner configured to make the configuration simpler for the user
            if key in self.slots_to_discard or key == "group_name" and len(self.commander_choice) == 2:
                continue
            # If the slot contains "type" add choices between pose and joint states
            elif "type" in key:
                self.add_choice_slot(key, COMMANDER_DATA_TYPE_CHOICE)
            # If we have at least two planners configured, provide a choice
            elif key == "group_name":
                choices = self.commander_choice[:]
                editable = False
                if "Collisions" in self.title():
                    editable = True
                # For the AllowCollisions state, make the field editable, so the user can add other groups, even if not
                # configured in GRIP
                self.add_choice_slot(key, choices, editable)
            # Initialise a choice slot providing the joint states and poses already configured in GRIP
            elif key in ("target_name", "starting_name"):
                self.add_choice_slot(key, [""], True)
            # Only two kind of collisions can be changed
            elif key == "collision":
                self.add_choice_slot(key, ["self collision", "object collision"])
            # Can only allow (True) or disallow (False) a specific collision
            elif key == "allow":
                self.add_choice_slot(key, ["True", "False"])
            else:
                # Get any default value
                default_value = re.findall("\"(.*?)\"", value)
                if len(default_value) > 1:
                    default_value = ", ".join(default_value)
                elif key != "plan_name":
                    default_value = value
                else:
                    default_value = None
                # Add the configuration slot
                self.add_configuration_slot(key, placeholder_text=default_value)

    def get_slot_config(self, slot_name):
        """
            Get the current config of a given slot

            @param slot_name: String specifying from which slot we want to extract the config from
            @return: String, list, int or float corresponding to the current config
        """
        # If we have only one commander then returns it
        if slot_name == "group_name" and len(self.commander_choice) == 2:
            return self.commander_choice[-1]

        return super(CommanderStateConfigBox, self).get_slot_config(slot_name)


class GeneratedStateConfigBox(GenericConfigBoxWidget):

    """
        Class derived from GenericConfigBoxWidget that creates the configuration area for a state running external code
        or a sensor
    """

    def __init__(self, source, state_parameters, parent=None):
        """
            Intialize the widget

            @param source: String corresponding to the type of the state
            @param state_parameters: Dictionary containing the parameters of the given state type
            @param parent: Parent (QWidget) of this widget
        """
        super(GeneratedStateConfigBox, self).__init__("type: {}".format(source), parent=parent)
        # If the state correponds to a sensor, then create an attribute to the class that must be linked to the sensor
        if "sensor_topic" in state_parameters:
            self.topic_mapping = self.parent().state_info["data_topics"]
        else:
            # Get a pointer to the task editor area
            task_editor_area = self.parent().state.container.editor_widget.parent().parent().parent().parent()
            # To get access to the robot integration area
            robot_integration_area = task_editor_area.framework_gui.robot_integration_area
            # Get access to the settings configuration
            settings_config = robot_integration_area.settings_config_widget
            # Initialize the list of registered msgs in GRIP's editors
            self.known_msgs = {"pose": settings_config.named_poses.poses.keys(),
                               "joint state": settings_config.named_joint_states.valid_input.keys(),
                               "trajectory": settings_config.named_trajectories.valid_input.keys(),
                               "plan": list()}
            # Connect the signals coming from the different editors
            settings_config.named_poses.canBeSaved.connect(self.update_known_poses)
            settings_config.named_joint_states.canBeSaved.connect(self.update_known_joint_states)
            settings_config.named_trajectories.canBeSaved.connect(self.update_known_trajectories)
        # Initialize the content
        self.initialize_content(state_parameters)

    def initialize_content(self, state_parameters):
        """
            Add configuration slots for all the parameters parsed from the state source

            @param state_parameters: Dictionary containing all the parameters to display
        """
        # For each parameter coming from the state
        for key, value in state_parameters.items():
            # Make sure to ignore some slots as we already take care of them internally
            if key in self.slots_to_discard:
                continue
            # For two particular slots provide choices corresponding to all the managers types
            elif key == "output_type" or key == "input_type":
                self.add_choice_slot(key, ALL_MANAGER_TYPE_CHOICE)
            elif key == "sensor_topic":
                self.add_choice_slot(key, self.topic_mapping.keys())
            elif key == "input":
                self.add_choice_slot(key, [""], True)
            else:
                # Add the configuration slot
                self.add_configuration_slot(key, None)

    def add_choice_slot(self, slot_name, choices, is_editable=False):
        """
            Add a line to the layout containing both a label and a QComboBox

            @param slot_name: Name of the parameter to configure
            @param choices: List of strings the user can select
            @param is_editable: Boolean stating whether the user can edit its content or not. Default is False
        """
        # Create the label
        slot_title = QLabel(slot_name + ":", objectName="slot {}".format(slot_name))
        # Configure the QComboBox
        list_choice = QComboBox(objectName="choice {}".format(slot_name))
        # Set the different choices
        list_choice.addItems(choices)
        list_choice.setEditable(is_editable)
        if slot_name == "input_type":
            list_choice.currentTextChanged.connect(self.update_choice_content)
        # Add both widgets in the layout
        self.layout.addWidget(slot_title, self.number_rows, 0)
        self.layout.addWidget(list_choice, self.number_rows, 1)
        # Update the class' attributes
        self.number_rows += 1
        self.registered_keys.append(slot_name)

    def update_choice_content(self, current_text):
        """
            Update the choices given in the combo box associated to the sender

            @param current_text: New value of the text from the QCombobox emitting the signal
        """
        # Get the other (associated) QComboBox
        widget = self.findChild(QComboBox, "{}".format(self.sender().objectName().replace("_type", "")))
        # When restoring, allows us to get the text previously set
        widget_text = widget.currentText()
        # Allows for removing useless suggestions
        widget.clear()
        # Depending on the current text of the sender, update the possible choices
        if not current_text:
            widget.addItem("")
        else:
            widget.addItems([""] + self.known_msgs[current_text])
            # Restore the previously set text
            widget.setCurrentText(widget_text)

    def update_known_poses(self):
        """
            Update available poses defined in the corresponding editor
        """
        self.known_msgs["pose"] = self.sender().poses.keys()

    def update_known_joint_states(self):
        """
            Update the available joint states defined in the corresponding editor
        """
        known_js = list() if not self.sender().valid_input else self.sender().valid_input.keys()
        self.known_msgs["joint state"] = known_js

    def update_known_trajectories(self):
        """
            Update the available trajectories defined in the corresponding editor
        """
        known_traj = list() if not self.sender().valid_input else self.sender().valid_input.keys()
        self.known_msgs["trajectory"] = known_traj

    def get_slot_config(self, slot_name, return_mapped=True):
        """
            Get the current config of a given slot

            @param slot_name: String specifying from which slot we want to extract the config from
            @param return_mapped: Boolean stating whether the returned parameter should be the renamed topic or not.
                                  This parameter matters only for states generated for sensors
            @return: String, list, int or float corresponding to the current config
        """
        slot_config = super(GeneratedStateConfigBox, self).get_slot_config(slot_name)

        # For the sensor_topic slot we must send the real topic name if requested, otherwise send the remapped one
        if slot_name == "sensor_topic" and return_mapped:
            return self.topic_mapping[slot_config]
        else:
            return slot_config


class StateMachineConfigBox(GenericConfigBoxWidget):

    """
        Class derived from GenericConfigBoxWidget that creates the configuration area for a StateMachine
    """

    def __init__(self, source, state_machine_parameters, parent=None):
        """
            Intialize the widget

            @param source: String corresponding to the type of the state machine
            @param state_machine_parameters: Dictionary containing the parameters of the given state machine type
            @param parent: Parent (QWidget) of this widget
        """
        super(StateMachineConfigBox, self).__init__("type: {}".format(source), parent=parent)
        # Initialize the content
        self.initialize_content(state_machine_parameters)

    def initialize_content(self, state_machine_parameters):
        """
            Add configuration slots for the userdata of the state machine

            @param state_machine_parameters: Dictionary containing all the parameters to display
        """
        value = state_machine_parameters["userdata"]
        # Get any default value
        default_value = re.findall("\"(.*?)\"", value)
        if len(default_value) > 1:
            default_value = ", ".join(default_value)
        else:
            default_value = value
        # Add the configuration slot
        self.add_configuration_slot("userdata", placeholder_text=default_value)
