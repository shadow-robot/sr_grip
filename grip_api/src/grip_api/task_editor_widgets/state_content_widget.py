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

import inflection
import os
from PyQt5.QtWidgets import QWidget, QGridLayout
from grip_core.utils.file_parsers import AVAILABLE_STATES, get_import_statement
from grip_core.utils.common_paths import EXTERNAL_COMPONENT_TEMPLATE, GENERATED_STATES_FOLDER, SENSOR_TEMPLATE
from .state_config_widgets import StateConfigBox, CommanderStateConfigBox, GeneratedStateConfigBox, StateMachineConfigBox


class StateContentWidget(QWidget):

    """
        Widget that contains the area in which the user can configure a state
    """

    def __init__(self, state, parent=None):
        """
            Initialize the widget

            @param state: Object (State) to which this widget is added
            @param parent: Parent (QWidget) of the state
        """
        # Get the state
        self.state = state
        super().__init__(parent)
        # State displayer in which the state is displayed
        state_displayer = self.state.container.editor_widget.parent().parent().parent().parent().state_displayer
        # Get the name of the states to be generated (i.e. corresponding to integrated components)
        self.states_to_generate = list()
        if "Generated" in state_displayer.list_widget.states_to_display:
            self.states_to_generate = state_displayer.list_widget.states_to_display["Generated"]
        # Initialize the UI
        self.init_ui()

    def init_ui(self):
        """
            Set the layout of the widget
        """
        # Create a scalable widget
        self.layout = QGridLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(self.layout)
        self.initialise_config_interface()
        # Add the configuration area in the layout
        self.layout.addWidget(self.config_state)
        # Make sure the widget's dimensions are updated
        self.adjustSize()

    def initialise_config_interface(self):
        """
            Initialise the proper widget that allows the user to configure the state from the task editor
        """
        # Extract information from the state and set the proper configuration interface for the state
        if self.state.type in AVAILABLE_STATES:
            self.state_info = AVAILABLE_STATES[self.state.type]
            config_box = StateConfigBox if self.state.type != "Select" else GeneratedStateConfigBox
        elif self.state.type in AVAILABLE_STATES["Commander"]:
            self.state_info = AVAILABLE_STATES["Commander"][self.state.type]
            config_box = CommanderStateConfigBox
        elif self.state.type in self.states_to_generate:
            self.initialize_state_info()
            config_box = GeneratedStateConfigBox
        # Create the configuration area
        self.config_state = config_box(self.state.type, self.state_info["parameters"], parent=self)
        # Make sure the widget has the proper dimensions
        self.config_state.adjustSize()

    def reset_ui(self):
        """
            Reinitialise the widget allowings the user to configure the state from the task editor
        """
        # Update the config_state attribute
        self.initialise_config_interface()
        # Add the configuration in the layout
        self.layout.addWidget(self.config_state)
        # Remove the previous one
        self.layout.itemAt(0).widget().setParent(None)
        # Correct size this widget should have (inherited from child)
        correct_size = self.config_state.size()
        # Make sure the widget does not get too much space
        self.setMaximumSize(correct_size)
        # Adjust the size of this widget
        self.adjustSize()
        # Make sure this widget takes all the space it needs
        self.resize(correct_size)
        # Update the dimensions of the graphical state
        self.state.graphics_state.update_dimensions()

    def initialize_state_info(self):
        """
            Initialize the state_info attribute for a generated state
        """
        # Set the to_generate attribute of the state to True
        self.state.to_generate = True
        # Get the state info (there is too much info in there, so filter it out)
        state_info = self.states_to_generate[self.state.type]["info"]
        # Create the attribute
        self.state_info = dict()
        # If the state corresponds to an external component
        if "action/service" in state_info:
            self.state_info["template"] = EXTERNAL_COMPONENT_TEMPLATE
            outcomes = [str(i) for i in range(state_info["number_outcomes"])] + ["state_failure"]
            self.state_info["parameters"] = dict([("input", None), ("input_type", None), ("output_type", None),
                                                  ("output", None), ("outcomes", outcomes)])
            self.state_info["action/service"] = state_info["action/service"]
            self.state_info["server_name"] = state_info["server_name"]
        elif "data_topics" in state_info:
            self.state_info["template"] = SENSOR_TEMPLATE
            outcomes = ["success", "failure"]
            self.state_info["parameters"] = dict([("sensor_topic", None), ("output", None),
                                                  ("outcomes", outcomes)])
            self.state_info["data_topics"] = state_info["data_topics"]
        # Get common values regardless of what kind of state is generated
        self.state_info["filename"] = inflection.underscore(self.state.type)
        self.state_info["name"] = inflection.camelize(self.state.type)
        self.state_info["source"] = self.state_info["filename"] + ".py"
        final_path = os.path.join(GENERATED_STATES_FOLDER, self.state_info["source"])
        self.state_info["import_statement"] = get_import_statement(final_path)

    def get_outcomes(self):
        """
            Get the default outcomes set in the state

            @return: List of the outcomes (list of string)
        """
        outcomes = self.state_info["parameters"]["outcomes"]
        if isinstance(outcomes, list):
            return outcomes
        elif isinstance(outcomes, str):
            list_outcomes = outcomes.replace("[", "").replace("]", "").replace("\"", "").replace(" ", "").split(",")
            return list_outcomes
        else:
            return None

    def get_config(self, to_save=False):
        """
            Retrieve the current configuration of the content of the state

            @param to_save: Boolean specifying whether the returned config is going to be saved or if it used to
                            generate the state machine. Default is False (i.e. for generating the state machine)

            @return: Dictionary mapping the name of the parameters of the state to their configured values
        """
        state_config = dict()
        # Extract the different information from the state's content
        for parameter_name, default_config in self.state_info["parameters"].items():
            # Make sure to only extract displayed parameters
            if parameter_name not in self.config_state.slots_to_discard:
                # Specific case when we want to save the configuration for the sensor states
                if isinstance(self.config_state, GeneratedStateConfigBox) and to_save:
                    user_config = self.config_state.get_slot_config(parameter_name, False)
                else:
                    user_config = self.config_state.get_slot_config(parameter_name)
                # If the configuration slot was left empty, get the default configuration if it's not None
                if user_config == "" and default_config is not None:
                    user_config = self.config_state.to_format(default_config)
                # Store the user config in the dictionary
                state_config[parameter_name] = user_config
        return state_config

    def set_config(self, config):
        """
            Set the parameters of the state according to a previously saved configuration

            @param config: Dictionary mapping the name of the parameters of the state to their saved values
        """
        for param_name, param_value in config.items():
            self.config_state.set_slot_value(param_name, param_value)


class StateMachineContentWidget(QWidget):

    """
        Widget that contains the area in which the user can configure the userdata of a state machine
    """

    def __init__(self, state_machine, parent=None):
        """
            Initialize the widget

            @param state: Object (StateMachine) to which this widget is added
            @param parent: Parent (QWidget) of the state
        """
        # Store the state machine
        self.state_machine = state_machine
        super().__init__(parent)
        # Initialize the UI
        self.init_ui()

    def init_ui(self):
        """
            Set the layout of the widget
        """
        # Create a scalable widget
        self.layout = QGridLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(self.layout)
        # Create the configuration area
        self.config_state = StateMachineConfigBox(self.state_machine.def_container.type,
                                                  self.state_machine.def_container.parameters, parent=self)
        self.layout.addWidget(self.config_state)
        # Make sure the layout is adjusted to the size of the widget it contains
        self.adjustSize()
