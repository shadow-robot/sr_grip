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

import inflection
import os
from collections import OrderedDict
from graphical_editor_base import Serializable
from PyQt5.QtWidgets import QWidget, QGridLayout
from grip_core.utils.file_parsers import AVAILABLE_STATES, get_import_statement
from grip_core.utils.common_paths import EXTERNAL_COMPONENT_TEMPLATE, GENERATED_STATES_FOLDER
from state_config_widgets import StateConfigBox, CommanderStateConfigBox, GeneratedStateConfigBox, StateMachineConfigBox


class StateContentWidget(QWidget, Serializable):

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
        super(StateContentWidget, self).__init__(parent)
        # State displayer in which the state is displayed
        state_displayer = self.state.container.editor_widget.parent().parent().parent().parent().state_displayer
        # Get the name of the states to be generated (i.e. corresponding to integrated components)
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
        # Extract information from the state and set the proper configuration interface in the state
        if self.state.type in AVAILABLE_STATES:
            self.state_info = AVAILABLE_STATES[self.state.type]
            config_box = StateConfigBox
        elif self.state.type in AVAILABLE_STATES["Commander"]:
            self.state_info = AVAILABLE_STATES["Commander"][self.state.type]
            config_box = CommanderStateConfigBox
        elif self.state.type in self.states_to_generate:
            self.initialize_state_info()
            config_box = GeneratedStateConfigBox
        # Create the configuration area
        self.config_state = config_box(self.state.type, self.state_info["parameters"], parent=self)
        # Add the configuration area in the layout
        self.layout.addWidget(self.config_state)
        # Make sure the widget's dimensions are updated
        self.adjustSize()

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
            self.state_info["parameters"] = OrderedDict([("input", None), ("input_type", None), ("output_type", None),
                                                         ("output", None), ("outcomes", outcomes)])
            self.state_info["action/service"] = state_info["action/service"]
            self.state_info["server_name"] = state_info["server_name"]
        else:
            pass
        # Get common values regardless of which kinf of state is generated
        self.state_info["filename"] = inflection.underscore(self.state.type)
        self.state_info["name"] = self.state.type
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


class StateMachineContentWidget(QWidget, Serializable):

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
        super(StateMachineContentWidget, self).__init__(parent)
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
