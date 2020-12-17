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

import re
from graphical_editor_base import Serializable
from PyQt5.QtWidgets import QWidget, QGridLayout, QScrollArea, QLineEdit, QGroupBox, QLabel
from grip_core.utils.file_parsers import AVAILABLE_STATES
from PyQt5.QtCore import QEvent


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
        # Initialize the UI
        self.init_ui()

    def init_ui(self):
        """
            Set the layout and scrolling area of the widget
        """
        # Create a scalable widget
        self.layout = QGridLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(self.layout)
        # Extract information from the state
        self.state_info = AVAILABLE_STATES[self.state.type]
        # Create the configuration area
        self.config_state = StateConfigBox(self.state.type, self.state_info["parameters"], parent=self)
        # Wrap it into a scrolling area to limit the state size
        self.scroll_area = StateScrollingArea(self.config_state)
        # Add the scrolling area in here
        self.layout.addWidget(self.scroll_area)

    def get_outcomes(self):
        """
            Get the default outcomes set in the state

            @return: List of the outcomes (list of string)
        """
        raw_text = self.state_info["parameters"]["outcomes"]
        list_outcomes = raw_text.replace("[", "").replace("]", "").replace("\"", "").replace(" ", "").split(",")
        return list_outcomes


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

    def get_slot_config(self, slot_name):
        """
            Get the current config of a given slot

            @param slot_name: String specifying from which slot we want to extract the config from
            @return: String, list, int or float corresponding to the current config
        """
        raw_text = self.findChild(QLineEdit, "line {}".format(slot_name)).text()

        if not raw_text:
            return raw_text

        # If the text does not correspond to a list or tuple
        if "[" not in raw_text and "[" not in raw_text:
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
            Add configuration slots for every parameters parsed from the state source (except outcomes)

            @param state_parameters: Dictionary containing all the parameters to display
        """
        # For each parameter coming from the state
        for key, value in state_parameters.items():
            # Ignore outcomes since changing it would require changing the actual implementation of the state
            if key == "outcomes":
                continue
            # Get any default value
            default_value = re.findall("\"(.*?)\"", value)
            if len(default_value) > 1:
                default_value = ", ".join(default_value)
            else:
                default_value = value
            # Add the configuration slot
            self.add_configuration_slot(key, placeholder_text=default_value)


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


class StateScrollingArea(QScrollArea):

    """
        Customized scrolling area that allows for a more natural scrolling effect
    """

    def __init__(self, widget, parent=None):
        """
            Initialize the widget and adjust its size

            @param widget: QWidget the scrolling area will contain
            @param parent: Parent of this widget (QWidget)
        """
        super(StateScrollingArea, self).__init__(parent=parent)
        # Set the widget inside the scrolling area
        self.setWidget(widget)
        # By default, we want just the mandatory parameters to be displayed without scrolling
        max_y = self.widget().findChild(QLineEdit, "line {}".format("input_keys")).y()
        self.setMaximumHeight(max_y)
        # Make sure the new height is applied
        self.adjustSize()

    def eventFilter(self, q_object, q_event):
        """
            Filter out the different events received by this widget

            @param q_object: QObject associated with the event
            @param q_event: Nature of the event (QEvent)
            @return: True of the event is fitlered, otherwise False
        """
        event_type = q_event.type()
        if event_type == QEvent.Wheel:
            # Get the target value of the scroll. The /6 allows for getting a better control inside the box
            target_scroll = self.verticalScrollBar().value() - q_event.angleDelta().y() / 6
            # Clamp the scroll when at min and max, which avoids to scroll in the view itself
            if target_scroll >= self.verticalScrollBar().maximum():
                self.verticalScrollBar().setValue(self.verticalScrollBar().maximum())
            elif target_scroll <= self.verticalScrollBar().minimum():
                self.verticalScrollBar().setValue(self.verticalScrollBar().minimum())
            # Apply the scroll
            else:
                self.verticalScrollBar().setValue(target_scroll)
            return True
        # Native event handler for all other events
        return super(StateScrollingArea, self).eventFilter(q_object, q_event)
