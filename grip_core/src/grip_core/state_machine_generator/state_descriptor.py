#!/usr/bin/env python

# Copyright 2019, 2020 Shadow Robot Company Ltd.
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


class StateDescriptor(object):

    """
        Class used to store all required information to describe a state
    """

    def __init__(self, state_description, parent_params):
        """
            Initialise the attributes of the class and load the state description

            @param state_description: Dictionary containing the names and values of the parameters of the state
            @param parent_params: Dictionary containing the parameters of the state's parent
        """
        # Initialise the parameters of the state
        self.parameters = {}
        # Contains the parent's parameters
        self.parent_parameters = parent_params.copy()
        # Will contain the name of the file in which the state is defined
        self.source_name = None
        # Will contain the type of state (StatePlan, StateMove, StateGrasp or custom made....)
        self.type = None
        # Will contain the transitions defined for this state
        self.transitions = None
        # Loads the parameters of the state
        self._load_params(state_description)

    def _load_params(self, state_description):
        """
            Load the parameters of the state

            @param state_description: Dictionary containing parameters name and values of the state
        """
        # If the field "params" is defined for the state, use it for initialising the corresponding attribute
        if "params" in state_description:
            self.parameters = state_description["params"]
        # Go through all the paremeters describing the state
        for parameter_name, parameter in state_description.items():
            # Store the type of the state (name of the state's class defined in the corresponding python file)
            if parameter_name == "source":
                self.set_source(parameter)
            # Store the transitions
            elif parameter_name == "transitions":
                self.transitions = parameter
            # filters what we want to store in the parameters attribute
            elif parameter_name not in ["params", "states", "outcome_map", "default_outcome"]:
                # If we have "params.something", it means that we are using the parent's parameter named something
                # add str() to make the parameter iterable (can be int) without changing the outcome of the test
                if "params." in str(parameter):
                    self.parameters[parameter_name] = self.parent_parameters[parameter.replace("params.", "")]
                else:
                    self.parameters[parameter_name] = parameter

    def set_source(self, source):
        """
            Set the source name and the type of the state

            @param source: String containing the source of the state (name of the python file without .py)
        """
        self.source_name = source
        # Get the type from the source following PEP-8 naming rules
        self.type = "".join([word.capitalize() for word in source.split("_")])

    def __str__(self):
        """
            Return the string representation of class' content

            @return: String containing some of the information stored in the class
        """
        return "params: {} \ntype: {}".format(self.parameters, self.type)
