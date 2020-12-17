#!/usr/bin/env python

# Copyright 2019 Shadow Robot Company Ltd.
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

from config_parser import StateMachineConfigParser
from templater import StateMachineTemplater
import sys

if __name__ == '__main__':
    # Initialise the parser and templater
    parser = StateMachineConfigParser(sys.argv[1], sys.argv[2])
    templater = StateMachineTemplater(sys.argv[3], sys.argv[4])

    # For each state machine (starting with nested one) generate the corresponding file
    for state_machine in reversed(parser.state_machines):
        templater.generate_state_machine(state_machine)
