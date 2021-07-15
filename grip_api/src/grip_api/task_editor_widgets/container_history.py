#!/usr/bin/env python

# Copyright 2021 Shadow Robot Company Ltd.
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


class ContainerHistory(object):
    """
        Object that stores snapshots of the state of a given container at a given time
    """
    def __init__(self, container, memory_length=16):
        """
            Initialize the object

            @param container: Container object from which we want to store snapshots
            @param memory_length: Integer (> 1) that describes the maximum nuber of previous steps to keep in memory
        """
        # Store the container
        self.container = container
        # Remove one from the memory_length since we start with index=0
        self.memory_length = memory_length - 1
        # Make sure to have everything empty
        self.clear()

    def clear(self):
        """
            Empty the history stack and reset the step counter
        """
        self.history_stack = list()
        self.history_step = -1

    def can_undo(self):
        """
            Return True if the operation undo can be performed, False otherwise

            @return: True if undo can be performed else return False
        """
        return self.history_step > 0

    def can_redo(self):
        """
            Return True if the operation redo can be performed, False otherwise

            @return: True if redo can be performed else return False
        """
        return self.history_step < len(self.history_stack) - 1

    def undo(self):
        """
            Perform the undo operation (restore the previous state of the container that has been stored)
        """
        if self.can_undo():
            # Get the last but one snapshot
            self.history_step -= 1
            # Restore the given snapshot
            self.restore_history()

    def redo(self):
        """
            Perform the redo operation (restore a state that has been previously canceled)
        """
        if self.can_redo():
            # Get the "future" next snapshot
            self.history_step += 1
            # Restore the given snapshot
            self.restore_history()

    def store_current_history(self, set_modified=False):
        """
            Store the current content of the container associated to this object

            @param set_modified: Boolean specifying whether the TaskEditorArea object should have the can_be_saved
                                 updated or not
        """
        # Change the can_be_saved flag of the task editor area if required
        if set_modified:
            # TODO: Send a signal so that the Task Editor area gets the can_be_saved updated
            task_editor_area = self.container.editor_widget.parent().parent().parent().parent()

        # If the current step is not the latest one
        if self.can_redo():
            # Discard all the other "future" steps
            self.history_stack = self.history_stack[:self.history_step + 1]

        # If the history stack is full, remove the oldest step
        if self.history_step == self.memory_length:
            self.history_stack.pop(0)
            self.history_step -= 1

        # Storing the current state of the container
        self.history_stack.append(self.container.save())
        # Increment the history step
        self.history_step += 1

    def restore_history(self):
        """
            Restore the state of the container at a given time history_step
        """
        snapshot = self.history_stack[self.history_step]
        self.container.restore(snapshot)
