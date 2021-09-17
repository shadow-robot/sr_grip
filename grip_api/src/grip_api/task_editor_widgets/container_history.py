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
    def __init__(self, container, memory_length=32):
        """
            Initialize the object

            @param container: Container object from which we want to store snapshots
            @param memory_length: Integer (> 1) that describes the maximum number of previous steps to keep in memory
        """
        # Store the container
        self.container = container
        # Remove one from the memory_length since we start with index=0
        self.memory_length = memory_length - 1
        # Make sure to have everything empty
        self.clear()

    def clear(self):
        """
            Empty the history stack, reset the step counter and initial snapshot
        """
        self.history_stack = list()
        self.history_step = -1
        self.initial_snapshot = None

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

    def set_initial_snapshot(self):
        """
            Store the state of the container and set it as initial
        """
        self.initial_snapshot = self.container.save().copy()

    def store_current_history(self):
        """
            Store the current content of the container associated to this object
        """
        # If the current step is not the latest one
        if self.can_redo():
            # Discard all the other "future" steps
            self.history_stack = self.history_stack[:self.history_step + 1]

        # If the history stack is full, remove the oldest step
        if self.history_step == self.memory_length:
            self.history_stack.pop(0)
            self.history_step -= 1

        # If it is the first history step, then set the initial snapshot
        if self.history_step == -1 and self.initial_snapshot is None:
            self.set_initial_snapshot()

        # Store the current state of the container and emit a signal if the state of the container has changed
        self.evaluate_snapshot(store=True)

    def evaluate_snapshot(self, store=False):
        """
            Evaluate the current state of the container (snapshot) and emit a signal if the two states are different.
            The current state is by default not saved into the history stack

            @param store: Boolean specifying if the snapshot should be added to the history stack
        """
        snapshot = self.container.save()
        if store:
            # Storing the current state of the container
            self.history_stack.append(snapshot)
            # Increment the history step
            self.history_step += 1
        # Emit a signal stating whether the state of the container has changed or not
        self.emit_signal(snapshot)

    def restore_history(self):
        """
            Restore the state of the container at a given time history_step
        """
        snapshot = self.history_stack[self.history_step]
        self.container.restore(snapshot)
        # Emit a signal stating whether the restored snapshot is different from the initial one or not
        self.emit_signal(snapshot)

    def emit_signal(self, snapshot):
        """
            Make the GraphicalEditorWidget object emit a signal that specifies if the content of the container has been
            modified

            @param snapshot: Dictionary obtained by running container.save()
        """
        # Don't send anything if the initial snapshot is None
        if self.initial_snapshot is None:
            return

        # We need to have the sorted because each value is a list and the order of the elements might change
        is_different = any(sorted(x) != sorted(y) for x, y in zip(self.initial_snapshot.values(), snapshot.values()))
        self.container.editor_widget.hasBeenModified.emit(is_different)
