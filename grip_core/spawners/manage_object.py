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

import rospy
from gazebo_ros.gazebo_interface import spawn_sdf_model_client
from geometry_msgs.msg import Pose
import tf
import os
from gazebo_msgs.srv import DeleteModel
from grip_core.srv import AddGazeboMapping, DeleteGazeboMapping, UpdateACMEntry
import argparse


class ManageObject(object):

    """
        Class allowing users to either add or remove objects in Gazebo and in the collision matrix.
    """

    def __init__(self, object_type, mode, object_name, object_pose, reference_frame, file_path):
        """
            Initialise the attributes of the class

            @param object_type: Type of object to spawn. It must be the name of a folder contained in 'models' of your
                                description package
            @param mode: Boolean stating whether the specified object must be removed or add (True means deleting it)
            @param object_name: Optional name allowing to add several objects of the same type.
            @param object_pose: Pose message describing how the object must be spawned
            @param reference_frame: Provide the reference frame of the pose provided
            @param file_path: Optional path to the sdf file of the object to spawn, which allows to spawn objects not
                              in the 'models' folder
        """
        # Get the object name
        self.object_name = object_name
        # Get the reference frame
        self.reference_frame = reference_frame
        # Get the pose
        self.object_pose = object_pose
        # Store the path of the default environment in which we can find objects (description_package/models)
        self.default_env = rospy.get_param("gazebo_model_path")
        # Store the object type
        self.object_type = object_type
        # Get the file path
        self.file_path = os.path.abspath(file_path) if file_path else ""
        # Initialise the service to update the ACM
        self.update_acm = rospy.ServiceProxy("update_acm_entry", UpdateACMEntry)
        # Sanity check to make sure that everything works
        if not (self.file_path or self.object_type) and not mode:
            rospy.logerr("Either a file or a model name should be provided to add an object. Cannot proceed")
            return
        # If mode is set to false then spawn the object
        if not mode:
            self._spawn_sdf_object()
        # Otherwise delete it
        else:
            self._delete_object()

    def _spawn_sdf_object(self):
        """
            Function allowing to spawn an object desribed by a sdf file in gazebo, while allowing automatic collision
            generation in moveit and updating the ACM
        """
        # Access to the service through the proxy
        add_mapping_service = rospy.ServiceProxy('add_gazebo_mapping', AddGazeboMapping)
        # If a path is provided then set the object type accordingly
        if self.file_path:
            file_to_load = self.file_path
            self.object_type = self.file_path
            # If a name has not been set, then extract it from the path
            if not self.object_name:
                self.object_name = file_to_load.split("/")[-2]
        else:
            file_to_load = self.default_env + '{}/model.sdf'.format(self.object_type)
        # Read the xml file
        model_sdf_file = open(file_to_load, 'r').read()

        # Add the maping between the name of the object and the type of the object to make the link between
        # gazebo and moveit
        response = add_mapping_service(self.object_name, self.object_type)
        # If something went wrong dispaly an error message and quit
        if not response.success:
            rospy.logerr("An error occured when handling the gazebo object {}".format(self.object_name))
            return
        # Otherwise spawn the object in gazebo
        spawn_sdf_model_client(self.object_name, model_sdf_file, rospy.get_namespace(), self.object_pose,
                               self.reference_frame, "gazebo")
        # And update the ACM
        response = self.update_acm(0, self.object_name, True)

    def _delete_object(self):
        """
            Function allowing to remove an object by its name from gazebo and the collision matrix
        """
        try:
            # Get access to the service to remove object from gazebo and for removing the mapping
            delete_model = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
            delete_mapping_service = rospy.ServiceProxy('delete_gazebo_mapping', DeleteGazeboMapping)
            # Delete the mapping
            response = delete_mapping_service(self.object_name)
            if not response.success:
                rospy.logerr("An error occurred when deleting the gazebo object: {}".format(self.object_name))
                return
            # If everything is right then delete the object and update the ACM
            delete_model(model_name=self.object_name)
            response = self.update_acm(1, self.object_name, True)
            if not response.success:
                rospy.logwarn("The object has been removed from gazebo but not from the ACM")
        except rospy.ServiceException as exception:
            rospy.logerr("Delete model service call failed: %s", exception)


def triplet_floats_to_list(string_value):
    """
        Function parsing a string containing three floats to a list of three floats. For instance "-0.5 0.1 0.4" becomes
        [-0.5, 0.1, 0.4]

        @param string_value: String containing three elements separated by spaces such as "a b c"
        @return List corresponding to the input list
    """
    list_float = string_value.split()
    # Check that it contains the proper number of elements
    if len(list_float) != 3:
        raise argparse.ArgumentError
    return map(float, list_float)

if __name__ == '__main__':
    # Initialise the node
    rospy.init_node('manage_object')

    # Set an argument parser to get get all the parameters nicely
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--delete', choices=["False", "false", "true", "True"],
                        help='Use this flag if you want to delete an object')
    parser.add_argument('-p', '--position', type=triplet_floats_to_list, default=[0, 0, 0],
                        help='Position, use as three floats separated by a space. Must be between quotes')
    parser.add_argument('-o', '--orientation', type=triplet_floats_to_list, default=[0, 0, 0],
                        help='Orientation (rpy), use as three floats separated by a space. Must be between quotes')
    parser.add_argument('-n', '--name', default="", help="Name given to the object to spawn")
    parser.add_argument('-t', '--type', default="", help="Type of object to spawn")
    parser.add_argument('-r', '--reference', default="world",
                        help="Reference frame in which the position and orientationis given")
    parser.add_argument('-f', '--file', default="", help="Path to the sdf file to load")
    # Parse the arguments
    args = parser.parse_args(rospy.myargv()[1:])

    # Set the name of the object depending on the input
    args.name = args.type if not args.name else args.name

    # Create the Pose message from the position and orientation asked as input
    pose = Pose()
    pose.position.x = args.position[0]
    pose.position.y = args.position[1]
    pose.position.z = args.position[2]
    quat = tf.transformations.quaternion_from_euler(args.orientation[0], args.orientation[1], args.orientation[2])
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]
    # Provide everything to the class
    manage_object = ManageObject(args.type, args.delete in ["True", "true"], args.name, pose, args.reference, args.file)
