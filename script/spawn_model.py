#!/usr/bin/python
import sys
import rospy
import os

import geometry_msgs.msg
from std_msgs.msg import String
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
import tf.transformations as tft

def cb_model_spawner(msg):
    if (msg.data == "pub"):
        print("recieved!")
        rospy.wait_for_service("gazebo/spawn_urdf_model")
        # rospy.wait_for_service("gazebo/delete_model")
        spawn_model = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
        # delete_model = rospy.ServicePoxy("gazebo/delete_model", DeleteModel)

        object_pose = geometry_msgs.msg.Pose()
        object_pose.position.x = 0.3
        object_pose.position.y = -0.1
        object_pose.position.z = 0.5
        object_pose.orientation.w = 1.0

        spawn_model(
            model_name='simple_box',
            model_xml=open('/home/glab/ros_ws/src/iiwa_examples/script/models/simple_box.urdf').read(),
            robot_namespace='/',
            initial_pose=object_pose,
            reference_frame='world'
        )

if __name__ == "__main__":
    rospy.init_node("object_spawner")

    # this was missing....
    rospy.sleep(1)

    ## gazebo model
    sub_model_spawner = rospy.Subscriber('/model_spawner', String, cb_model_spawner)
    rospy.spin()


