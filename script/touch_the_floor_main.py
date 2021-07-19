#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import sys

import geometry_msgs.msg
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import WrenchStamped
from math import pi

import moveit_commander
from moveit_commander import conversions

from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel

import rospy
import tf
import tf2_geometry_msgs
import tf2_ros

import termcolor


def loginfo(msg):
    rospy.loginfo('[{}] {}'.format(rospy.get_name(),
                                   termcolor.colored(msg, 'green')))

class MainController(object):
    def __init__(self):
        super(MainController, self).__init__()

        ## Instantiate a 'RobotCommander' object. This provides information such as robot's
        ## kinematic model and the robot's current joint states:
        robot = moveit_commander.RobotCommander()

        ## Instantiate a 'PlanningSceneInterface' object. This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a 'MoveGroupCommander' object. This object is an interface
        ## to a planning group (group of joints):
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        # end-effector link of the group
        ee_link = move_group.get_end_effector_link()

        # Misc variables
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.ee_link = ee_link
        self.eef_ft = None

        ## force torque sensor
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.sub_ft_sensor = rospy.Subscriber('/iiwa/state/CartesianWrench', WrenchStamped, self._cb_ft_sensor, queue_size=1)

        # ## gazebo model
        # loginfo("ADD GAZEBO MODEL")
        # rospy.wait_for_service("gazebo/spawn_urdf_model")
        # rospy.wait_for_service("gazebo/delete_model")
        # loginfo("Gazebo Service")
        # spawn_model = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
        # delete_model = rospy.ServicePoxy("gazebo/delete_model", DeleteModel)
        #
        # spawn_model(
        #     model_name='simple_box',
        #     model_xml=open('/home/daeun/ros_ws/src/iiwa_examples/model/simple_box.urdf'),
        #     robot_namespace='/',
        #     initial_pose=geometry_msgs.msg.Pose(),
        #     reference_frame='world'
        # )

    def _cb_ft_sensor(self,msg):
        force = msg.wrench.force
        torque = msg.wrench.torque
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame='world',
                source_frame=msg.header.frame_id,
                time=msg.header.stamp,
                timeout=rospy.Duration(2.0),
            )
        except Exception:
            link_pose = self.robot._r.get_link_pose(msg.header.frame_id)
            transform = TransformStamped(
                header = msg.header,
                transform = conversions.list_to_transform(link_pose)
            )
            transform.header.frame_id = self.robot.get_planning_frame()

        force = tf2_geometry_msgs.do_transform_vector3(
            Vector3Stamped(header=msg.header, vector=force), transform).vector
        torque = tf2_geometry_msgs.do_transform_vector3(
            Vector3Stamped(header=msg.header, vector=torque), transform).vector
        self.eef_ft = Wrench(force=force, torque=torque)

    def add_object(self):
        loginfo("ADD OBJECT")

        ## RVIZ
        cylinder_pose = geometry_msgs.msg.PoseStamped()
        cylinder_pose.header.frame_id = "world"
        cylinder_pose.pose = self.move_group.get_current_pose().pose
        cylinder_pose.pose.position.x = 0.56
        cylinder_pose.pose.position.z = 0.05
        cylinder_name = "cylinder"
        self.scene.add_cylinder(cylinder_name, cylinder_pose, height=1.0, radius=0.05)

        ## GAZEBO


    def go_to_joint_state(self):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = pi/4
        joint_goal[2] = 0
        joint_goal[3] = -pi/2
        joint_goal[4] = 0
        joint_goal[5] = pi/4
        joint_goal[6] = 0

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        loginfo ("INIT POSE")
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

    def go_down_to_floor(self, reactive=True):
        # self.move_group.clear_pose_targets()
        pose = self.move_group.get_current_pose().pose
        pose.orientation = Quaternion(
            *tf.transformations.quaternion_from_euler(-3.14, 0, 0))
        # self.move_group.set_pose_target(pose)
        # self.move_group.go(wait=True)
        rospy.sleep(1)

        loginfo ("GOING DOWN")
        self.move_group.clear_pose_targets()
        pose.position.z = -1.0
        self.move_group.clear_pose_targets()
        plan, fraction = self.move_group.compute_cartesian_path([pose], 0.001, 0.0)
        self.move_group.execute(plan, wait=False)

        while True:
            force_z = self.eef_ft.force.z
            loginfo ("Force in z axis: {}".format(force_z))
            if reactive and force_z > 0:
                print ("Touched the floor")
                self.move_group.stop()
                self.move_group.clear_pose_targets()
                pose = self.move_group.get_current_pose().pose
                pose.position.z += 0.01
                plan, fraction = self.move_group.compute_cartesian_path([pose], 0.01, 0.0)
                self.move_group.execute(plan, wait=True)
                break
            rospy.sleep(0.01)

def main():
    # First initialize 'moveit commander' and a 'rospy' node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('touch_the_ground')
    rospy.sleep(1) # wait for 1 sec
    c = MainController()

    reactive = rospy.get_param('~reactive', True)
    loginfo('Reactive Planning: {}'.format(reactive))

    c.go_to_joint_state()
    # c.add_object()
    # c.go_down_to_floor(reactive = reactive)
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException():
        pass
