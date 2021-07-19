#!/usr/bin/env python
import sys
from math import pi

import geometry_msgs.msg
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import WrenchStamped
import std_msgs.msg

import moveit_msgs
import moveit_commander
from moveit_commander import conversions

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

        self.scene.remove_world_object("cylinder")

        ## force torque sensor
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.sub_ft_sensor = rospy.Subscriber('/iiwa/state/CartesianWrench', WrenchStamped, self._cb_ft_sensor, queue_size=1)

        ## publisher for spawing gazebo model
        self.model_spawner_publisher = rospy.Publisher('/model_spawner',std_msgs.msg.String, queue_size=10)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

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

    def add_object(self, pose):
        loginfo("ADD OBJECT")
        ## RVIZ
        cylinder_pose = geometry_msgs.msg.PoseStamped()
        cylinder_pose.header.frame_id = "world"
        cylinder_pose.pose = pose
        cylinder_name = "cylinder"
        self.scene.add_cylinder(cylinder_name, cylinder_pose, height=1.0, radius=0.05)

    def go_to_joint_state(self):
        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = 0
        joint_goal[2] = 0
        joint_goal[3] = 0
        joint_goal[4] = 0
        joint_goal[5] = 0
        joint_goal[6] = 0

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        loginfo ("INIT POSE1")
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        joint_goal[0] = pi/9
        joint_goal[1] = pi/8
        joint_goal[2] = 0
        joint_goal[3] = -pi/3
        joint_goal[4] = 0
        joint_goal[5] = pi/2
        joint_goal[6] = 0

        loginfo ("INIT POSE2")
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()



    def go_to_target_pose(self, reactive=True):
        move_group = self.move_group
        # move_group.clear_pose_targets()
        target_pose = geometry_msgs.msg.Pose()
        target_pose.orientation.x = 1.0
        target_pose.position.x = 0.35
        target_pose.position.y = -0.3
        target_pose.position.z = 0.4

        loginfo ("PLAN THE MOTION")
        move_group.set_pose_target(target_pose)
        plan = move_group.plan()
        # plan = move_group.go(wait=False)
        # plan, fraction = self.move_group.compute_cartesian_path([target_pose], 0.001, 0.0)


        # publish the obstacle
        rospy.sleep(1)
        self.model_spawner_publisher.publish("pub")
        object_pose = geometry_msgs.msg.Pose()
        object_pose.position.x = 0.3
        object_pose.position.y = -0.1
        object_pose.position.z = 0.5
        object_pose.orientation.w = 1.0
        self.add_object(object_pose)

        move_group.execute(plan[1], wait=False)
        # self.move_group.execute(plan, wait=False)
        # move_group.go(wait=False)

        # display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        # display_trajectory.trajectory_start = self.robot.get_current_state()
        # display_trajectory.trajectory.append(plan[1])
        # # Publish
        # self.display_trajectory_publisher.publish(display_trajectory)

        while True:
            force_y = self.eef_ft.force.y
            loginfo ("Force in y axis: {}".format(force_y))
            if reactive and force_y > 0.01:
                print ("Touched the cylinder")
                move_group.stop()
                move_group.clear_pose_targets()
                move_group.set_pose_target(target_pose)
                rospy.sleep(1)  # wait for 1 sec
                plan = move_group.go(wait=True)

                move_group.stop()
                move_group.clear_pose_targets()
                break
            rospy.sleep(0.01)

def main():
    # First initialize 'moveit commander' and a 'rospy' node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('plan_the_motion')
    rospy.sleep(1) # wait for 1 sec
    c = MainController()

    reactive = rospy.get_param('~reactive', True)
    loginfo('Reactive Planning: {}'.format(reactive))

    c.go_to_joint_state()
    c.go_to_target_pose(reactive = reactive)
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException():
        pass
