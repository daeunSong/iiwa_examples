#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <iiwa_msgs/MoveToJointPositionAction.h>
#include <iiwa_msgs/MoveAlongSplineAction.h>
#include <iiwa_msgs/SetPTPJointSpeedLimits.h>
#include <iiwa_msgs/SetPTPCartesianSpeedLimits.h>
#include <iiwa_msgs/SetEndpointFrame.h>
#include <iiwa_ros/state/cartesian_pose.hpp>

static iiwa_msgs::SplineSegment getSplineSegment (geometry_msgs::Pose waypoint_pose, int type = iiwa_msgs::SplineSegment::SPL) {
	iiwa_msgs::SplineSegment segment;
	// Segment type
	segment.type = type;
	// Header
	segment.point.poseStamped.header.frame_id = "iiwa_link_0";
	// Pose
	segment.point.poseStamped.pose.position.x = waypoint_pose.position.x;
	segment.point.poseStamped.pose.position.y = waypoint_pose.position.y;
	segment.point.poseStamped.pose.position.z = waypoint_pose.position.z;
	// Orientation
	segment.point.poseStamped.pose.orientation.x = waypoint_pose.orientation.x;
	segment.point.poseStamped.pose.orientation.y = waypoint_pose.orientation.y;
	segment.point.poseStamped.pose.orientation.z = waypoint_pose.orientation.z;
	segment.point.poseStamped.pose.orientation.w = waypoint_pose.orientation.w;
	// Redundancy
	segment.point.redundancy.status = -1;
	segment.point.redundancy.turn = -1;

	return segment;
}

static bool setPTPJointSpeedLimits(ros::NodeHandle& nh) {
	ROS_INFO("Setting PTP joint speed limits...");
	ros::ServiceClient setPTPJointSpeedLimitsClient = nh.serviceClient<iiwa_msgs::SetPTPJointSpeedLimits>("/iiwa/configuration/setPTPJointLimits");
	iiwa_msgs::SetPTPJointSpeedLimits jointSpeedLimits;
	jointSpeedLimits.request.joint_relative_velocity = 0.2;
	jointSpeedLimits.request.joint_relative_acceleration = 0.5;
	if (!setPTPJointSpeedLimitsClient.call(jointSpeedLimits)) {
		ROS_ERROR("Service call failed.");
		return false;
	}
	else if (!jointSpeedLimits.response.success) {
		ROS_ERROR_STREAM("Service call returned error: "+jointSpeedLimits.response.error);
		return false;
	}

	ROS_INFO("Done.");
	return true;
}

static bool setPTPCartesianSpeedLimits(ros::NodeHandle& nh) {
	ROS_INFO("Setting PTP Cartesian speed limits...");
	ros::ServiceClient setPTPCartesianSpeedLimitsClient = nh.serviceClient<iiwa_msgs::SetPTPCartesianSpeedLimits>("/iiwa/configuration/setPTPCartesianLimits");
	iiwa_msgs::SetPTPCartesianSpeedLimits cartesianSpeedLimits;
	cartesianSpeedLimits.request.maxCartesianVelocity = 0.5;
	cartesianSpeedLimits.request.maxCartesianAcceleration = 0.5;
	cartesianSpeedLimits.request.maxCartesianJerk = -1.0; // ignore
	cartesianSpeedLimits.request.maxOrientationVelocity = 0.5;
	cartesianSpeedLimits.request.maxOrientationAcceleration = 0.5;
	cartesianSpeedLimits.request.maxOrientationJerk = -1.0; // ignore
	if (!setPTPCartesianSpeedLimitsClient.call(cartesianSpeedLimits)) {
		ROS_ERROR("Failed.");
		return false;
	}
	else if (!cartesianSpeedLimits.response.success) {
		ROS_ERROR_STREAM("Service call returned error: "+cartesianSpeedLimits.response.error);
		return false;
	}

	ROS_INFO("Done.");
	return true;
}

static bool setEndpointFrame(ros::NodeHandle& nh, std::string frameId = "iiwa_link_ee") {
	ROS_INFO_STREAM("Setting endpoint frame to \""<<frameId<<"\"...");
	ros::ServiceClient setEndpointFrameClient = nh.serviceClient<iiwa_msgs::SetEndpointFrame>("/iiwa/configuration/setEndpointFrame");
	iiwa_msgs::SetEndpointFrame endpointFrame;
	endpointFrame.request.frame_id = frameId;
	if (!setEndpointFrameClient.call(endpointFrame)) {
		ROS_ERROR("Failed.");
		return false;
	}
	else if (!endpointFrame.response.success) {
		ROS_ERROR_STREAM("Service call returned error: "+endpointFrame.response.error);
		return false;
	}

	ROS_INFO("Done.");
	return true;
}

int main (int argc, char **argv)
{
	ros::init(argc, argv, "move_spline");
	ros::NodeHandle nh;


	// Set speed limit for motions in joint coordinates
	if (!setPTPJointSpeedLimits(nh)) {
		return 1;
	}

	// Set speed limits for motions in cartesian coordinates
	if (!setPTPCartesianSpeedLimits(nh)) {
		return 1;
	}

	// Set endpoint frame to flange, so that our Cartesian target coordinates are tool independent
	if (!setEndpointFrame(nh)) {
		return 1;
	}

    iiwa_ros::state::CartesianPose iiwa_pose_state;
    iiwa_pose_state.init("iiwa");

	// Create the action clients
	// Passing "true" causes the clients to spin their own threads
	actionlib::SimpleActionClient<iiwa_msgs::MoveToJointPositionAction> jointPositionClient("/iiwa/action/move_to_joint_position", true);
	actionlib::SimpleActionClient<iiwa_msgs::MoveAlongSplineAction> splineMotionClient("/iiwa/action/move_along_spline", true);

	ROS_INFO("Waiting for action servers to start...");
	// Wait for the action servers to start
	jointPositionClient.waitForServer(); //will wait for infinite time
	splineMotionClient.waitForServer();

	// ROS spinner.
    ros::AsyncSpinner spinner(1);
    spinner.start();
	ROS_INFO("Spinner started...");

	ROS_INFO("Action server started, moving to start pose...");
	// Define a goal
	iiwa_msgs::MoveToJointPositionGoal jointPositionGoal;
	// jointPositionGoal.joint_position.position.a1 =  0.00;
	// jointPositionGoal.joint_position.position.a2 =  0.19;
	// jointPositionGoal.joint_position.position.a3 =  0.00;
	// jointPositionGoal.joint_position.position.a4 = -1.40;
	// jointPositionGoal.joint_position.position.a5 =  0.00;
	// jointPositionGoal.joint_position.position.a6 =  1.56;
	// jointPositionGoal.joint_position.position.a7 =  0.00;
	jointPositionGoal.joint_position.position.a1 =  0.00;
	jointPositionGoal.joint_position.position.a2 =  0.435332;
	jointPositionGoal.joint_position.position.a3 =  0.00;
	jointPositionGoal.joint_position.position.a4 = -1.91986;
	jointPositionGoal.joint_position.position.a5 =  0.00;
	jointPositionGoal.joint_position.position.a6 = -0.785399;
	jointPositionGoal.joint_position.position.a7 =  0.00;
	// Send goal to action server
	jointPositionClient.sendGoal(jointPositionGoal);

	// Wait for the action to finish
	bool finished_before_timeout = jointPositionClient.waitForResult(ros::Duration(60.0));

	if (!finished_before_timeout) {
		ROS_WARN("iiwa motion timed out - exiting...");
		return 0;
	}
	else if (!jointPositionClient.getResult()->success) {
		ROS_ERROR("Action execution failed - exiting...");
		return 0;
	}

    ros::Duration(3).sleep(); // wait for 3 sec

    // Get Current Position
    iiwa_msgs::CartesianPose command_cartesian_position;
    command_cartesian_position = iiwa_pose_state.getPose();


	ROS_INFO_STREAM("Executing spline motion...");
	iiwa_msgs::MoveAlongSplineGoal splineMotion;

	// Current position (manually read from /iiwa/state/CartesianPose on our iiwa 14):
	// const double x = 0.478509765292;
	// const double y = 0;
	// const double z = 0.613500561539;
	// const double x = command_cartesian_position.poseStamped.pose.position.x;//0.478509765292;
	// const double y = command_cartesian_position.poseStamped.pose.position.y;//0;
	// const double z = command_cartesian_position.poseStamped.pose.position.z;//0.613500561539;
	// double a = command_cartesian_position.poseStamped.pose.orientation.x;
	// double b = command_cartesian_position.poseStamped.pose.orientation.y;
	// double c = command_cartesian_position.poseStamped.pose.orientation.z;
	// double w = command_cartesian_position.poseStamped.pose.orientation.w;

	// std::cout << a << b << c << w << std::endl;

	// Set up a spline segment that we use as a base for all motion points
	// The motion will look similar to this:
	//
	//   (5)---> (1) ----(2)
	//     \             /
	//      \           /
	//       \         /
	//       (4)     (3)
	//         \     /
	//          `---´
	//

	// 1. Add current position as first point.
	// This is not mandatory but ensures that the first linear segment looks exactly as we want it to
	splineMotion.spline.segments.push_back(getSplineSegment(command_cartesian_position.poseStamped.pose, iiwa_msgs::SplineSegment::SPL));

	command_cartesian_position.poseStamped.pose.position.y += 0.1;
	splineMotion.spline.segments.push_back(getSplineSegment(command_cartesian_position.poseStamped.pose, iiwa_msgs::SplineSegment::LIN));

	command_cartesian_position.poseStamped.pose.position.y -= 0.05;
	command_cartesian_position.poseStamped.pose.position.z -= 0.1;
	splineMotion.spline.segments.push_back(getSplineSegment(command_cartesian_position.poseStamped.pose, iiwa_msgs::SplineSegment::LIN));

	command_cartesian_position.poseStamped.pose.position.y -= 0.1;
	splineMotion.spline.segments.push_back(getSplineSegment(command_cartesian_position.poseStamped.pose, iiwa_msgs::SplineSegment::SPL));

	command_cartesian_position.poseStamped.pose.position.y -= 0.05;
	command_cartesian_position.poseStamped.pose.position.z += 0.1;
	splineMotion.spline.segments.push_back(getSplineSegment(command_cartesian_position.poseStamped.pose, iiwa_msgs::SplineSegment::LIN));

	command_cartesian_position.poseStamped.pose.position.y += 0.1;
	splineMotion.spline.segments.push_back(getSplineSegment(command_cartesian_position.poseStamped.pose, iiwa_msgs::SplineSegment::LIN));

	// // 2. Move linear to the side
	// splineMotion.spline.segments.push_back(getSplineSegment(x, y+0.1, z, a, b, c, w, iiwa_msgs::SplineSegment::LIN));

	// // 3. Move diagonally down, following a line
	// splineMotion.spline.segments.push_back(getSplineSegment(x, y+0.05, z-0.1, a, b, c, w,  iiwa_msgs::SplineSegment::LIN));

	// // 4. Do a bow
	// splineMotion.spline.segments.push_back(getSplineSegment(x, y-0.05, z-0.1, a, b, c, w, iiwa_msgs::SplineSegment::SPL));

	// // 5. Move diagonally up
	// splineMotion.spline.segments.push_back(getSplineSegment(x, y-0.1, z, a, b, c, w, iiwa_msgs::SplineSegment::LIN));

	// // 6. Do Linear motion back to start pose
	// splineMotion.spline.segments.push_back(getSplineSegment(x, y, z, a, b, c, w, iiwa_msgs::SplineSegment::LIN));

	 // Execute motion
	splineMotionClient.sendGoal(splineMotion);
	splineMotionClient.waitForResult();

	ROS_INFO("Done.");

	//exit
	return 0;
}