#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <iiwa_msgs/MoveToJointPositionAction.h>
#include <iiwa_msgs/MoveAlongSplineAction.h>
#include <iiwa_msgs/SetPTPJointSpeedLimits.h>
#include <iiwa_msgs/SetPTPCartesianSpeedLimits.h>
#include <iiwa_msgs/SetEndpointFrame.h>
#include <iiwa_ros/state/cartesian_pose.hpp>

#include <iiwa_ros/iiwa_ros.hpp>
#include <iiwa_ros/state/cartesian_pose.hpp>
#include <iiwa_ros/service/control_mode.hpp>
#include <iiwa_ros/conversions.hpp>

#include <ros/package.h>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#define TXT_FILE "/input/heart_path_c.txt"
#define BACKWARD 0.05
#define TRANSLATE_UP 0.43
#define TARGET_SIZE 0.5

using namespace std;

// Create MoveGroup
static const std::string PLANNING_GROUP = "manipulator";
static const std::string EE_LINK = "tool_link";
/* RRTConnectkConfigDefault, RRTkConfigDefault, RRTstartkConfigDefault, TRRTkConfigDefault, ESTkConfigDefault
   SBLkConfigDefault, LBKPIECEkConfigDefault, BKPIECEkConfigDefault, PRMkConfigDefault, PRMstarkConfigDefault */
static const std::string PLANNER_ID = "RRTConnectkConfigDefault";  
static const std::string REFERENCE_FRAME = "iiwa_link_0";

bool sim;

static iiwa_msgs::SplineSegment getSplineSegment (geometry_msgs::Pose waypoint_pose, int type = iiwa_msgs::SplineSegment::SPL) {
	iiwa_msgs::SplineSegment segment;
	// Segment type
	segment.type = type;
	// Header
	segment.point.poseStamped.header.frame_id = REFERENCE_FRAME;
	// Pose
	segment.point.poseStamped.pose.position.x = waypoint_pose.position.x;
	segment.point.poseStamped.pose.position.y = waypoint_pose.position.y;
	segment.point.poseStamped.pose.position.z = waypoint_pose.position.z;
	// Orientation
	segment.point.poseStamped.pose.orientation.x = waypoint_pose.orientation.x;
	segment.point.poseStamped.pose.orientation.y = waypoint_pose.orientation.y;
	segment.point.poseStamped.pose.orientation.z = waypoint_pose.orientation.z;
	segment.point.poseStamped.pose.orientation.w = -waypoint_pose.orientation.w;
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

static bool setEndpointFrame(ros::NodeHandle& nh, std::string frameId = EE_LINK) {
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

vector<string> split(string input, char delimiter){
  vector<string> ans;
  stringstream str(input);
  string temp;

  while(getline(str, temp, delimiter))
    ans.push_back(temp);
  return ans;
}

int main (int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "CommandRobotIIWA");
  ros::NodeHandle nh("~");

  // // ROS spinner.
  // ros::AsyncSpinner spinner(1);
  // spinner.start();

  // iiwa_ros::iiwa_ros my_iiwa;
  // my_iiwa.init();
  // for Cartesian Impedance Control
  iiwa_ros::state::CartesianPose iiwa_pose_state;
  iiwa_ros::service::ControlModeService iiwa_control_mode;
  // Low stiffness only along Z.
  iiwa_msgs::CartesianQuantity cartesian_stiffness = iiwa_ros::conversions::CartesianQuantityFromFloat(1500,1500,350,300,300,300);
  iiwa_msgs::CartesianQuantity cartesian_damping = iiwa_ros::conversions::CartesianQuantityFromFloat(0.7);
  actionlib::SimpleActionClient<iiwa_msgs::MoveToJointPositionAction> jointPositionClient("/iiwa/action/move_to_joint_position", true);
	actionlib::SimpleActionClient<iiwa_msgs::MoveAlongSplineAction> splineMotionClient("/iiwa/action/move_along_spline", true);

  iiwa_pose_state.init("iiwa");
  iiwa_control_mode.init("iiwa");

  std::string movegroup_name, ee_link, planner_id, reference_frame;
  geometry_msgs::PoseStamped init_pose, current_cartesian_position, command_cartesian_position, start, end;
  std::string joint_position_topic, cartesian_position_topic;
  std::vector<geometry_msgs::Pose> drawing_stroke;
  std::vector<geometry_msgs::Pose> linear_path;
  geometry_msgs::Pose drawing_point;
  geometry_msgs::Pose path_point;
  iiwa_msgs::CartesianPose iiwa_cartesian_position;
  iiwa_msgs::MoveAlongSplineGoal splineMotion;

  // // Dynamic parameters. Last arg is the default value. You can assign these from a launch file.
  // nh.param<std::string>("move_group", movegroup_name, PLANNING_GROUP);
  // nh.param<std::string>("ee_link", ee_link, EE_LINK);
  // nh.param<std::string>("planner_id", planner_id, PLANNER_ID);
  // nh.param<std::string>("reference_frame", reference_frame, REFERENCE_FRAME);


  // Set speed limit for motions in joint coordinates
  if (!setPTPJointSpeedLimits(nh)) {
      return 1;
  }

  // Set speed limits for motions in cartesian coordinates
  if (!setPTPCartesianSpeedLimits(nh)) {
      return 1;
  }

  // Set endpoint frame to flange, so that our Cartesian target coordinates are tool independent
  if (!setEndpointFrame(nh, reference_frame)) {
      return 1;
  }

  ROS_INFO("Waiting for action servers to start...");
  // Wait for the action servers to start
  jointPositionClient.waitForServer(); //will wait for infinite time
  splineMotionClient.waitForServer();
    

  // TXT file with list of coordinates
  ifstream txt(ros::package::getPath("iiwa_examples")+TXT_FILE);
  // check if text file is well opened
  if(!txt.is_open()){
    cout << "FILE NOT FOUND" << endl;
    return 1;
  }

  string line;
  bool init = false;
  double x, y, z, fraction;
  // MoveItErrorCode success_plan = MoveItErrorCode::FAILURE, motion_done = MoveItErrorCode::FAILURE;

  // initialization before start drawing
  while (ros::ok() && !init){
    ros::Duration(5).sleep(); // wait for 2 sec

    ROS_INFO("Sleeping 5 seconds before starting ... ");


    ROS_INFO("Action server started, moving to start pose...");
    // Define a goal
    iiwa_msgs::MoveToJointPositionGoal jointPositionGoal;
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

    ROS_INFO("Moved to the initial position");
    ROS_INFO("Sleeping 3 seconds before starting ... ");
    ros::Duration(3).sleep(); // wait for 3 sec

    // init_pose = move_group.getCurrentPose(ee_link);


    // find distance between wall and set as x-value
    ROS_INFO("The robot will be now set in Cartesian Impedance Mode");

    iiwa_control_mode.setCartesianImpedanceMode(cartesian_stiffness, cartesian_damping);
    iiwa_cartesian_position = iiwa_pose_state.getPose();
    command_cartesian_position = iiwa_cartesian_position.poseStamped;
    splineMotion.spline.segments.push_back(getSplineSegment(command_cartesian_position.pose, iiwa_msgs::SplineSegment::LIN));
    iiwa_cartesian_position.poseStamped.pose.position.x += BACKWARD;
    splineMotion.spline.segments.push_back(getSplineSegment(command_cartesian_position.pose, iiwa_msgs::SplineSegment::LIN));

    // Execute motion
    splineMotionClient.sendGoal(splineMotion);
    splineMotionClient.waitForResult();
    splineMotion.spline.segments.clear();


    ROS_INFO("Detecting the wall");
    ROS_INFO("Sleeping 3 seconds before starting ... ");
    ros::Duration(3).sleep(); // wait for 3 sec

    iiwa_cartesian_position = iiwa_pose_state.getPose();
    command_cartesian_position = iiwa_cartesian_position.poseStamped;
    splineMotion.spline.segments.push_back(getSplineSegment(command_cartesian_position.pose, iiwa_msgs::SplineSegment::LIN));
    iiwa_cartesian_position.poseStamped.pose.position.x -= BACKWARD;
    splineMotion.spline.segments.push_back(getSplineSegment(command_cartesian_position.pose, iiwa_msgs::SplineSegment::LIN));

    // Execute motion
    splineMotionClient.sendGoal(splineMotion);
    splineMotionClient.waitForResult();
    splineMotion.spline.segments.clear();

    x += 0.02; // default x position of the wall


    init = true;

    iiwa_cartesian_position = iiwa_pose_state.getPose();
    current_cartesian_position = iiwa_cartesian_position.poseStamped;

    command_cartesian_position = current_cartesian_position;
    drawing_point = current_cartesian_position.pose;

    // linear_path.push_back(current_cartesian_position.pose);
  }

  int stroke_num = 0;
  bool ready_to_draw = false;
  while(ros::ok() && getline(txt, line) && init){
    if(line == "End"){
      stroke_num++;

      ROS_INFO("The robot will be now set in Cartesian Impedance Mode");
      // move forward first to draw
      ROS_INFO("Moving Forward ... ");

      iiwa_control_mode.setCartesianImpedanceMode(cartesian_stiffness, cartesian_damping);
      command_cartesian_position.pose = drawing_stroke[0];
      splineMotion.spline.segments.push_back(getSplineSegment(command_cartesian_position.pose, iiwa_msgs::SplineSegment::LIN));

      // Execute motion
      splineMotionClient.sendGoal(splineMotion);
      splineMotionClient.waitForResult();
      splineMotion.spline.segments.clear();


      // draw a stroke
      ROS_INFO("Drawing %d th stroke ...", stroke_num);

      for (int i = 0 ; i < drawing_stroke.size(); i++)
        splineMotion.spline.segments.push_back(getSplineSegment(drawing_stroke[i], iiwa_msgs::SplineSegment::SPL));

      // Execute motion
      splineMotionClient.sendGoal(splineMotion);
      splineMotionClient.waitForResult();
      splineMotion.spline.segments.clear();
      // visual_tools.publishTrajectoryLine(my_plan.trajectory_, link_model, joint_model_group, rviz_visual_tools::colors::WHITE);


      // getCurrentPose ignores the reference frame, thus get the latest position from drawing_stroke
      ROS_INFO("Moving Backward ... \n");
      command_cartesian_position.pose = drawing_stroke.back();

      // move backward
      // linear_path.push_back(command_cartesian_position.pose);
      command_cartesian_position.pose.position.x -= BACKWARD;

      iiwa_control_mode.setPositionControlMode();
      splineMotion.spline.segments.push_back(getSplineSegment(command_cartesian_position.pose, iiwa_msgs::SplineSegment::LIN));

      // Execute motion
      splineMotionClient.sendGoal(splineMotion);
      splineMotionClient.waitForResult();
      splineMotion.spline.segments.clear();

      // linear_path.push_back(command_cartesian_position.pose);

      ready_to_draw = false;
      drawing_stroke.clear();
    }
    else{
      // read drawing
      vector<string> tempSplit = split(line, ' ');
      y = stod(tempSplit[0]);
      z = stod(tempSplit[1])+TRANSLATE_UP;

      if (!ready_to_draw){
        // move to the ready position (off the wall)
        ROS_INFO("The robot will be now set in Position Control Mode");
        ROS_INFO("Moving To Ready Position ... ");
        command_cartesian_position.pose.position.x = x - BACKWARD;
        command_cartesian_position.pose.position.y = y;
        command_cartesian_position.pose.position.z = z;

        iiwa_control_mode.setPositionControlMode();
        splineMotion.spline.segments.push_back(getSplineSegment(command_cartesian_position.pose, iiwa_msgs::SplineSegment::LIN));

        // Execute motion
        splineMotionClient.sendGoal(splineMotion);
        splineMotionClient.waitForResult();
        splineMotion.spline.segments.clear();

        // linear_path.push_back(command_cartesian_position.pose);
        ready_to_draw = true;
      }

      drawing_point.position.x = x;
      drawing_point.position.y = y;
      drawing_point.position.z = z;
      drawing_stroke.push_back(drawing_point); // push the point

    }
  }

  // cerr<<"Stopping spinner..."<<endl;
  // spinner.stop();

  cerr<<"Bye!"<<endl;

  return 0;
}
