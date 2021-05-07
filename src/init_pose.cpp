#include "iiwa_ros/iiwa_ros.hpp"
#include <iiwa_ros/state/cartesian_pose.hpp>
#include <iiwa_ros/service/control_mode.hpp>
#include <iiwa_ros/conversions.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/package.h>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace std;
using moveit::planning_interface::MoveItErrorCode;

// Create MoveGroup
static const std::string PLANNING_GROUP = "manipulator";
static const std::string EE_LINK = "iiwa_link_ee";
/* RRTConnectkConfigDefault, RRTkConfigDefault, RRTstartkConfigDefault, TRRTkConfigDefault, ESTkConfigDefault
   SBLkConfigDefault, LBKPIECEkConfigDefault, BKPIECEkConfigDefault, PRMkConfigDefault, PRMstarkConfigDefault */
static const std::string PLANNER_ID = "RRTConnectkConfigDefault";  
static const std::string REFERENCE_FRAME = "arm_mount_link";

bool sim;

vector<string> split(string input, char delimiter){
    vector<string> ans;
    stringstream str(input);
    string temp;
    
    while(getline(str, temp, delimiter)){
        ans.push_back(temp);
    }
    
    return ans;
}

int main (int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "CommandRobotMoveit");
    ros::NodeHandle nh("~");
    nh.param("sim", sim, true);

    // ROS spinner.
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    std::string movegroup_name, ee_link, planner_id, reference_frame;
    geometry_msgs::PoseStamped init_pose, current_cartesian_position, command_cartesian_position, start, end;
    std::string joint_position_topic, cartesian_position_topic;
    std::vector<geometry_msgs::Pose> drawing_stroke;
    std::vector<geometry_msgs::Pose> linear_path;
    geometry_msgs::Pose drawing_point;
    geometry_msgs::Pose path_point;

    // Dynamic parameters. Last arg is the default value. You can assign these from a launch file.
    nh.param<std::string>("move_group", movegroup_name, PLANNING_GROUP);
    nh.param<std::string>("ee_link", ee_link, EE_LINK);
    nh.param<std::string>("planner_id", planner_id, PLANNER_ID);
    nh.param<std::string>("reference_frame", reference_frame, REFERENCE_FRAME);

    // Create Move Group
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0; // 0.0
    const double eef_step = 0.001; // 0.001
    const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    const moveit::core::LinkModel* link_model =
      move_group.getCurrentState()->getLinkModel(ee_link);

    // Configure Move Group
    move_group.setPlanningTime(0.5);
    //move_group.setPlannerId(PLANNING_GROUP+"[RRTConnectkConfigDefault]");
    move_group.setPlannerId(planner_id);  
    move_group.setEndEffectorLink(ee_link);
    move_group.setPoseReferenceFrame(reference_frame);
    //ROS_INFO("Planner ID: %s", move_group.getPlannerId().c_str());
    ROS_INFO("Planning frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());
    //ROS_INFO("Pose Reference Frame: %s", move_group.getPoseReferenceFrame().c_str());

    string line;
    bool init = false;
    double x, y, z, fraction;
    MoveItErrorCode success_plan = MoveItErrorCode::FAILURE, motion_done = MoveItErrorCode::FAILURE;
    
    // initialization before start drawing
    while (ros::ok() && !init){      
        ros::Duration(5).sleep(); // wait for 2 sec
        
        ROS_INFO("Sleeping 5 seconds before starting ... ");

        // set all the joint values to the init joint position
        move_group.setStartStateToCurrentState();
        move_group.setJointValueTarget("iiwa_joint_1", 0.0);
        move_group.setJointValueTarget("iiwa_joint_2", 0.435332);
        move_group.setJointValueTarget("iiwa_joint_3", 0.0);
        move_group.setJointValueTarget("iiwa_joint_4", -1.91986);
        move_group.setJointValueTarget("iiwa_joint_5", 0.0);
        move_group.setJointValueTarget("iiwa_joint_6", -0.785399);
        move_group.setJointValueTarget("iiwa_joint_7", 0.0);
        success_plan = move_group.plan(my_plan);
        if (success_plan == MoveItErrorCode::SUCCESS) {
            motion_done = move_group.execute(my_plan);
            
        }
        ROS_INFO("Moved to the initial position");
        ROS_INFO("Sleeping 3 seconds before starting ... ");
        ros::Duration(3).sleep(); // wait for 3 sec
    }

    cerr<<"Stopping spinner..."<<endl;
    spinner.stop();

    cerr<<"Bye!"<<endl;

    return 0;
}