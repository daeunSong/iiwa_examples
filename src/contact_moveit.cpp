#include "iiwa_ros/iiwa_ros.hpp"
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
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

#define TXT_FILE "/input/Bear_Coordinates.txt"
#define BACKWARD 0.01

using namespace std;
using namespace Eigen;
using moveit::planning_interface::MoveItErrorCode;

// Create MoveGroup
static const std::string PLANNING_GROUP = "manipulator";
static const std::string EE_LINK = "iiwa_link_ee";
/* RRTConnectkConfigDefault, RRTkConfigDefault, RRTstartkConfigDefault, TRRTkConfigDefault, ESTkConfigDefault
   SBLkConfigDefault, LBKPIECEkConfigDefault, BKPIECEkConfigDefault, PRMkConfigDefault, PRMstarkConfigDefault */
static const std::string PLANNER_ID = "TRRTkConfigDefault";  
// static const std::string REFERENCE_FRAME = "arm_mount_link";
static const std::string REFERENCE_FRAME = "world";

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
    
    // iiwa_ros::iiwa_ros my_iiwa;
    // my_iiwa.init();

    std::string movegroup_name, ee_link, planner_id, reference_frame;
    geometry_msgs::PoseStamped current_cartesian_position, target_cartesian_position;
    std::vector<geometry_msgs::Pose> waypoints;

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
    move_group.setPlannerId(PLANNER_ID);  
    move_group.setEndEffectorLink(ee_link);
    move_group.setPoseReferenceFrame(reference_frame);
    //ROS_INFO("Planner ID: %s", move_group.getPlannerId().c_str());
    ROS_INFO("Planning frame: %s", move_group.getPlanningFrame().c_str());
    ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());
    //ROS_INFO("Pose Reference Frame: %s", move_group.getPoseReferenceFrame().c_str());
    
    // Moveit Visualization Tool 
    moveit_visual_tools::MoveItVisualTools visual_tools("iiwa_link_0");
    if (sim == true) { 
        visual_tools.deleteAllMarkers();
        visual_tools.trigger();
    }

    //Vector to scale 3D file units (to convert from mm to meters for example)
    Vector3d vectorScale(0.05, 0.05, 0.05);
    // Define a collision object ROS message.
    moveit_msgs::CollisionObject co;
    co.header.frame_id = move_group.getPlanningFrame();
    // The id of the object is used to identify it.
    co.id = "obstacle";

    //Path where the .dae or .stl object is located
    shapes::Mesh* m = shapes::createMeshFromResource("package://iiwa_gazebo/worlds/meshes/obstacle.stl", vectorScale); 
    ROS_INFO("Your mesh was loaded");

    shape_msgs::Mesh mesh;
    shapes::ShapeMsg mesh_msg;  
    shapes::constructMsgFromShape(m, mesh_msg);
    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

    // // TXT file with list of coordinates
    // ifstream txt(ros::package::getPath("iiwa_examples")+TXT_FILE);
    // // check if text file is well opened
    // if(!txt.is_open()){
    //     cout << "FILE NOT FOUND" << endl;
    //     return 1;
    // }

    // string line;
    // bool init = false;
    // double x, y, z, fraction;
    MoveItErrorCode success_plan = MoveItErrorCode::FAILURE, motion_done = MoveItErrorCode::FAILURE;
    
    // // initialization before start drawing
    while (ros::ok()){
        ros::Duration(5).sleep(); // wait for 2 sec
        ROS_INFO("Sleeping 5 seconds before starting ... ");

        // set all the joint values to the init joint position
        move_group.setStartStateToCurrentState();
        move_group.setJointValueTarget("iiwa_joint_1", 0.0);
        move_group.setJointValueTarget("iiwa_joint_2", 0.698132 );
        move_group.setJointValueTarget("iiwa_joint_3", 0.0);
        move_group.setJointValueTarget("iiwa_joint_4", -1.65806 );
        move_group.setJointValueTarget("iiwa_joint_5", 0.0);
        move_group.setJointValueTarget("iiwa_joint_6", 0.785398);
        move_group.setJointValueTarget("iiwa_joint_7", 0.0);
        success_plan = move_group.plan(my_plan);
        if (success_plan == MoveItErrorCode::SUCCESS) {
            motion_done = move_group.execute(my_plan);
            
        }
        ROS_INFO("Moved to the initial position");
        current_cartesian_position = move_group.getCurrentPose("iiwa_link_ee");   
        current_cartesian_position.pose.position.z -= 0.02;

        //Define a pose for your mesh (specified relative to frame_id)
        geometry_msgs::Pose obj_pose;
        obj_pose.position = current_cartesian_position.pose.position;

        // Add the mesh to the Collision object message 
        co.meshes.push_back(mesh);
        co.mesh_poses.push_back(obj_pose);
        co.operation = moveit_msgs::CollisionObject::ADD;
        
        // publish the collision object        
        std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.push_back(co);
        planning_scene_interface.addCollisionObjects(collision_objects);
        ros::Duration(2).sleep(); // wait for 2 sec
        ROS_INFO("Sleeping 2 seconds before starting ... ");
        // ROS_WARN_STREAM(current_cartesian_position);

        move_group.setStartStateToCurrentState();
        target_cartesian_position = current_cartesian_position;
        target_cartesian_position.pose.position.x -= 0.02;
        target_cartesian_position.pose.position.z += 0.08;
        target_cartesian_position.pose.orientation.x = 0;
        target_cartesian_position.pose.orientation.y = 0.7071068;
        target_cartesian_position.pose.orientation.z = 0;
        target_cartesian_position.pose.orientation.w = 0.7071068;
        move_group.setPoseTarget(target_cartesian_position);
        visual_tools.publishAxis(target_cartesian_position.pose, 0.1);
        move_group.setPlanningTime(1000.0);
        success_plan = move_group.plan(my_plan);
        if (success_plan == MoveItErrorCode::SUCCESS) {
            motion_done = move_group.execute(my_plan);    
        }
        ROS_INFO("Path Planned");

        // ROS_INFO("Sleeping 3 seconds before starting ... ");
        // ros::Duration(3).sleep(); // wait for 3 sec
        
    //     init = true;
    //     current_cartesian_position = move_group.getCurrentPose(ee_link);   
    //     command_cartesian_position = current_cartesian_position;
    //     drawing_point = current_cartesian_position.pose;

    //     x = current_cartesian_position.pose.position.x - 0.045; // default x position

    //     // linear_path.push_back(current_cartesian_position.pose);
    }

    cerr<<"Stopping spinner..."<<endl;
    spinner.stop();

    cerr<<"Bye!"<<endl;

    return 0;
}