#include "iiwa_ros/iiwa_ros.hpp"
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// OMPL
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/MechanicalWorkOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/tools/multiplan/OptimizePlan.h>
#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include <omplapp/config.h>
#include <iostream>
#include <fstream>
#include <Eigen/Geometry> 

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <visualization_msgs/Marker.h>
#include <ros/package.h>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <iiwa_ros/state/cartesian_pose.hpp>
#include <iiwa_ros/state/joint_position.hpp>
#include <iiwa_ros/command/cartesian_pose.hpp>
#include <iiwa_ros/command/joint_position.hpp>

#define FILE_NAME "/home/daeun/ros_ws/contact/src/iiwa_examples/config/contact_solution_path.txt"
#define FILE_NAME_ "/home/daeun/ros_ws/contact/src/iiwa_examples/config/contact_solution_path_save.txt"
#define ROBOT_FNAME "/home/daeun/ros_ws/contact/src/iiwa_stack/iiwa_description/meshes/tool/visual/robot.stl"
#define ENV_FNAME "/home/daeun/ros_ws/contact/src/iiwa_stack/iiwa_gazebo/worlds/meshes/obstacle.stl"

using namespace ompl;
using namespace std;
using namespace Eigen;
using moveit::planning_interface::MoveItErrorCode;

// Create MoveGroup
static const std::string PLANNING_GROUP = "manipulator";
static const std::string EE_LINK = "tool_link_ee";
/* RRTConnectkConfigDefault, RRTkConfigDefault, RRTstartkConfigDefault, TRRTkConfigDefault, ESTkConfigDefault
   SBLkConfigDefault, LBKPIECEkConfigDefault, BKPIECEkConfigDefault, PRMkConfigDefault, PRMstarkConfigDefault */
static const std::string PLANNER_ID = "TRRTkConfigDefault";  
// static const std::string REFERENCE_FRAME = "arm_mount_link";
static const std::string REFERENCE_FRAME = "world";

bool sim;

std::vector<std::string> split(std::string input, char delimiter){
    std::vector<std::string> ans;
    std::stringstream str(input);
    std::string temp;
    
    while(getline(str, temp, delimiter)){
        ans.push_back(temp);
    }
    
    return ans;
}

/* This function solves the problem using mesh models and saves the path into txt file */
void solve(){
    // plan in SE3
    app::SE3RigidBodyPlanning setup;

    // load the robot and the environment
    std::string robot_fname = ROBOT_FNAME;
    std::string env_fname = ENV_FNAME;
    setup.setRobotMesh(robot_fname);
    setup.setEnvironmentMesh(env_fname);

    // Bounds for Apartment environment
    base::RealVectorBounds bounds(3);
    bounds.low[0] = -0.83;
    bounds.low[1] = -0.84;
    bounds.low[2] = -1.00;
    bounds.high[0] = 0.84;
    bounds.high[1] = 0.84;
    bounds.high[2] = 0.55;

    // define start state
    base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
    start->setX(0.31);
    start->setY(0);
    start->setZ(0.55);
    start->rotation().setIdentity();

    // define goal state
    base::ScopedState<base::SE3StateSpace> goal(start);
    goal->setX(0.31);
    goal->setY(0);
    goal->setZ(-1.2);
    goal->rotation().x= 0;
    goal->rotation().y= -0.7071067811865475;
    goal->rotation().z= 0;
    goal->rotation().w= 0.7071067811865476;

    // set the start & goal states
    setup.setStartAndGoalStates(start, goal);

    // Bound the state space
    setup.getSpaceInformation()->getStateSpace()->as<base::SE3StateSpace>()->setBounds(bounds);

    // setting collision checking resolution to 1% of the space extent
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.001);

    // use RRTConnect for planning
    setup.setPlanner(std::make_shared<geometric::TRRT>(setup.getSpaceInformation()));

    // make sure the planners run until the time limit, and get the best possible solution
    base::OptimizationObjectivePtr obj;
    obj = std::make_shared<base::MechanicalWorkOptimizationObjective>(setup.getSpaceInformation());
    obj->setCostThreshold(base::Cost(1000.00));
    setup.getProblemDefinition()->setOptimizationObjective(obj);

    // we call setup just so print() can show more information
    setup.setup();
    // setup.print();

    // try to solve the problem
    if (setup.solve(10))
    {
        // simplify & print the solution
        setup.simplifySolution();
        std::ofstream fout;  
        fout.open(FILE_NAME); // save path as a file
        setup.getSolutionPath().printAsMatrix(fout);
        fout.close();
    }
}

std::vector<geometry_msgs::Pose> transform_path(geometry_msgs::Pose init_pose_){
    /*
    WARNING!
    EIGEN QUATERNION is in order: w, x, y, z
    */

    std::vector<geometry_msgs::Pose> transformed_path;

    // read a saved path
    std::ifstream f (FILE_NAME); 
    if(!f.is_open()){
        std::cout << "FILE NOT FOUND" << std::endl;
    }

    std::string line;
    // Eigen::Vector3d translate(0.65345, 0.000159969, 0.27459);
    // Eigen::Quaterniond rotate(0, 0, 1, 0);
    Eigen::Vector3d translate(init_pose_.position.x, init_pose_.position.y, init_pose_.position.z);
    Eigen::Quaterniond rotate(init_pose_.orientation.w, init_pose_.orientation.x, init_pose_.orientation.y, init_pose_.orientation.z);

    // std::vector<Eigen::Vector3d> positions;
    // std::vector<Eigen::Quaterniond> orientations;

    getline(f, line);
    std::vector<std::string> init_pose = split(line, ' ');
    Eigen::Vector3d init_position(std::stod(init_pose[0]),std::stod(init_pose[1]),std::stod(init_pose[2])); 
    Eigen::Quaterniond init_orientation(std::stod(init_pose[6]),std::stod(init_pose[3]),std::stod(init_pose[4]),std::stod(init_pose[5]));
    // remove the starting position
    // positions.push_back(init_position-init_position+translate);
    // orientations.push_back(init_orientation*rotate);

    while (getline(f, line)){
        if (line.empty()) break;
        // get one pose
        std::vector<std::string> pose = split(line, ' ');
        Eigen::Vector3d position(std::stod(pose[0]),std::stod(pose[1]),std::stod(pose[2])); 
        Eigen::Quaterniond orientation(std::stod(pose[6]),std::stod(pose[3]),std::stod(pose[4]),std::stod(pose[5]));

        // transform
        position -= init_position;
        position *= 0.05;
        position = rotate * position;
        position += translate;
        orientation *= rotate;

        geometry_msgs::Pose waypoint;
        waypoint.position.x = position.x();
        waypoint.position.y = position.y();
        waypoint.position.z = position.z();
        waypoint.orientation.x = orientation.x();
        waypoint.orientation.y = orientation.y();
        waypoint.orientation.z = orientation.z();
        waypoint.orientation.w = orientation.w();

        transformed_path.push_back(waypoint);
    }

    // std::ofstream fout(FILE_NAME_); 
    // for (int i = 0 ; i < positions.size(); i++){
    //     fout << positions[i].x() << " " << positions[i].y() << " " << positions[i].z() << " ";
    //     fout << orientations[i].x()  << " " << orientations[i].y() << " " << orientations[i].z() << " " << orientations[i].w() << "\n";
    // }

    // fout.close();
    return transformed_path;
}

int main (int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "CommandRobotMoveit");
    ros::NodeHandle nh("~");
    nh.param("sim", sim, false);

    iiwa_ros::state::CartesianPose iiwa_pose_state;
    iiwa_ros::state::JointPosition iiwa_joint_state;
    iiwa_ros::command::CartesianPose iiwa_pose_command;
    iiwa_ros::command::JointPosition iiwa_joint_command;
    iiwa_ros::service::TimeToDestinationService iiwa_time_destination;

    iiwa_pose_state.init("iiwa");
    iiwa_pose_command.init("iiwa");
    iiwa_joint_state.init("iiwa");
    iiwa_joint_command.init("iiwa");
    iiwa_time_destination.init("iiwa");

    // ROS spinner.
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    // iiwa_ros::iiwa_ros my_iiwa;
    // my_iiwa.init();

    // solve the problem
    solve();

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
    const double jump_threshold = 0.01; // 0.0
    const double eef_step = 0.001; // 0.001
    const std::vector<double> tolerance_pose(3, 0.00001);
    const std::vector<double> tolerance_angle(3, 0.00001);
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

    // .. _RobotModelLoader:
    //     http://docs.ros.org/melodic/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    // Using the :moveit_core:`RobotModel`, we can construct a
    // :moveit_core:`RobotState` that maintains the configuration
    // of the robot. We will set all joints in the state to their
    // default values. We can then get a
    // :moveit_core:`JointModelGroup`, which represents the robot
    // model for a particular group, e.g. the "panda_arm" of the Panda
    // robot.
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    // const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");

    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    // Moveit Visualization Tool 
    moveit_visual_tools::MoveItVisualTools visual_tools("iiwa_link_0");
    if (sim == true) { 
        visual_tools.deleteAllMarkers();
        visual_tools.trigger();
    }

    //Vector to scale 3D file units 
    Vector3d vectorScale(0.05, 0.05, 0.05);
    // Define a collision object ROS message.
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();
    // The id of the object is used to identify it.
    collision_object.id = "obstacle";

    //Path where the .dae or .stl object is located
    shapes::Mesh* m = shapes::createMeshFromResource("package://iiwa_gazebo/worlds/meshes/obstacle.stl", vectorScale); 
    ROS_INFO("Your mesh was loaded");

    shape_msgs::Mesh mesh;
    shapes::ShapeMsg mesh_msg;  
    shapes::constructMsgFromShape(m, mesh_msg);
    mesh = boost::get<shape_msgs::Mesh>(mesh_msg);

    MoveItErrorCode success_plan = MoveItErrorCode::FAILURE, motion_done = MoveItErrorCode::FAILURE;
    double fraction;
    
    while (ros::ok()){
        ros::Duration(5).sleep(); // wait for 2 sec
        ROS_INFO("Sleeping 5 seconds before starting ... ");

        iiwa_msgs::JointPosition command_joint_position;
        iiwa_msgs::CartesianPose command_cartesian_position;

        command_joint_position = iiwa_joint_state.getPosition();
        command_joint_position.position.a1 = 0.0;
        command_joint_position.position.a2 = 0.872665;
        command_joint_position.position.a3 = 0.0;
        command_joint_position.position.a4 = -1.13446;
        command_joint_position.position.a5 = 0.0;
        command_joint_position.position.a6 = 1.13446;
        command_joint_position.position.a7 = 0.0;
        iiwa_joint_command.setPosition(command_joint_position);


        ros::Duration(5).sleep(); // wait for 5 sec
        command_cartesian_position = iiwa_pose_state.getPose();
        ROS_INFO("Sleeping 5 seconds before starting ... ");

        std::vector<geometry_msgs::Pose> linear_path;
        linear_path = transform_path(command_cartesian_position.poseStamped.pose);

        for (int i = 0; i < linear_path.size(); i++)
        {
            command_cartesian_position.poseStamped.pose= linear_path[i];
            iiwa_pose_command.setPose(command_cartesian_position.poseStamped);
            ros::Duration(0.5).sleep(); // wait for 5 sec
        }


        // MOTION PLANNING
        // fraction = 0.0;
        // while (fraction < 0.8){
        //     fraction = move_group.computeCartesianPath(linear_path, eef_step, jump_threshold, trajectory, true);
        //     my_plan.trajectory_ = trajectory;
            
        //     if (fraction < 0.8) ROS_WARN_STREAM("PATH ERROR");
        //     else {
        //         move_group.execute(my_plan);  // execute plan
        //         linear_path.clear();
        //     }
        // }

        // ROS_WARN_STREAM("START POSE:");
        // ROS_WARN_STREAM(current_cartesian_position.pose);

        // ROS_WARN_STREAM("GOAL POSE:");
        // ROS_WARN_STREAM(target_cartesian_position.pose);

        // collision_objects.clear();
        // collision_object.operation = collision_object.REMOVE;
        // collision_objects.push_back(collision_object);
        // planning_scene_interface.applyCollisionObjects(collision_objects);

        // start planning
        // move_group.setPlanningTime(6000.0);
        // success_plan = move_group.plan(my_plan);
        // if (success_plan == MoveItErrorCode::SUCCESS) {
        //     motion_done = move_group.execute(my_plan);    
        //     ROS_INFO("Path Planned");
        // }
        // else {
        //     ROS_ERROR("Path NOT Planned");
        // }

        // ros::Duration(2).sleep();
        // break;
        // collision_objects.clear();
        // collision_object.operation = collision_object.ADD;
        // collision_objects.push_back(collision_object);
        // planning_scene_interface.applyCollisionObjects(collision_objects);
        break;

    }
    

    cerr<<"Stopping spinner..."<<endl;
    spinner.stop();

    cerr<<"Bye!"<<endl;

    return 0;
}