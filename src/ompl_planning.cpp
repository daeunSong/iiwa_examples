/*********************************************************************
 * Rice University Software Distribution License
 *
 * Copyright (c) 2010, Rice University
 * All Rights Reserved.
 *
 * For a full description see the file named LICENSE.
 *
 *********************************************************************/

/* Author: Ioan Sucan */

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/tools/multiplan/OptimizePlan.h>
#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include <omplapp/config.h>
#include <iostream>
#include <fstream>
#include <Eigen/Geometry> 
#include <vector>
#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <geometry_msgs/Pose.h>

#define FILE_NAME "/home/daeun/ros_ws/contact/src/iiwa_examples/config/contact_solution_path.txt"
#define FILE_NAME_ "/home/daeun/ros_ws/contact/src/iiwa_examples/config/contact_solution_path_.txt"
#define ROBOT_FNAME "/home/daeun/ros_ws/contact/src/iiwa_stack/iiwa_description/meshes/tool/visual/robot.stl"
#define ENV_FNAME "/home/daeun/ros_ws/contact/src/iiwa_stack/iiwa_gazebo/worlds/meshes/obstacle.stl"

using namespace ompl;


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
    obj = std::make_shared<base::PathLengthOptimizationObjective>(setup.getSpaceInformation());
    obj->setCostThreshold(base::Cost(100.00));
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

void transform_path(geometry_msgs::Pose init_pose_){
    /*
    WARNING!
    EIGEN QUATERNION is in order: w, x, y, z
    */

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

    std::vector<Eigen::Vector3d> positions;
    std::vector<Eigen::Quaterniond> orientations;

    getline(f, line);
    std::vector<std::string> init_pose = split(line, ' ');
    Eigen::Vector3d init_position(std::stod(init_pose[0]),std::stod(init_pose[1]),std::stod(init_pose[2])); 
    Eigen::Quaterniond init_orientation(std::stod(init_pose[6]),std::stod(init_pose[3]),std::stod(init_pose[4]),std::stod(init_pose[5]));
    positions.push_back(init_position-init_position+translate);
    orientations.push_back(init_orientation*rotate);

    while (getline(f, line)){
        if (line.empty()) break;
        // get one pose
        std::vector<std::string> pose = split(line, ' ');
        Eigen::Vector3d position(std::stod(pose[0]),std::stod(pose[1]),std::stod(pose[2])); 
        Eigen::Quaterniond orientation(std::stod(pose[6]),std::stod(pose[3]),std::stod(pose[4]),std::stod(pose[5]));

        // move
        position -= init_position;
        position *= 0.05;
        position = rotate * position;
        position += translate;
        orientation *= rotate;

        positions.push_back(position);
        orientations.push_back(orientation);

        // std::cout << position << std::endl;
        // std::cout << orientation << std::endl;
    }

    std::ofstream fout(FILE_NAME_); 
    for (int i = 0 ; i < positions.size(); i++){
        fout << positions[i].x() << " " << positions[i].y() << " " << positions[i].z() << " ";
        fout << orientations[i].x()  << " " << orientations[i].y() << " " << orientations[i].z() << " " << orientations[i].w() << "\n";
    }

    fout.close();
}

int main()
{
    geometry_msgs::Pose init_pose;
    init_pose.position.x = 0.65345;
    init_pose.position.y = 0.000159969;
    init_pose.position.z = 0.27459;
    init_pose.orientation.x = 0.0;
    init_pose.orientation.y = 1.0;
    init_pose.orientation.z = 0.0;
    init_pose.orientation.w = 0.0;

    solve();
    transform_path(init_pose);

    return 0;
}
