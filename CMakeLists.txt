cmake_minimum_required(VERSION 3.5)
project(iiwa_examples)

# set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  iiwa_ros
  moveit_core 
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(drawing_moveit src/drawing_moveit.cpp)
add_dependencies(drawing_moveit iiwa_msgs_generate_messages_cpp)
target_link_libraries(drawing_moveit ${catkin_LIBRARIES})

add_executable(drawing_spline src/drawing_iiwa.cpp)
add_dependencies(drawing_spline iiwa_msgs_generate_messages_cpp)
target_link_libraries(drawing_spline ${catkin_LIBRARIES})

add_executable(updown_moveit src/updown_moveit.cpp)
add_dependencies(updown_moveit iiwa_msgs_generate_messages_cpp)
target_link_libraries(updown_moveit ${catkin_LIBRARIES})

add_executable(init_pose src/init_pose.cpp)
add_dependencies(init_pose iiwa_msgs_generate_messages_cpp)
target_link_libraries(init_pose ${catkin_LIBRARIES})

add_executable(move_spline src/move_spline_demo.cpp)
add_dependencies(move_spline iiwa_msgs_generate_messages_cpp)
target_link_libraries(move_spline ${catkin_LIBRARIES})

