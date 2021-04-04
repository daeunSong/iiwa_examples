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

add_executable(updown_moveit src/updown_moveit.cpp)
add_dependencies(updown_moveit iiwa_msgs_generate_messages_cpp)
target_link_libraries(updown_moveit ${catkin_LIBRARIES})

add_executable(contact_moveit src/contact_moveit.cpp)
add_dependencies(contact_moveit iiwa_msgs_generate_messages_cpp)
target_link_libraries(contact_moveit ${catkin_LIBRARIES})
