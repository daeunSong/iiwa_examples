cmake_minimum_required(VERSION 3.5)
project(iiwa_examples)

# set the default build type
if (NOT CMAKE_BUILD_TYPE)
    # By default, use Release mode
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Type of build" FORCE)

    # On 32bit architectures, use RelWithDebInfo
    if (CMAKE_COMPILER_IS_GNUCC AND CMAKE_SIZEOF_VOID_P EQUAL 4)
        set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Type of build" FORCE)
    endif()
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  iiwa_ros
  moveit_core 
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
)

catkin_package()

#find_package(ompl)

set(OMPL_INCLUDE_DIRS /usr/local/include/ompl-1.5)
set(OMPL_LIBRARIES /home/daeun/devel/omplapp-1.5.2-Source/build/Release/lib/libompl.so)
set(OMPLAPP_LIBRARIES /home/daeun/devel/omplapp-1.5.2-Source/build/Release/lib/libompl_app.so /home/daeun/devel/omplapp-1.5.2-Source/build/Release/lib/libompl_app_base.so)

#message("ompl libraries: ${OMPL_LIBRARIES}")
#message("ompl incldue: ${OMPL_INCLUDE_DIRS}")
#message("omplapp libraries: ${OMPLAPP_LIBRARIES}")

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ${OMPL_INCLUDE_DIRS})

add_executable(drawing_moveit src/drawing_moveit.cpp)
add_dependencies(drawing_moveit iiwa_msgs_generate_messages_cpp)
target_link_libraries(drawing_moveit ${catkin_LIBRARIES})

add_executable(updown_moveit src/updown_moveit.cpp)
add_dependencies(updown_moveit iiwa_msgs_generate_messages_cpp)
target_link_libraries(updown_moveit ${catkin_LIBRARIES})

add_executable(contact_moveit src/contact_moveit.cpp)
add_dependencies(contact_moveit iiwa_msgs_generate_messages_cpp)
target_link_libraries(contact_moveit ${catkin_LIBRARIES})

add_executable(ompl_planning src/ompl_planning.cpp)
add_dependencies(ompl_planning iiwa_msgs_generate_messages_cpp)
target_link_libraries(ompl_planning ${catkin_LIBRARIES} ${OMPL_LIBRARIES} ${OMPLAPP_LIBRARIES})
