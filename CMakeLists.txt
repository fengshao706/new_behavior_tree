cmake_minimum_required(VERSION 3.0.2)
project(new_behavior_tree)

## Use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore, enforcing cleaner code.
add_definitions(-Wall -Wno-enum-compare)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        rm_msgs
        behaviortree_cpp
        rm_common
        nav_msgs
        sensor_msgs
        move_base_msgs
        tf
        service_processor
        dynamic_reconfigure
        controller_manager
        mbf_msgs
        control_toolbox
        costmap_2d
        global_planner
        tf2_geometry_msgs
)


# Generate added messages and services with any dependencies listed here
#generate_messages(
#        DEPENDENCIES
#        std_msgs  # Or other packages containing msgs
#        actionlib_msgs
#)

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        CATKIN_DEPENDS
        roscpp
        rm_msgs
        behaviortree_cpp
        rm_common
        nav_msgs
        sensor_msgs
        move_base_msgs
        tf
        dynamic_reconfigure
        controller_manager
        costmap_2d
        service_processor
        DEPENDS
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

## Declare cpp executablesm
FILE(GLOB ALL_SOURCES "src/*.cpp")

add_executable(${PROJECT_NAME} include/behavior_tree/chassis_action_node.h
 include/behavior_tree/gimbal_action_node.h include/behavior_tree/manual_action_node.h  include/behavior_tree/shooter_action_node.h
 include/perception_layer.h include/register_node.h src/common/tools.cpp src/main.cpp
src/perception_layer.cpp src/register_node.cpp include/behavior_tree/common/action_node.h
        include/behavior_tree/common/action_node.cpp
        include/behavior_tree/common/action_node.h
        include/behavior_tree/condition_node.h
        include/behavior_tree/condition_node.cpp
        src/common/posture_manager.cpp

)


#add_dependencies(${PROJECT_NAME}
#        ${catkin_EXPORTED_TARGETS}
#)
## Specify libraries to link executable targets against
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
)

