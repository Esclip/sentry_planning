cmake_minimum_required(VERSION 3.0.2)
project(sentry_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  joint_state_controller
  robot_state_publisher
)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        controller_manager
        joint_state_controller
        robot_state_publisher
        )

catkin_package(
        LIBRARIES sentry_controller
        CATKIN_DEPENDS roscpp rospy std_msgs
        #  LIBRARIES mbot_control.launch
        #  CATKIN_DEPENDS controller_manager joint_state_controller robot_state_publisher
        #  DEPENDS system_lib
)


include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)

add_executable( sentry_controller
        src/sentry_controller.cpp
        )

target_link_libraries( sentry_controller
        ${catkin_LIBRARIES})