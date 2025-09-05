cmake_minimum_required(VERSION 3.0.2)
project(mbot_control)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        controller_manager
        joint_state_controller
        robot_state_publisher
)

catkin_package(
        LIBRARIES mbot_control
        CATKIN_DEPENDS roscpp rospy std_msgs
#  LIBRARIES mbot_control.launch
#  CATKIN_DEPENDS controller_manager joint_state_controller robot_state_publisher
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable( mbot_controller
        src/mbot_controller.cpp
        )

target_link_libraries( mbot_controller
        ${catkin_LIBRARIES})