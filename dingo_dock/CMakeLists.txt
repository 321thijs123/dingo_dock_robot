cmake_minimum_required(VERSION 3.0.2)
project(dingo_dock)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  actionlib
  move_base_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dingo_dock
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cloudfilter src/cloudfilter.cpp)
target_link_libraries(cloudfilter ${catkin_LIBRARIES})

add_executable(nav_goal_sender src/nav_goal_sender.cpp)
target_link_libraries(nav_goal_sender ${catkin_LIBRARIES})

add_executable(drivetest src/drivetest.cpp)
target_link_libraries(drivetest ${catkin_LIBRARIES})

add_executable(controller src/controller.cpp)
target_link_libraries(controller ${catkin_LIBRARIES})
