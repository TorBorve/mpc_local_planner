cmake_minimum_required(VERSION 3.0.2)
project(mpc_local_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf2
  tf2_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mpc_local_planner
#  CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  tests
  ${catkin_INCLUDE_DIRS}
)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

# FILE(GLOB SOURCES "src/*.cpp")

add_executable(main src/main.cpp src/utilities.cpp src/MPC.cpp src/bounds.cpp)
target_link_libraries(main ${catkin_LIBRARIES} ipopt)

add_executable(testCode tests/main.cpp src/bounds.cpp)