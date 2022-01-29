cmake_minimum_required(VERSION 3.0.2)
project(mpc_local_planner)

# CHANGE TO YOUR PATH TO ACADOS
set(ACADOS_LIB "/home/torb/libraries/acados")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# Uncomment if you want debugging
set(CMAKE_BUILD_TYPE Debug)

# Default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

add_compile_definitions(__PATH__="${PROJECT_SOURCE_DIR}")
file(MAKE_DIRECTORY "${PROJECT_SOURCE_DIR}/logs")

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
  ${ACADOS_LIB}/include/acados
  ${ACADOS_LIB}/include/blasfeo/include
  ${ACADOS_LIB}/include/hpipm/include
  ${ACADOS_LIB}/include
  ${PROJECT_SOURCE_DIR}/acados/c_generated_code
  /usr/local/include
)

link_directories(
  /usr/local/lib
  ${ACADOS_LIB}/lib
)

add_custom_command(
  OUTPUT gen_acados_c_code
  COMMAND ${ACADOS_LIB}/build/env/bin/python3 main.py 
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/acados
  DEPENDS ${PROJECT_SOURCE_DIR}/acados/main.py
)
add_custom_target(
  acados_generated_lib
  COMMAND $(MAKE) shared_lib
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/acados/c_generated_code/
  DEPENDS gen_acados_c_code
)

add_library( myacadoslib SHARED IMPORTED )
set_target_properties( myacadoslib PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/acados/c_generated_code/libacados_ocp_solver_bicycle_model.so )

FILE(GLOB SOURCES "src/*.cpp")
add_executable(main ${SOURCES})
target_link_libraries(main ipopt myacadoslib acados hpipm blasfeo m ${catkin_LIBRARIES})
add_dependencies(main acados_generated_lib)

add_executable(testMpc tests/main.cpp src/bounds.cpp src/utilities.cpp src/MPC.cpp src/solverCppAD.cpp)
target_link_libraries(testMpc ipopt myacadoslib acados hpipm blasfeo m ${catkin_LIBRARIES})
add_dependencies(testMpc acados_generated_lib) 