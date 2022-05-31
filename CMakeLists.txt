cmake_minimum_required(VERSION 3.0.2)
project(mpc_local_planner)

set(ACADOS_LIB $ENV{ACADOS_SOURCE_DIR})

add_compile_options(-std=c++14)

# Uncomment if you want debugging
set(CMAKE_BUILD_TYPE Debug)

# Default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

add_compile_definitions(__PATH__="${PROJECT_SOURCE_DIR}")

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
   CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs tf2 tf2_ros
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ACADOS_LIB}/include/acados
  ${ACADOS_LIB}/include/blasfeo/include
  ${ACADOS_LIB}/include/hpipm/include
  ${ACADOS_LIB}/include
  ${PROJECT_SOURCE_DIR}/acados/path_tracking/c_generated_code
  ${PROJECT_SOURCE_DIR}/acados/point_stab/c_generated_code
  /usr/local/include
)

link_directories(
  /usr/local/lib
  ${ACADOS_LIB}/lib
)

add_custom_command(
  OUTPUT ${PROJECT_SOURCE_DIR}/acados/path_tracking/c_generated_code/main_path_tracking.c
  COMMAND ${ACADOS_LIB}/build/env/bin/python3 mpcPathTracking.py
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/acados/path_tracking
  DEPENDS acados/path_tracking/mpcPathTracking.py params/mpc.yaml
)

add_custom_target(
  path_tracking_lib
  COMMAND $(MAKE) shared_lib
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/acados/path_tracking/c_generated_code/
  DEPENDS ${PROJECT_SOURCE_DIR}/acados/path_tracking/c_generated_code/main_path_tracking.c
)

add_library( path_tracking_mpc SHARED IMPORTED )
set_target_properties( path_tracking_mpc PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/acados/path_tracking/c_generated_code/libacados_ocp_solver_path_tracking.so )

########
add_custom_command(
  OUTPUT ${PROJECT_SOURCE_DIR}/acados/point_stab/c_generated_code/main_point_stab.c
  COMMAND ${ACADOS_LIB}/build/env/bin/python3 mpcPointStab.py
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/acados/point_stab
  DEPENDS acados/point_stab/mpcPointStab.py params/mpc.yaml
)

add_custom_target(
  point_stab_lib
  COMMAND $(MAKE) shared_lib
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/acados/point_stab/c_generated_code/
  DEPENDS ${PROJECT_SOURCE_DIR}/acados/point_stab/c_generated_code/main_point_stab.c
)

add_library( point_stab_mpc SHARED IMPORTED )
set_target_properties( point_stab_mpc PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/acados/point_stab/c_generated_code/libacados_ocp_solver_point_stab.so )

FILE(GLOB SOURCES "src/*.cpp")

add_executable(ff_mpc ${SOURCES})
target_link_libraries(ff_mpc path_tracking_mpc point_stab_mpc acados hpipm blasfeo m ${catkin_LIBRARIES})
add_dependencies(ff_mpc path_tracking_lib point_stab_lib)

include_directories(
  tests
)
# add_executable(testMpc tests/main.cpp src/bounds.cpp src/utilities.cpp src/MPC.cpp src/solverCppAD.cpp src/AcadosSolver.cpp)
# target_link_libraries(testMpc ipopt myacadoslib acados hpipm blasfeo m ${catkin_LIBRARIES})
# add_dependencies(testMpc acados_generated_lib) 