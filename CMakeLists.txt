cmake_minimum_required(VERSION 3.5)
project(mpc_local_planner)

# Enable option for specifying params file used in mpc.
option(MPC_PARAMS "mpc_params" OFF)

set(ACADOS_LIB $ENV{ACADOS_SOURCE_DIR})

# add_compile_options(-std=c++14)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Uncomment if you want debugging
# set(CMAKE_BUILD_TYPE Debug)

# Default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

add_compile_definitions(__PATH__="${PROJECT_SOURCE_DIR}")

# Check if MPC_PARAMS file is specified
if (NOT MPC_PARAMS)
  message(FATAL_ERROR "MPC_PARAMS option not set. Needed for determining what parameters the mpc should be compiled with. You can set it using cmake args. Example: catkin build --cmake-args -DMPC_PARMS=my_yaml_file.yaml")
endif()

# check that params file exists
if (NOT EXISTS ${PROJECT_SOURCE_DIR}/config/${MPC_PARAMS})
  message(FATAL_ERROR "The mpc params file does not exist. Specify a yaml file that exists. Path: ${PROJECT_SOURCE_DIR}/config/${MPC_PARAMS}")
endif()

# Copy params file to new file
configure_file(${PROJECT_SOURCE_DIR}/config/${MPC_PARAMS} ${PROJECT_SOURCE_DIR}/build/auto_gen.yaml COPYONLY)

# make command for generating the build folders for the acados code
add_custom_command(
  OUTPUT ${PROJECT_SOURCE_DIR}/build/path_tracking/
  COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_SOURCE_DIR}/build/path_tracking/
)
add_custom_command(
  OUTPUT ${PROJECT_SOURCE_DIR}/build/point_stab/
  COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_SOURCE_DIR}/build/point_stab/
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

# catkin_package(
# #  INCLUDE_DIRS include
# #  LIBRARIES mpc_local_planner
#    CATKIN_DEPENDS nav_msgs roscpp rospy std_msgs tf2 tf2_ros
# #  DEPENDS system_lib
# )

include_directories(
  include
  # ${catkin_INCLUDE_DIRS}
  ${ACADOS_LIB}/include/acados
  ${ACADOS_LIB}/include/blasfeo/include
  ${ACADOS_LIB}/include/hpipm/include
  ${ACADOS_LIB}/include
  ${PROJECT_SOURCE_DIR}/build/path_tracking/c_generated_code
  ${PROJECT_SOURCE_DIR}/build/point_stab/c_generated_code
  /usr/local/include
)

link_directories(
  /usr/local/lib
  ${ACADOS_LIB}/lib
)

add_custom_command(
  OUTPUT ${PROJECT_SOURCE_DIR}/build/path_tracking/c_generated_code/main_path_tracking.c
  COMMAND ${ACADOS_LIB}/build/env/bin/python3 ${PROJECT_SOURCE_DIR}/acados/mpcPathTracking.py
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/build/path_tracking
  DEPENDS acados/mpcPathTracking.py ${PROJECT_SOURCE_DIR}/build/auto_gen.yaml ${PROJECT_SOURCE_DIR}/build/path_tracking/
)

add_custom_target(
  path_tracking_lib
  COMMAND $(MAKE) shared_lib
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/build/path_tracking/c_generated_code/
  DEPENDS ${PROJECT_SOURCE_DIR}/build/path_tracking/c_generated_code/main_path_tracking.c 
)

add_library( path_tracking_mpc SHARED IMPORTED )
set_target_properties( path_tracking_mpc PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/build/path_tracking/c_generated_code/libacados_ocp_solver_path_tracking.so )

########
add_custom_command(
  OUTPUT ${PROJECT_SOURCE_DIR}/build/point_stab/c_generated_code/main_point_stab.c
  COMMAND ${ACADOS_LIB}/build/env/bin/python3 ${PROJECT_SOURCE_DIR}/acados/mpcPointStab.py
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/build/point_stab
  DEPENDS acados/mpcPointStab.py ${PROJECT_SOURCE_DIR}/build/auto_gen.yaml ${PROJECT_SOURCE_DIR}/build/point_stab/
)

add_custom_target(
  point_stab_lib
  COMMAND $(MAKE) shared_lib
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/build/point_stab/c_generated_code/
  DEPENDS ${PROJECT_SOURCE_DIR}/build/point_stab/c_generated_code/main_point_stab.c
)

add_library( point_stab_mpc SHARED IMPORTED )
set_target_properties( point_stab_mpc PROPERTIES IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/build/point_stab/c_generated_code/libacados_ocp_solver_point_stab.so )

FILE(GLOB SOURCES "src/*.cpp")

add_executable(ff_mpc ${SOURCES})
ament_target_dependencies(ff_mpc 
rclcpp
nav_msgs
std_msgs
tf2
tf2_ros
example_interfaces
)
target_link_libraries(ff_mpc path_tracking_mpc point_stab_mpc acados hpipm blasfeo m)
add_dependencies(ff_mpc path_tracking_lib point_stab_lib)

install(TARGETS
  ff_mpc
  DESTINATION lib/${PROJECT_NAME}  
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
DESTINATION share/${PROJECT_NAME})

install(DIRECTORY rviz
DESTINATION share/${PROJECT_NAME})

install(DIRECTORY build/
DESTINATION share/${PROJECT_NAME}
FILES_MATCHING PATTERN "*.yaml")

ament_package()