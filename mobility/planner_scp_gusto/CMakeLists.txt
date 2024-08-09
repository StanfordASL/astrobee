# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
# 
# All rights reserved.
# 
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

cmake_minimum_required(VERSION 3.0)
project(planner_scp_gusto)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin2 REQUIRED COMPONENTS
  roscpp
  nodelet
  pluginlib
  actionlib
  choreographer
  tf2_geometry_msgs
  ff_util
  ff_msgs
  mapper
)

# OSQP-eigen
find_package(OsqpEigen REQUIRED)

## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED COMPONENTS common)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    planner_scp_gusto
  CATKIN_DEPENDS
    roscpp
    nodelet
    pluginlib
    actionlib
    choreographer
    tf2_geometry_msgs
    ff_util
    ff_msgs
    mapper
)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  /usr/local/include/osqp
  /usr/local/include/OsqpEigen
  # ${OsqpEigen_INCLUDE_DIRS}
  # ${osqp_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(planner_scp_gusto
  src/planner_scp_gusto.cc
  src/planner_scp_gusto_nodelet.cc
  src/optim.cc
)
add_dependencies(planner_scp_gusto ${catkin_EXPORTED_TARGETS})
target_link_libraries(planner_scp_gusto ${catkin_LIBRARIES})

target_link_libraries(planner_scp_gusto OsqpEigen::OsqpEigen)

#############
## Install ##
#############

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Mark nodelet_plugin for installation
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
