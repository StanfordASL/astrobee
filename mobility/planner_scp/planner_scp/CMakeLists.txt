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

project(planner_scp)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations  -Werror=return-type -fPIC")
find_package(DecompUtil REQUIRED)
find_package(catkin2 REQUIRED COMPONENTS roscpp roslib nodelet pluginlib actionlib tf)

set(LIBS
  ${SCP_LIBRARIES}
  # osqp::osqp
  # OsqpEigen::OsqpEigen
  # ${BULLET_LIBRARIES}
  ${DECOMP_UTIL_LIBRARIES}
  ${SPARSE_MAPPING_LIBRARIES}
)

set(INCLUDES
  ${SCP_INCLUDE_DIRS}
  # ${BULLET_INCLUDE_DIRS} 
  ${DECOMP_UTIL_INCLUDE_DIRS}
  ${SPARSE_MAPPING_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES
    planner_scp
  DEPENDS
  CATKIN_DEPENDS
    roscpp
    nodelet
    pluginlib
    actionlib
    tf
)

create_library(TARGET planner_scp
  LIBS ${LIBS} ${catkin_LIBRARIES} ff_nodelet msg_conversions config_server config_client jsonloader ff_flight mapper
  INC  ${catkin_INCLUDE_DIRS} ${INCLUDES}
  DEPS choreographer
)

create_tool_targets(DIR test 
  LIBS ${LIBS} 
  INC  ${INCLUDES}
)

install_launch_files()
