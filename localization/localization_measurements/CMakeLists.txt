#Copyright(c) 2017, United States Government, as represented by the
#Administrator of the National Aeronautics and Space Administration.
#
#All rights reserved.
#
#The Astrobee platform is licensed under the Apache License, Version 2.0
#(the "License"); you may not use this file except in compliance with the
#License.You may obtain a copy of the License at
#
#http:  // www.apache.org/licenses/LICENSE-2.0
#
#Unless required by applicable law or agreed to in writing, software
#distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.See the
#License for the specific language governing permissions and limitations
#under the License.

cmake_minimum_required(VERSION 3.0)
project(localization_measurements)

## Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin2 REQUIRED COMPONENTS
  config_reader
  cv_bridge
  ff_msgs
  localization_common
  msg_conversions
  point_cloud_common
  vision_common
)

find_package(PCL REQUIRED COMPONENTS common features filters octree search)

find_package(pcl_conversions REQUIRED)

# Find OpenCV
find_package(OpenCV 4.0 REQUIRED)

# System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

catkin_package(
  LIBRARIES ${PROJECT_NAME} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${pcl_conversions_INCLUDE_DIRS}  
  INCLUDE_DIRS include ${PCL_INCLUDE_DIRS} 
  CATKIN_DEPENDS config_reader cv_bridge ff_msgs localization_common msg_conversions point_cloud_common vision_common 
)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
)

# Declare C++ libraries
add_library(${PROJECT_NAME}
  src/depth_image.cc
  src/measurement_conversions.cc
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PCL_LIBRARIES})


if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_plane
    test/test_plane.test
    test/test_plane.cc
  )
  target_link_libraries(test_plane
    localization_measurements gtsam
  )
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_depth_image
    test/test_depth_image.test
    test/test_depth_image.cc
  )
  target_link_libraries(test_depth_image
    localization_measurements gtsam
  )
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_measurement_conversions
    test/test_measurement_conversions.test
    test/test_measurement_conversions.cc
  )
  target_link_libraries(test_measurement_conversions
    localization_measurements gtsam
  )

endif()

#############
## Install ##
#############

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
