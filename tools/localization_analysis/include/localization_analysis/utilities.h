/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */
#ifndef LOCALIZATION_ANALYSIS_UTILITIES_H_
#define LOCALIZATION_ANALYSIS_UTILITIES_H_

#include <camera/camera_params.h>
#include <graph_localizer/graph_localizer.h>
#include <localization_common/utilities.h>
#include <localization_measurements/timestamped_pose.h>
#include <vision_common/feature_tracker.h>

#include <opencv2/core/mat.hpp>

#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>

#include <string>
#include <vector>

namespace localization_analysis {
bool string_ends_with(const std::string& str, const std::string& ending);

template <typename MsgType>
void SaveMsg(const MsgType& msg, const std::string& topic, rosbag::Bag& bag) {
  const ros::Time timestamp = localization_common::RosTimeFromHeader(msg.header);
  bag.write("/" + topic, timestamp, msg);
}

geometry_msgs::PoseStamped PoseMsg(const Eigen::Isometry3d& global_T_body, const std_msgs::Header& header);

geometry_msgs::PoseStamped PoseMsg(const Eigen::Isometry3d& global_T_body, const localization_common::Time time);

geometry_msgs::PoseStamped PoseMsg(const gtsam::Pose3& global_T_body, const localization_common::Time time);

geometry_msgs::PoseStamped PoseMsg(const localization_measurements::TimestampedPose& timestamped_pose);
}  // namespace localization_analysis

#endif  // LOCALIZATION_ANALYSIS_UTILITIES_H_
