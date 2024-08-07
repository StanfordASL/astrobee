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

#include <localization_common/logger.h>
#include <localization_common/pose_with_covariance_interpolater.h>
#include <localization_common/test_utilities.h>
#include <localization_common/utilities.h>

#include <gtest/gtest.h>

namespace lc = localization_common;

TEST(PoseWithCovarianceInterpolaterTester, Interpolate) {
  std::vector<lc::PoseWithCovariance> poses;
  std::vector<lc::Time> timestamps;
  for (int i = 0; i < 10; ++i) {
    poses.emplace_back(lc::PoseWithCovariance(lc::RandomIsometry3d(), lc::PoseCovariance::Identity()));
    timestamps.emplace_back(i);
  }
  lc::PoseWithCovarianceInterpolater interpolater(timestamps, poses);

  // Too low
  {
    const auto interpolated_pose = interpolater.Interpolate(-1);
    EXPECT_TRUE(interpolated_pose == boost::none);
  }
  // Too high
  {
    const auto interpolated_pose = interpolater.Interpolate(11);
    EXPECT_TRUE(interpolated_pose == boost::none);
  }
  // Valid
  {
    const auto interpolated_pose = interpolater.Interpolate(3.3);
    ASSERT_TRUE(interpolated_pose != boost::none);
    const auto expected_interpolated_pose = lc::Interpolate(poses[3], poses[4], 0.3);
    EXPECT_MATRIX_NEAR(interpolated_pose->pose, expected_interpolated_pose.pose, 1e-6);
  }
  // Exact pose/timestamp
  {
    const auto interpolated_pose = interpolater.Interpolate(8);
    ASSERT_TRUE(interpolated_pose != boost::none);
    EXPECT_MATRIX_NEAR(interpolated_pose->pose, poses[8].pose, 1e-6);
  }
}

TEST(PoseWithCovarianceInterpolaterTester, Relative) {
  std::vector<lc::PoseWithCovariance> poses;
  std::vector<lc::Time> timestamps;
  for (int i = 0; i < 10; ++i) {
    poses.emplace_back(lc::PoseWithCovariance(lc::RandomIsometry3d(), lc::PoseCovariance::Identity()));
    timestamps.emplace_back(i);
  }
  lc::PoseWithCovarianceInterpolater interpolater(timestamps, poses);

  // Too low
  {
    const auto relative_pose = interpolater.Relative(-1, 5);
    EXPECT_TRUE(relative_pose == boost::none);
  }
  // Too high
  {
    const auto relative_pose = interpolater.Relative(5, 11);
    EXPECT_TRUE(relative_pose == boost::none);
  }
  // Valid
  {
    const auto relative_pose = interpolater.Relative(2.2, 3.3);
    ASSERT_TRUE(relative_pose != boost::none);
    const auto expected_relative_pose =
      lc::Interpolate(poses[2], poses[3], 0.2).pose.inverse() * lc::Interpolate(poses[3], poses[4], 0.3).pose;
    EXPECT_MATRIX_NEAR(relative_pose->pose, expected_relative_pose, 1e-6);
  }
  // Exact pose/timestamp
  {
    const auto relative_pose = interpolater.Relative(7, 8);
    ASSERT_TRUE(relative_pose != boost::none);
    EXPECT_MATRIX_NEAR(relative_pose->pose, poses[7].pose.inverse() * poses[8].pose, 1e-6);
  }
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
