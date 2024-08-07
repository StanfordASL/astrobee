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
#ifndef VISION_COMMON_FEATURE_TRACKER_PARAMS_H_
#define VISION_COMMON_FEATURE_TRACKER_PARAMS_H_

namespace vision_common {
struct FeatureTrackerParams {
  // Remove a feature track if not detected in the most recently provided set of feature points.
  bool remove_undetected_feature_tracks;
};
}  // namespace vision_common

#endif  // VISION_COMMON_FEATURE_TRACKER_PARAMS_H_
