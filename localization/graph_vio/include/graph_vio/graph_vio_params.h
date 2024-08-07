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

#ifndef GRAPH_VIO_GRAPH_VIO_PARAMS_H_
#define GRAPH_VIO_GRAPH_VIO_PARAMS_H_

#include <factor_adders/depth_odometry_factor_adder_params.h>
#include <factor_adders/standstill_factor_adder_params.h>
#include <factor_adders/vo_smart_projection_factor_adder_params.h>
#include <node_adders/combined_nav_state_node_adder.h>
#include <node_adders/combined_nav_state_node_adder_model_params.h>
#include <optimizers/nonlinear_optimizer.h>
#include <sliding_window_graph_optimizer/sliding_window_graph_optimizer_params.h>
#include <vision_common/standstill_params.h>

#include <boost/serialization/serialization.hpp>

namespace graph_vio {
struct GraphVIOParams {
  factor_adders::DepthOdometryFactorAdderParams depth_odometry_factor_adder;
  factor_adders::StandstillFactorAdderParams standstill_factor_adder;
  factor_adders::VoSmartProjectionFactorAdderParams vo_smart_projection_factor_adder;
  node_adders::CombinedNavStateNodeAdder::Params combined_nav_state_node_adder;
  node_adders::CombinedNavStateNodeAdderModelParams combined_nav_state_node_adder_model;
  optimizers::NonlinearOptimizerParams nonlinear_optimizer;
  sliding_window_graph_optimizer::SlidingWindowGraphOptimizerParams sliding_window_graph_optimizer;
  vision_common::StandstillParams standstill;

  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    ar& BOOST_SERIALIZATION_NVP(depth_odometry_factor_adder);
    ar& BOOST_SERIALIZATION_NVP(standstill_factor_adder);
    ar& BOOST_SERIALIZATION_NVP(vo_smart_projection_factor_adder);
    ar& BOOST_SERIALIZATION_NVP(combined_nav_state_node_adder);
    ar& BOOST_SERIALIZATION_NVP(nonlinear_optimizer);
    ar& BOOST_SERIALIZATION_NVP(sliding_window_graph_optimizer);
    ar& BOOST_SERIALIZATION_NVP(standstill);
  }
};
}  // namespace graph_vio

#endif  // GRAPH_VIO_GRAPH_VIO_PARAMS_H_
