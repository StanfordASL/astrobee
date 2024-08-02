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

// Standard includes
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

// FSW includes
#include <ff_util/ff_nodelet.h>
#include <ff_util/ff_flight.h>
#include <ff_common/ff_names.h>
#include <ff_util/config_server.h>
#include <ff_util/config_client.h>
#include <msg_conversions/msg_conversions.h>

// For the planner implementation API
#include <choreographer/planner.h>

// For the scp gusto planner implementation
#include <planner_scp_gusto/planner_scp_gusto.h>

// C++ includes
#include <vector>
#include <algorithm>
#include <cmath>

/**
 * \ingroup planner
 */
namespace planner_scp_gusto {

using RESPONSE = ff_msgs::PlanResult;

class PlannerSCPGustoNodelet : public planner::PlannerImplementation {
 public:
  PlannerSCPGustoNodelet() :
    planner::PlannerImplementation("scp_gusto", "SCP Gusto path planner") {}
  ~PlannerSCPGustoNodelet() {}

 protected:
  bool InitializePlanner(ros::NodeHandle *nh) {
    // Grab some configuration parameters for this node
    cfg_.Initialize(GetPrivateHandle(), "mobility/planner_scp_gusto.config");
    cfg_.Listen(boost::bind(
      &PlannerSCPGustoNodelet::ReconfigureCallback, this, _1));
    // Setup a timer to forward diagnostics
    timer_d_ = nh->createTimer(
      ros::Duration(ros::Rate(DEFAULT_DIAGNOSTICS_RATE)),
        &PlannerSCPGustoNodelet::DiagnosticsCallback, this, false, true);
    // Save the epsilon value
    epsilon_ = cfg_.Get<double>("epsilon");
      // Notify initialization complete
    NODELET_DEBUG_STREAM("Initialization complete");
    // Success
    return true;
  }

  void DiagnosticsCallback(const ros::TimerEvent &event) {
    SendDiagnostics(cfg_.Dump());
  }

  bool ReconfigureCallback(dynamic_reconfigure::Config &config) {
    if (!cfg_.Reconfigure(config))
      return false;
    epsilon_ = cfg_.Get<double>("epsilon");
    return true;
  }

  void PlanCallback(ff_msgs::PlanGoal const& goal) {
    // Do some basic error checks
    ff_msgs::PlanResult plan_result;
    plan_result.response = RESPONSE::SUCCESS;
    // if (goal.states.size() < 2)
    //   plan_result.response = RESPONSE::NOT_ENOUGH_STATES;
    // if (goal.check_obstacles)
    //   plan_result.response = RESPONSE::OBSTACLES_NOT_SUPPORTED;
    // if (plan_result.response < 0)
    //   return PlanResult(plan_result);
    // // Save the information
    // desired_vel_   = goal.desired_vel;
    // desired_omega_ = goal.desired_omega;
    // desired_accel_ = goal.desired_accel;
    // desired_alpha_ = goal.desired_alpha;
    // min_control_period_ = 1.0 / goal.desired_rate;
    // // Setup header and keep track of time as we generate trapezoidal ramps
    // plan_result.segment.clear();
    // // Generate the trapezoidal ramp
    // ros::Time offset = goal.states.front().header.stamp;
    // for (size_t i = 1; i < goal.states.size(); i++) {
    //   // Get the requested duration and offset
    //   double dt =
    //     (goal.states[i].header.stamp - goal.states[i-1].header.stamp).toSec();
    //   // Extract affine transformations for the two poses
    //   Eigen::Affine3d tf_1 =
    //     msg_conversions::ros_pose_to_eigen_transform(goal.states[i-1].pose);
    //   Eigen::Affine3d tf_4 =
    //     msg_conversions::ros_pose_to_eigen_transform(goal.states[i].pose);
    //   Eigen::Vector3d delta = tf_4.translation() - tf_1.translation();

    // }
    // // Special case: we might already be there
    // if (plan_result.segment.size() < 2)
    //   plan_result.response = RESPONSE::ALREADY_THERE;
    // Callback with the result
    return PlanResult(plan_result);
  }

  // Called to interrupt the process
  void CancelCallback() {}

 protected:
  ff_util::ConfigServer cfg_;
  ros::Timer timer_d_;
  double desired_vel_;
  double desired_omega_;
  double desired_accel_;
  double desired_alpha_;
  double min_control_period_;
  double epsilon_;
};

PLUGINLIB_EXPORT_CLASS(planner_scp_gusto::PlannerSCPGustoNodelet,
  nodelet::Nodelet);

}  // namespace planner_scp_gusto
