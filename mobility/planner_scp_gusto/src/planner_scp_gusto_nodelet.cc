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
#include <planner_scp_gusto/types.h>

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
    const std::vector<geometry_msgs::PoseStamped> &states = goal.states;
    ROS_ERROR_STREAM("SB Got states from plancallback");

    scp::Vec13 x0, xg;
    x0.segment(0, 3) << states.front().pose.position.x, states.front().pose.position.y,
      states.front().pose.position.z;
    x0.segment(3, 3) << 0, 0, 0;
    x0.segment(6, 4) << states.front().pose.orientation.x, states.front().pose.orientation.y,
      states.front().pose.orientation.z, states.front().pose.orientation.w;
    x0.segment(10, 3) << 0, 0, 0;

    xg.segment(0, 3) << states.back().pose.position.x, states.back().pose.position.y,
      states.back().pose.position.z;
    xg.segment(3, 3) << 0, 0, 0;
    xg.segment(6, 4) << states.back().pose.orientation.x, states.back().pose.orientation.y,
      states.back().pose.orientation.z, states.back().pose.orientation.w;
    xg.segment(10, 3) << 0, 0, 0;

    // Temporarily make a fake setpoint to stay in same position and add to plan_result
    geometry_msgs::PoseStamped state = states.front();
    ff_util::Setpoint sp;
    sp.when = state.header.stamp;
    sp.pose.position = state.pose.position;
    sp.pose.orientation = state.pose.orientation;
    geometry_msgs::Vector3 zeros;
    zeros.x = 0.0; zeros.y = 0.0; zeros.z = 0.0;
    sp.twist.linear = zeros;
    sp.twist.angular = zeros;
    sp.accel.linear = zeros;
    sp.accel.angular = zeros;
    plan_result.segment.push_back(sp);

    ROS_WARN_STREAM("The segment added was "<< state.pose.position.x);

    plan_result.response = RESPONSE::SUCCESS;

    // Special case: we might already be there
    if (plan_result.segment.size() < 2)
      plan_result.response = RESPONSE::ALREADY_THERE;
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
