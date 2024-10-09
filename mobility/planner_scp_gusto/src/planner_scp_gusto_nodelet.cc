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
#include <planner_scp_gusto/optim.h>

// C++ includes
#include <vector>
#include <algorithm>
#include <cmath>

#define DEBUG true
#define OUTPUT_DEBUG NODELET_DEBUG_STREAM

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

 private:
  // planner configuration params
  bool faceforward_;      // Face-forward trajectory?
  bool check_obstacles_;  // Perform obstacle checking
  float desired_vel_;     // Soft limit on velocity
  float desired_accel_;   // Soft limit on accel
  float desired_omega_;   // Soft limit on omega
  float desired_alpha_;   // Soft limit on alpha
  float control_rate_;    // Control frequency
  double max_time_;       // Max generation time
  double epsilon_;
  bool use_2d;            // true for granite table
  std::string flight_mode_;
  ros::NodeHandle *nh_;

  uint N;
  scp::TOP* top;

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
    // Create a new optimization problem
    top = new scp::TOP(20., 801);
    top->Solve();
    // Save node handle
    nh_ = nh;
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

  void PlanCallback(ff_msgs::PlanGoal const &goal) override {
    ff_msgs::PlanResult plan_result;
    const std::vector<geometry_msgs::PoseStamped> &states = goal.states;

    if (!LoadActionParams(goal)) {
      ROS_ERROR_STREAM("Planner params are bad");
      ff_msgs::PlanResult plan_result;
      plan_result.response = RESPONSE::BAD_ARGUMENTS;
      PlanResult(plan_result);
      return;
    }

    std::cout << "SCP::PlanCallback start " << states.front().pose.position << std::endl;
    std::cout << "SCP::PlanCallback goal " << states.back().pose.position << std::endl;

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

    std::cout << "SCP:: states.front() orientation " << states.front().pose.orientation.x << " "
      << states.front().pose.orientation.y << " " << states.front().pose.orientation.z << " "
      << states.front().pose.orientation.w << std::endl;
    std::cout << "SCP:: states.back() orientation " << states.back().pose.orientation.x << " "
      << states.back().pose.orientation.y << " " << states.back().pose.orientation.z << " "
      << states.back().pose.orientation.w << std::endl;

    // double candidate_Tf = 2*ChooseFinalTime(states.front(), states.back());

    // get sample increment
    double radius;
    if (!cfg_.Get<double>("robot_radius", radius)) {
      radius = 0.26;
      // TODO(Somrita): Need to read from mapper.config instead of granite.config, i.e., need a new config listener
      // ROS_ERROR("Failed to load robot radius from config");
      // return;
    }

    geometry_msgs::InertiaStamped inertia_msg;
    if (!ff_util::FlightUtil::GetInertiaConfig(inertia_msg)) {
      ROS_ERROR("Failed to load inertia from config");
      return;
    }

    // try to get zones
    std::vector<ff_msgs::Zone> zones;
    if (!load_map()) {
      ROS_ERROR("Planner failed to load keepins and keepouts");
      return;
    }

    // Check if already at goal state
    scp::Vec3 diff;
    diff << std::abs(xg(0)-x0(0)),
            std::abs(xg(1)-x0(1)),
            std::abs(xg(2)-x0(2));
    diff -= scp::Vec3::Ones() * map_res_;
    if (diff(0) < 0 && diff(1) < 0 && diff(2) < 0) {
     ROS_INFO_STREAM("Start and goal are within map resolution: " << diff.transpose());
    }

    // update solver settings
    std::cout << "SCP:: Setting x0 as " << x0.transpose() << std::endl;
    std::cout << "SCP:: Setting xg as " << xg.transpose() << std::endl;
    top->x0 = x0;
    top->xg = xg;
    top->radius_ = radius;
    // top->mass = mass;
    top->mass = inertia_msg.inertia.m;
    top->J << inertia_msg.inertia.ixx, inertia_msg.inertia.ixy, inertia_msg.inertia.ixz,
              inertia_msg.inertia.ixy, inertia_msg.inertia.iyy, inertia_msg.inertia.iyz,
              inertia_msg.inertia.ixz, inertia_msg.inertia.iyz, inertia_msg.inertia.izz;
    top->Jinv = top->J.inverse();
    top->keep_in_zones_ = keep_in_zones_;
    top->keep_out_zones_ = keep_out_zones_;

    /*
    if (candidate_Tf > top->Tf) {
      top->Tf = std::max(max_time_, candidate_Tf);
      top->dh = top->Tf / (top->N-1);
      top->UpdateDynamics();
      ROS_ERROR_STREAM("Problem Tf updated to : " << top->Tf);
      // size_t new_N = static_cast<size_t>(1+std::ceil(top->Tf / 0.1));
      // top->UpdateProblemDimension(new_N);
    }
    */

    // solve
    ROS_INFO_STREAM("Starting SCP");
    // auto start = std::chrono::high_resolution_clock::now();
    bool is_solved = top->Solve();
    // auto stop = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);
    // std::cout << "Solver worked in " << duration.count()/1e6 << "s!" << std::endl;

    std::string pass = is_solved ? "solved" : "failed";
    OUTPUT_DEBUG(
        "PlannerSCP: Planner::SCP: Finished solving with status: " << pass);
        // << " in " << duration.count()/1e6 << "s!");

    if (is_solved) {
      sample_trajectory(&plan_result.segment);
      plan_result.response = RESPONSE::SUCCESS;
      NODELET_FATAL_STREAM("Returning plan");
      ROS_INFO_STREAM("SCP::Planner found solution!");
    } else {
      ROS_ERROR_STREAM("SCP::Planner failed to find solution!");
    }
    PlanResult(plan_result);
  }

  // Called to interrupt the process
  void CancelCallback() {}

 protected:
  ff_util::ConfigServer cfg_;
  ros::Timer timer_d_;
  double planner_dt_{0.5};  // how often to sample trajectory for output
  double map_res_{0.5};     // map resolution

 private:
  std::vector<Eigen::AlignedBox3d> keep_in_zones_;
  std::vector<Eigen::AlignedBox3d> keep_out_zones_;

  void sample_trajectory(std::vector<ff_msgs::ControlState> *controls) {
    size_t N = top->N;
    scp::decimal_t dh = top->dh;

    top->PolishSolution();    // ensure quaternions are normalized

    scp::decimal_t mass = top->mass;
    scp::Mat3 J = top->J;
    scp::Mat3 Jinv = top->Jinv;

    for (size_t ii = 0; ii < top->N-1; ii++) {
      ff_msgs::ControlState state;
      state.when = ros::Time(ii*dh);

      scp::Vec3 F = top->Uprev[ii].segment(0, 3);
      scp::Vec3 M = top->Uprev[ii].segment(3, 3);
      scp::Vec3 omega = top->Xprev[ii].segment(10, 3);
      scp::Vec3 accel = 1/mass * F;
      scp::Vec3 alpha = Jinv * (M - omega.cross(J*omega));

      // Linear state
      state.pose.position.x = top->Xprev[ii](0);
      state.pose.position.y = top->Xprev[ii](1);
      state.pose.position.z = top->Xprev[ii](2);

      state.twist.linear.x = top->Xprev[ii](3);
      state.twist.linear.y = top->Xprev[ii](4);
      state.twist.linear.z = top->Xprev[ii](5);

      state.accel.linear.x = accel(0);
      state.accel.linear.y = accel(1);
      state.accel.linear.z = accel(2);

      // Rotational state
      state.pose.orientation.x = top->Xprev[ii](6);
      state.pose.orientation.y = top->Xprev[ii](7);
      state.pose.orientation.z = top->Xprev[ii](8);
      state.pose.orientation.w = top->Xprev[ii](9);

      state.twist.angular.x = omega(0);
      state.twist.angular.y = omega(1);
      state.twist.angular.z = omega(2);

      state.accel.angular.x = alpha(0);
      state.accel.angular.y = alpha(1);
      state.accel.angular.z = alpha(2);

      controls->push_back(state);
    }

    // Final state has 0 velocity and acceleration
    ff_msgs::ControlState state;
    state.when = ros::Time(dh*(N-1));
    state.pose.position.x = top->Xprev[N-1](0);
    state.pose.position.y = top->Xprev[N-1](1);
    state.pose.position.z = top->Xprev[N-1](2);
    state.pose.orientation.x = top->Xprev[N-1](6);
    state.pose.orientation.y = top->Xprev[N-1](7);
    state.pose.orientation.z = top->Xprev[N-1](8);
    state.pose.orientation.w = top->Xprev[N-1](9);
    controls->push_back(state);

    NODELET_FATAL_STREAM("Done packing controls");
  }

  bool load_map() {
    if (!cfg_.Get<double>("map_resolution", map_res_)) map_res_ = 0.5;

    double radius;
    if (!cfg_.Get<double>("robot_radius", radius)) radius = 0.26;

    std::vector<ff_msgs::Zone> zones;
    bool got = GetZones(zones);
    if (!got) return false;

    // Global min and max of map
    Eigen::Vector3d min, max;
    min << 1000.0, 1000.0, 1000.0;
    max << -1000.0, -1000.0, -1000.0;

    Eigen::Vector3d zmin, zmax;   // Min and max of each zone
    Eigen::AlignedBox3d temp;

    // Reset keepin and keepout zones
    keep_in_zones_.clear();
    keep_out_zones_.clear();

    // Iterate through each zone and save
    for (auto &zone : zones) {
      zmin << std::min(zone.min.x, zone.max.x),
          std::min(zone.min.y, zone.max.y), std::min(zone.min.z, zone.max.z);
      zmax << std::max(zone.min.x, zone.max.x),
          std::max(zone.min.y, zone.max.y), std::max(zone.min.z, zone.max.z);

      if (zone.type == ff_msgs::Zone::KEEPIN) {
        // Update global min and max of map
        for (int ii = 0; ii < 3; ii++) {
          //  Need to check across all combinations
          //  since some JSON params have flipped corners
          min(ii) = std::min(min(ii), zmin(ii));
          min(ii) = std::min(min(ii), zmax(ii));
          max(ii) = std::max(max(ii), zmin(ii));
          max(ii) = std::max(max(ii), zmax(ii));
        }

        temp.extend(Eigen::Vector3d(zmin(0), zmin(1), zmin(2)));
        temp.extend(Eigen::Vector3d(zmax(0), zmax(1), zmax(2)));
        keep_in_zones_.push_back(temp);
        std::cout << "Keepin zone: " << zmin.transpose() << " " << zmax.transpose() << std::endl;
      } else {
        // Inflate obstacles by Astrobee radius
        zmin << zmin(0) - radius, zmin(1) - radius, zmin(2) - radius;
        zmax << zmax(0) + radius, zmax(1) + radius, zmax(2) + radius;

        temp.extend(Eigen::Vector3d(zmin(0), zmin(1), zmin(2)));
        temp.extend(Eigen::Vector3d(zmax(0), zmax(1), zmax(2)));
        keep_out_zones_.push_back(temp);
        std::cout << "Keepin zone: " << zmin.transpose() << " " << zmax.transpose() << std::endl;
      }
    }
    bool add_custom_keep_out_zone = true;
    if (add_custom_keep_out_zone) {
      Eigen::AlignedBox3d temp;
      temp.extend(Eigen::Vector3d(-0.1, -0.1, -2));
      temp.extend(Eigen::Vector3d(0.1, 0.1, 0));
      keep_out_zones_.push_back(temp);
    }

    std::cout << "# of keepin zones: " << keep_in_zones_.size() << std::endl;
    std::cout << "# of keepout zones: " << keep_out_zones_.size() << std::endl;

    // Update position bds for solver
    for (size_t ii = 0; ii < 3; ii++) {
      top->pos_min_(ii) = min(ii);
      top->pos_max_(ii) = max(ii);
    }

    if (keep_in_zones_.size() == 0) {
      ROS_ERROR("Zero keepin zones!! Plan failed");
      return false;
    }

    return true;
  }

  bool LoadActionParams(const ff_msgs::PlanGoal &goal) {
    // Planner params
    faceforward_ = goal.faceforward;
    check_obstacles_ = goal.check_obstacles;
    desired_vel_ = goal.desired_vel;
    desired_accel_ = goal.desired_accel;
    desired_omega_ = goal.desired_omega;
    desired_alpha_ = goal.desired_alpha;
    max_time_ = goal.max_time.toSec();
    control_rate_ = goal.desired_rate;

    // Solver params
    top->faceforward_     = faceforward_;
    top->desired_vel_     = desired_vel_;
    top->desired_accel_   = desired_accel_;
    top->desired_omega_   = desired_omega_;
    top->desired_alpha_   = desired_alpha_;
    top->max_time_        = max_time_;
    top->control_rate_    = control_rate_;
    // top->UpdateStateCons();
    // top->UpdateVelCons();
    // top->UpdateAngVelCons();

    // check validity of params
    if (control_rate_ == 0.0 || desired_vel_ == 0.0 || desired_accel_ == 0.0 ||
        desired_omega_ == 0.0 || desired_alpha_ == 0.0) {
      return false;
    }
    return true;
  }
};

PLUGINLIB_EXPORT_CLASS(planner_scp_gusto::PlannerSCPGustoNodelet,
  nodelet::Nodelet);

}  // namespace planner_scp_gusto
