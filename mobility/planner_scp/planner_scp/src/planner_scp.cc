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

#include <geometry_msgs/PoseArray.h>
#include <std_msgs/Float32MultiArray.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
// #include <eigen_conversions/eigen_msg.h>
// #include <tf_conversions/tf_eigen.h>
#include <boost/lexical_cast.hpp>

#include <scp/traj_opt.h>

// FSW libraries
#include <ff_util/config_server.h>
#include <ff_util/ff_nodelet.h>
#include <ff_util/ff_service.h>
#include <msg_conversions/msg_conversions.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <choreographer/planner.h>
#include <jsonloader/keepout.h>
#include <config_reader/config_reader.h>

// FSW messages
#include <ff_msgs/Zone.h>

#include <chrono>

#define DEBUG true
#define OUTPUT_DEBUG NODELET_DEBUG_STREAM

namespace planner_scp {

using RESPONSE = ff_msgs::PlanResult;

class PlannerSCP : public planner::PlannerImplementation {
 public:
  PlannerSCP() : planner::PlannerImplementation("scp", "SCP planner") {}
  virtual ~PlannerSCP() {}

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
  virtual bool InitializePlanner(ros::NodeHandle *nh) {
    // Grab some configuration parameters for this node from the LUA config
    cfg_.Initialize(GetPrivateHandle(), "mobility/planner_scp.config");
    cfg_.Listen(boost::bind(&PlannerSCP::ReconfigureCallback, this, _1));

    // Setup a timer to forward diagnostics
    timer_d_ =
        nh->createTimer(ros::Duration(ros::Rate(DEFAULT_DIAGNOSTICS_RATE)),
                        &PlannerSCP::DiagnosticsCallback, this, false, true);

    top = new scp::TOP(10., 201);
    top->Solve();

    nh_ = nh;

    epsilon_ = cfg_.Get<double>("epsilon");

    use_2d = false;
    if (cfg_.Get<bool>("two_d", use_2d)) use_2d = true;
    // if (!use_2d) ROS_ERROR_STREAM("SCP does not support 3D yet!");

    return true;
  }


  bool ReconfigureCallback(dynamic_reconfigure::Config &config) {
    cfg_.Reconfigure(config);
    return true;
  }


  void DiagnosticsCallback(const ros::TimerEvent &event) {
    SendDiagnostics(cfg_.Dump());
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

    // double candidate_Tf = 2*ChooseFinalTime(states.front(), states.back());

    // get sample increment
    double radius;
    if (!cfg_.Get<double>("robot_radius", radius)) radius = 0.26;

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
    top->x0 = x0;
    top->xg = xg;
    top->radius_ = radius;

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
    ROS_ERROR_STREAM("Starting SCP");
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
    } else {
      ROS_ERROR_STREAM("SCP::Planner failed to find solution!");
    }
    PlanResult(plan_result);
  }

  // Called to interrupt the process
  virtual void CancelCallback() {}

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
      } else {
        // Inflate obstacles by Astrobee radius
        zmin << zmin(0) - radius, zmin(1) - radius, zmin(2) - radius;
        zmax << zmax(0) + radius, zmax(1) + radius, zmax(2) + radius;

        temp.extend(Eigen::Vector3d(zmin(0), zmin(1), zmin(2)));
        temp.extend(Eigen::Vector3d(zmax(0), zmax(1), zmax(2)));
        keep_out_zones_.push_back(temp);
      }
    }

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
    top->UpdateStateCons();
    top->UpdateVelCons();
    top->UpdateAngVelCons();

    // check validity of params
    if (control_rate_ == 0.0 || desired_vel_ == 0.0 || desired_accel_ == 0.0 ||
        desired_omega_ == 0.0 || desired_alpha_ == 0.0) {
      return false;
    }
    return true;
  }

  double ChooseFinalTime(const geometry_msgs::PoseStamped start,
                        const geometry_msgs::PoseStamped finish) {
    Eigen::Affine3d p0 =
        msg_conversions::ros_pose_to_eigen_transform(start.pose);
    Eigen::Affine3d p1 =
        msg_conversions::ros_pose_to_eigen_transform(finish.pose);

    Eigen::AngleAxisd error_ang(p0.rotation().inverse()* p1.rotation());
    Eigen::Vector3d error_vec(p1.translation() - p0.translation());

    // Make sure that we are taking the minimum angle of rotatin
    error_ang.angle() = fmod(error_ang.angle(), 2.0 * M_PI);
    if (error_ang.angle() > M_PI)
      error_ang.angle() -= 2.0 * M_PI;
    if (error_ang.angle() < -M_PI)
      error_ang.angle() += 2.0 * M_PI;
    if (error_ang.angle() < 0) {
      error_ang.angle() *= -1.0;
      error_ang.axis() *= -1.0;
    }

    // Get the linear and rotational magnitudes
    double lin_m = error_vec.norm();
    double rot_m = error_ang.angle();
    // Special case: the beginning and end poses are the same
    if (lin_m < epsilon_ && rot_m < epsilon_) {
      return epsilon_;
    }
    // Greedy phase - calculate the linear and rotational ramps separately
    double lin_t = 0.0, lin_r = 0.0, lin_c = 0.0, lin_h = 0.0;
    lin_t = GreedyRamp(lin_m, desired_vel_, desired_accel_, lin_r, lin_c, lin_h, epsilon_);
    double rot_t = 0.0, rot_r = 0.0, rot_c = 0.0, rot_h = 0.0;
    rot_t = GreedyRamp(rot_m, desired_omega_, desired_alpha_, rot_r, rot_c, rot_h, epsilon_);
    // Now determine whether the angular or linear component dominates...
    double tmin = std::max(lin_t, rot_t);
    return tmin;
  }

  //////////////////////////////////////////////////////////////////////////////
  // Given some linear or angular displacement (d) and a maximum acceleration //
  // (a) and velocity (v), return the time taken (t) to achieve the motion as //
  // quickly as possible. Also return the time (r) needed to accelerate to or //
  // decelerate from the cruise phase, which lasts a given time (c) at a      //
  // constant velocity (h).                                                   //
  //////////////////////////////////////////////////////////////////////////////
  double GreedyRamp(double d,     // Distance
                    double v,     // Max velocity
                    double a,     // Max acceleration
                    double &r,    // Ramp time
                    double &c,    // Time at constant velocity
                    double &h,    // Constant velocity
                    double epsilon) {
    if (d < epsilon)             // Doesn't work for small / negative numbers
      return 0.0;
    h = sqrt(a * d);              // The max vel required if one had zero dwell
    if (h > v) {                  // If the required velocity is too high
      h = v;                      // Clamp the velocity to maximum
      r = h / a;                  // Time taken to ramp up and down to max vel
      c = (d - h * h / a) / h;    // Dwell time at maxmimum velocity
    } else {                      // If we don't need to achieve max velocity
      r = d / h;                  // Time taken to ramp up/down to right vel
      c = 0.0;                    // Time at constant velocity
    }
    return (r * 2.0 + c);         // Minimum time required to complete action
  }

  // void ConnectedCallback(void) {
  //   NODELET_DEBUG_STREAM("ConnectedCallback()");
  //   if (!client_z_.IsConnected()) return;  // Zone
  //   if (!client_r_.IsConnected()) return;  // Register
  //   if (state_ != INITIALIZING) return;    // Don't initialize twice
  //   // Register this planner
  //   NODELET_DEBUG_STREAM("Registering planner");
  //   client_r_.Call(registration_);
  //   // Move to waiting state
  //   state_ = WAITING;
  // }

  // // Timeout on a trajectory generation request
  // void GetZoneTimeoutCallback(void) {
  //   return InitFault("Timeout connecting to the get zone service");
  // }
};

// PLUGINLIB_DECLARE_CLASS(planner_scp, PlannerSCP,
//                         planner_scp::PlannerSCP, nodelet::Nodelet);

PLUGINLIB_EXPORT_CLASS(planner_scp::PlannerSCP, nodelet::Nodelet);
}  // namespace planner_scp
