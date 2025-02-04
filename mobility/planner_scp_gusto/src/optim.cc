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

#include "planner_scp_gusto/optim.h"

#include <sys/stat.h>

#include <algorithm>
#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <fstream>
#include <tuple>
#include <chrono>
#include <ctime>
#include <sstream>


#ifdef PROFILING
#undef PROFILING
#endif
#include <torch/torch.h>

namespace scp {

TOP::TOP(decimal_t Tf_, int N_)
    : N(N_),
      Tf(Tf_),
      net(std::make_shared<Net>()),
      optimizer(net->parameters(), torch::optim::AdamOptions(0.001)),
      spline_net(std::make_shared<SplineNet>()),
      spline_optimizer(spline_net->parameters(), torch::optim::AdamOptions(0.001)) {
  std::cout << "TOP constructor called!" << std::endl;
  state_dim = 13;
  state_dim_lin = 6;
  state_dim_nlin = 7;
  state_bd_dim = 7;
  pos_dim = 3;
  lin_vel_dim = 3;
  quat_dim = 4;
  ang_vel_dim = 3;
  control_dim = 6;
  control_dim_lin = 3;
  control_dim_nlin = 3;
  dh = Tf / N;

  // Network for warm start
  use_nn_warm_start = false;
  nn_model_path = "";

  // Mode to create training data
  nn_training_mode = false;
  // Spline or regular network
  nn_spline_mode = false;
  nn_spline_model_path = "";

  // Folder to save outputs
  output_dir = "planner_scp_gusto_outputs";

  // Whether to print constraints to file and save traj to file
  save_constraints_to_file = true;
  save_trajectory_to_file = true;

  // TODO(somrita): Implement all of these
  is_granite = false;
  enforce_init_cond = true;
  enforce_final_cond = true;
  enforce_lin_dynamics = true;
  enforce_rot_dynamics = true;
  enforce_force_norm = false;
  enforce_moment_norm = false;
  enforce_state_LB = false;
  enforce_state_UB = false;
  enforce_lin_vel_norm = false;
  enforce_ang_vel_norm = false;
  enforce_trust_region_const = false;
  enforce_obs_avoidance_const = false;
  enforce_state_bounds = true;

  enforce_lin_vel_limit = false;
  enforce_ang_vel_limit = false;

  lin_vel_limit = 0.2000;
  ang_vel_limit = 0.1745;

  penalize_total_force = false;
  penalize_total_moment = false;

  num_force_norm_slack_vars = penalize_total_force ? (control_dim_lin+1)*(N-1) : control_dim_lin*(N-1);
  num_force_norm_slack_vars_per_iter = penalize_total_force ? (control_dim_lin+1) : control_dim_lin;
  num_moment_norm_slack_vars = penalize_total_moment ? (control_dim_nlin+1)*(N-1) : control_dim_nlin*(N-1);
  num_moment_norm_slack_vars_per_iter = penalize_total_moment ? (control_dim_nlin+1) : control_dim_nlin;

  // TODO(somrita): Are the following implemented?
  free_final_state = false;
  state_con_strict = false;  // true=enforce cons tight, false=use slack vars
  lin_vel_strict = false;   // true=enforce cons tight, false=use slack vars
  ang_vel_strict = false;   // true=enforce cons tight, false=use slack vars

  solver = NULL;

  x0 << 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
  xg << 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;

  // Defaults are ISS params (will be written by planner_scp_gusto_nodelet)
  radius_ = 0.26;
  mass = 9.583788668;
  J << 0.153427995, 0, 0,
    0, 0.14271405, 0,
    0, 0, 0.162302759;
  Jinv = J.inverse();

  desired_vel_ = 0.2000;
  desired_accel_ = 0.0175;
  desired_omega_ = 0.1745;
  desired_alpha_ = 0.1745;

  keep_in_zones_.clear();
  keep_out_zones_.clear();

  // Placeholders for initialization run (will be set by planner_scp_gusto_nodelet)
  x_max << 20.0, 20.0, 20.0,
    lin_vel_limit, lin_vel_limit, lin_vel_limit,
    1, 1, 1, 1,
    ang_vel_limit, ang_vel_limit, ang_vel_limit;
  x_min = -x_max;

  std::cout << "[TOP constructor] Min position" << x_min(0) << x_min(1) << x_min(2) << std::endl;
  std::cout << "[TOP constructor] Min quaternion" << x_min(6) << x_min(7) << x_min(8) << x_min(9) <<std::endl;

  // OSQP solver settings (must be set before UpdateProblemDimension)
  abs_tol_ = 1e-5;  // default 1e-03
  rel_tol_ = 1e-5;  // default 1e-03
  primal_tol_ = 1e-8;  // default 1e-04
  dual_tol_ = 1e-8;  // default 1e-04
  rho_ = 0.3;  // default 0.1
  sigma_ = 1e-8;  // default 1e-06
  max_iter_solver_ = 8000;  // default 4000

  ResetSCPParams();
  UpdateProblemDimension(N);

  // Run OSQP initialization run on demo problem
  solver->settings()->setVerbosity(false);
  if (!solver->solve()) {
    solver_ready_ = false;
  }

  if (solver_ready_) {
    std::cout << "[TOP constructor] Solver ready to solve!" << std::endl;
  } else {
    std::cout << "[TOP constructor] Solver failed to instantiate!" << std::endl;
  }
}

TOP::~TOP() {
  std::cout << "Destructor called for TOP object." << std::endl;
}

size_t TOP::GetNumTOPVariables() {
  // TODO(somrita): Continue updating these
  // size_t num_force_norm_slack_vars = penalize_total_force ? (control_dim_lin+1)*(N-1) : control_dim_lin*(N-1);
  // size_t num_moment_norm_slack_vars = penalize_total_moment ? (control_dim_nlin+1)*(N-1) : control_dim_nlin*(N-1);
  return state_dim * N                  // State variables
         + control_dim * (N - 1);        // Control variables
        //  + num_force_norm_slack_vars    // Force norm slack variables
        //  + num_moment_norm_slack_vars   // Moment norm slack variables
        //  + state_bd_dim * (N - 1)       // State LB slack variables
        //  + state_bd_dim * (N - 1)       // State UB slack variables
        //  + (lin_vel_dim + 1) * (N - 1)  // Linear velocity norm slack variables
        //  + (ang_vel_dim + 1) * (N - 1);  // Angular velocity norm slack variables
        //  + 3 * (N - 1);                 // Obstacle avoidance binary variables
        //  + 2 * (N - 1);                 // Obstacle avoidance slack variables
}

size_t TOP::GetNumTOPConstraints(bool verbose = false) {
  size_t num_init_cond_constr = state_dim;
  size_t num_final_cond_constr = state_dim;
  size_t num_lin_dynamics_constr = state_dim_lin * (N - 1);   // (x,y,z) and (vx,vy,vz) for each time step
  size_t num_rot_dynamics_constr = state_dim_nlin * (N - 1);  // (q0,q1,q2,q3) and (wx,wy,wz) for each time step
  size_t num_obs_avoidance_const = N * pos_dim;               // Exactly 3 (XYZ) constraints per time step
  size_t num_state_bounds_const = N * pos_dim;                      // 3 (XYZ) constraints per time step
  size_t num_lin_vel_limit_const = N * lin_vel_dim;                 // 3 (XYZ) constraints per time step
  size_t num_ang_vel_limit_const = N * ang_vel_dim;                 // 3 (XYZ) constraints per time step
  size_t num_total_constr = (enforce_init_cond ? num_init_cond_constr : 0) +
                           (enforce_final_cond ? num_final_cond_constr : 0) +
                           (enforce_lin_dynamics ? num_lin_dynamics_constr : 0) +
                           (enforce_rot_dynamics ? num_rot_dynamics_constr : 0) +
                          //  (enforce_obs_avoidance_const ? num_obs_avoidance_const : 0) +
                           ((enforce_state_bounds || enforce_obs_avoidance_const) ? num_state_bounds_const : 0) +
                           (enforce_lin_vel_limit ? num_lin_vel_limit_const : 0) +
                           (enforce_ang_vel_limit ? num_ang_vel_limit_const : 0);
  if (enforce_force_norm || enforce_moment_norm || enforce_state_LB || enforce_state_UB || enforce_lin_vel_norm ||
      enforce_ang_vel_norm) {
    throw std::runtime_error("Error: Constraints not implemented yet!");
    return false;
  }
  if (verbose) {
    // Print which constraints are enabled and corresponding number of constraints
    std::cout << "[TOP::GetNumTOPConstraints] enforce_init_cond: " << enforce_init_cond << "  (" << num_init_cond_constr
              << " constraints)" << std::endl;
    std::cout << "[TOP::GetNumTOPConstraints] enforce_final_cond: " << enforce_final_cond
              << "  (" << num_final_cond_constr << " constraints)" << std::endl;
    std::cout << "[TOP::GetNumTOPConstraints] enforce_lin_dynamics: " << enforce_lin_dynamics
              << "  (" << num_lin_dynamics_constr << " constraints)" << std::endl;
    std::cout << "[TOP::GetNumTOPConstraints] enforce_rot_dynamics: " << enforce_rot_dynamics
              << "  (" << num_rot_dynamics_constr << " constraints)" << std::endl;
    std::cout << "[TOP::GetNumTOPConstraints] enforce_obs_avoidance_const: " << enforce_obs_avoidance_const
              << "  (" << num_obs_avoidance_const << " constraints)" << std::endl;
    std::cout << "[TOP::GetNumTOPConstraints] enforce_state_bounds: " << enforce_state_bounds
              << "  (" << num_state_bounds_const << " constraints)" << std::endl;
    std::cout << "[TOP::GetNumTOPConstraints] enforce_lin_vel_limit: " << enforce_lin_vel_limit
              << "  (" << num_lin_vel_limit_const << " constraints)" << std::endl;
    std::cout << "[TOP::GetNumTOPConstraints] enforce_ang_vel_limit: " << enforce_ang_vel_limit
              << "  (" << num_ang_vel_limit_const << " constraints)" << std::endl;
    std::cout << "[TOP::GetNumTOPConstraints] Total constraints: " << num_total_constr << std::endl;
  }

  return num_total_constr;
}

Vec3 TOP::MinPos() { return Vec3(x_min(0), x_min(1), x_min(2)); }

Vec3 TOP::MaxPos() { return Vec3(x_max(0), x_max(1), x_max(2)); }

void TOP::printBoxHeader(const std::string& title, int width = 60) {
  int padding = (width - title.size() - 2) / 2;  // Calculate padding
  std::cout << "╔" << std::string(width - 2, '═') << "╗\n";
  std::cout << "║" << std::string(padding, ' ') << title << std::string(padding, ' ')
            << ((title.size() % 2 == 0) ? "" : " ") << "║\n";
  std::cout << "╠" << std::string(width - 2, '═') << "╣\n";
}

void TOP::ResetSCPParams() {
  // SCP parameters
  solved_ = false;
  max_iter = 3;
  Delta_0 = 100;
  Delta = Delta_0;
  omega_0 = 1.;
  omega = omega_0;
  omega_max = 1.0e9;
  rho_0 = 0.01;
  rho_1 = 5.;
  beta_fail = 0.1;
  beta_succ = 2.;
  gamma_fail = 5.;
  convergence_threshold = 1e-2;
}

void TOP::UpdateProblemDimension(size_t N_) {
  // Allocate matrices for variables and constraints

  N = N_;
  dh = Tf / N;
  std::cout << "[TOP::UpdateProblemDimension] N = " << N << std::endl;
  std::cout << "[TOP::UpdateProblemDimension] mass: " << mass << std::endl;
  std::cout << "[TOP::UpdateProblemDimension] inertia: " << J << std::endl;
  std::cout << "[TOP::UpdateProblemDimension] desired accel: " << desired_accel_ << std::endl;

  if (!solver) {
    // delete solver;
    solver = new OsqpEigen::Solver();
  } else {
    solver->clearSolver();
    // solver->clearSolverVariables();
    solver->data()->clearHessianMatrix();
    solver->data()->clearLinearConstraintsMatrix();
  }

  Xprev.resize(N);
  Uprev.resize(N-1);

  size_t n_obs = keep_out_zones_.size();
  support_vectors.resize(n_obs*(N-1));
  obs_ub.resize(n_obs*(N-1));
  obs_clearance = 0.10;   // 10cm clearance from obstacles

  fs.resize(N-1);
  As.resize(N-1);
  Bs.resize(N-1);

  size_t num_vars = GetNumTOPVariables();
  size_t num_cons = GetNumTOPConstraints(/*verbose=*/ true);

  hessian.resize(num_vars, num_vars);
  linear_con_mat.resize(num_cons, num_vars);
  gradient.resize(num_vars);
  lower_bound.resize(num_cons);
  upper_bound.resize(num_cons);
  qp_soln.resize(num_vars);

  std::cout << "[TOP::UpdateProblemDimension] Num vars: " << num_vars << " Num cons: " << num_cons << std::endl;
  std::cout << "[TOP::UpdateProblemDimension] linear_con_mat size: " << linear_con_mat.rows() << " x "
            << linear_con_mat.cols() << std::endl;

  if (use_nn_warm_start) {
    std::cout << "[TOP::UpdateProblemDimension] Using NN warm start" << std::endl;
    InitTrajWarmStart();
  } else {
    std::cout << "[TOP::UpdateProblemDimension] Using straight line cold start" << std::endl;
    InitTrajStraightline();
  }

  std::cout << "[TOP::UpdateProblemDimension] Init traj start: " << Xprev[0].transpose() << std::endl;
  std::cout << "[TOP::UpdateProblemDimension] Init traj end: " << Xprev[N-1].transpose() << std::endl;

  std::cout << "[TOP::UpdateProblemDimension] Keep in zones: " << std::endl;
  for (size_t i = 0; i < keep_in_zones_.size(); ++i) {
    std::cout << "Zone " << i << std::endl;
    std::cout << "min: " << keep_in_zones_[i].min().transpose() << std::endl;
    std::cout << "max: " << keep_in_zones_[i].max().transpose() << std::endl;
  }
  std::cout << "[TOP::UpdateProblemDimension] Keep out zones: " << std::endl;
  for (size_t i = 0; i < keep_out_zones_.size(); ++i) {
    std::cout << "Zone " << i << std::endl;
    std::cout << "min: " << keep_out_zones_[i].min().transpose() << std::endl;
    std::cout << "max: " << keep_out_zones_[i].max().transpose() << std::endl;
  }

  // Set warm start
  for (size_t ii = 0; ii < N; ii++) {
    qp_soln.segment(state_dim*ii, state_dim) = Xprev[ii];
  }
  for (size_t ii = 0; ii < N-1; ii++) {
    qp_soln.segment(state_dim*N+control_dim*ii, control_dim) = Uprev[ii];
  }

  for (size_t ii = 0; ii < num_cons; ii++) {
    lower_bound(ii) = -OsqpEigen::INFTY;
    upper_bound(ii) = OsqpEigen::INFTY;
  }

  SetSimpleConstraints();
  SetSimpleCosts();

  verbose_ = true;  // TODO(somrita): Change back to false
  warm_start_ = true;
  solver->settings()->setWarmStart(warm_start_);
  solver->settings()->setAbsoluteTolerance(abs_tol_);
  // std::cout << "Set abs tol to " << solver->settings()->eps_rel <<std::endl;
  solver->settings()->setRelativeTolerance(rel_tol_);
  solver->settings()->setPrimalInfeasibilityTollerance(primal_tol_);
  solver->settings()->setDualInfeasibilityTollerance(dual_tol_);
  // which justifies using a low value of sigma and a high value of rho for equality constraints.
  solver->settings()->setRho(rho_);
  solver->settings()->setSigma(sigma_);
  solver->settings()->setMaxIteraction(max_iter_solver_);
  solver->settings()->setScaling(1);  // Enable scaling
  solver->settings()->setPolish(true);          // Enable solution polishing

  solver->settings()->setVerbosity(verbose_);
  solver->data()->setNumberOfVariables(num_vars);
  solver->data()->setNumberOfConstraints(num_cons);

  // ValidateQPProblem(); // Slow, checks convexity

  // Dry run for initialization
  if (!solver->data()->setHessianMatrix(hessian)) {
    solver_ready_ = false;
  } else if (!solver->data()->setGradient(gradient)) {
    solver_ready_ = false;
  } else if (!solver->data()->setLinearConstraintsMatrix(linear_con_mat)) {
    solver_ready_ = false;
  } else if (!solver->data()->setLowerBound(lower_bound)) {
    solver_ready_ = false;
  } else if (!solver->data()->setUpperBound(upper_bound)) {
    solver_ready_ = false;
  } else if (!solver->initSolver()) {
    solver_ready_ = false;
  } else {
    solver_ready_ = true;
  }
}

void TOP::InitTrajStraightline() {
  // See quaternion convention
  // http://wiki.ros.org/tf2/Tutorials/Quaternions#Components_of_a_quaternion
  Quat q0 = Quat(x0(9), x0(6), x0(7), x0(8));
  Quat qg = Quat(xg(9), xg(6), xg(7), xg(8));
  std::cout << "[TOP::InitTrajStraightLine] x0: " << x0.transpose() << std::endl;
  std::cout << "[TOP::InitTrajStraightLine] xg: " << xg.transpose() << std::endl;

  for (size_t ii = 0; ii < N; ii++) {
    Xprev[ii] = x0 + (xg-x0)*ii/(N-1.);
    // Quat q = q0;
    Quat q = q0.slerp(ii/(N-1.), qg);
    Xprev[ii](6) = q.x();
    Xprev[ii](7) = q.y();
    Xprev[ii](8) = q.z();
    Xprev[ii](9) = q.w();
  }

  for (size_t ii = 0; ii < N-1; ii++) {
    for (size_t jj = 0; jj < control_dim; jj++) {
      Uprev[ii](jj) = 0;
    }
  }
  if (save_trajectory_to_file) {
    std::string fname = std::string(is_granite ? "granite" : "iss") +"_initial_straight_line_trajectory";
    WriteTrajectoryToFile(fname);
  }
}

void TOP::InitTrajWarmStart() {
  // Settings
  bool U_linear_only = true;
  std::string Xinit_method =
    (nn_spline_mode ? "spline" : "linear_interpolation");  // "spline, "forward_dynamics" or "linear_interpolation"


  if (Xinit_method == "forward_dynamics" || Xinit_method == "linear_interpolation") {
    // Load model
    LoadModel(nn_model_path);
    // For these modes we call the regular InferenceNN that returns U0 and Uf.
    // Call InferenceNN(x0, xg) to get U0, Uf
    Vec6 U0, Uf;
    std::tie(U0, Uf) = InferenceNN(x0, xg);
    // Interpolate linearly for N steps to get Uprev
    Vec6Vec U_inter;
    for (size_t i = 0; i < N; ++i) {
      Vec6 U = U0 + (i/(N-1))*(Uf - U0);
      if (U_linear_only) {
        U(3) = 0.0;
        U(4) = 0.0;
        U(5) = 0.0;
      }
      U_inter.push_back(U);
    }
    Uprev = U_inter;

    if (Xinit_method == "forward_dynamics") {
      // Use dynamics to get Xprev
      Vec13Vec X_inter;
      X_inter.push_back(x0);
      for (size_t i = 0; i < N; ++i) {
        Vec13 X = ForwardDynamics(X_inter[i], Uprev[i]);
        X_inter.push_back(X);
      }
      Xprev = X_inter;
    } else if (Xinit_method == "linear_interpolation") {
      // Linearly interpolate between x0 and xg
      Vec13Vec X_inter;
      for (size_t i = 0; i < N; i++) {
        X_inter.push_back(x0 + (xg - x0) * i / (N - 1));
      }
      Xprev = X_inter;
    }
  } else if (Xinit_method == "spline") {
    // Load model
    LoadModel(nn_spline_model_path);
    // Get the spline coefficients from inference.
    Vec4 coeff_x, coeff_y, coeff_z;
    std::tie(coeff_x, coeff_y, coeff_z) = InferenceNNSpline(x0, xg);

    // Create Xprev using the spline for x, y, and z and linear interpolation for the remaining state.
    Vec13Vec X_inter;
    for (size_t i = 0; i < N; ++i) {
      double t = (N > 1) ? static_cast<double>(i) / (N - 1) : 0.0;
      Vec13 X;

      // Evaluate cubic splines for x, y, and z.
      X(0) = coeff_x(0) + coeff_x(1) * t + coeff_x(2) * t * t + coeff_x(3) * t * t * t;
      X(1) = coeff_y(0) + coeff_y(1) * t + coeff_y(2) * t * t + coeff_y(3) * t * t * t;
      X(2) = coeff_z(0) + coeff_z(1) * t + coeff_z(2) * t * t + coeff_z(3) * t * t * t;

      // For remaining state indices (3 to 12), linearly interpolate between x0 and xg.
      for (int j = 3; j < 13; ++j) {
        X(j) = x0(j) + (xg(j) - x0(j)) * t;
      }
      X_inter.push_back(X);
    }
    Xprev = X_inter;

    // Initialize Uprev to all zeros.
    Vec6Vec U_inter(N, Vec6::Zero());
    Uprev = U_inter;
  }

  if (save_trajectory_to_file) {
    std::string fname = std::string(is_granite ? "granite" : "iss") + "_initial_nn_warm_start_trajectory";
    WriteTrajectoryToFile(fname);
  }
  return;
}

decimal_t TOP::ComputeSignedDistance(const Vec3& point) {
  // Initialize the signed distance
  decimal_t signed_distance = 0.0;

  // Extract the keep-out zone and apply a buffer for clearance
  Eigen::AlignedBox3d box = keep_out_zones_[0];
  Vec3 ko_min_original = box.min();
  Vec3 ko_max_original = box.max();
  Vec3 ko_min = ko_min_original - Vec3(obs_clearance, obs_clearance, obs_clearance);
  Vec3 ko_max = ko_max_original + Vec3(obs_clearance, obs_clearance, obs_clearance);

  // Clip ko_min and ko_max to be within position bounds
  ko_min = ko_min.cwiseMax(MinPos());
  ko_max = ko_max.cwiseMin(MaxPos());

  // Compute the signed distance for each dimension (x, y, z)
  decimal_t dist_x = std::max(0.0, std::max(ko_min[0] - point[0], point[0] - ko_max[0]));
  decimal_t dist_y = std::max(0.0, std::max(ko_min[1] - point[1], point[1] - ko_max[1]));
  decimal_t dist_z = std::max(0.0, std::max(ko_min[2] - point[2], point[2] - ko_max[2]));

  // Identify the maximum signed distance
  signed_distance = std::max({dist_x, dist_y, dist_z});

  return signed_distance;
}

// Compute the signed distance gradient for a point with respect to the box
Vec3 TOP::ComputeSignedDistanceGradient(const Vec3& point) {
  // Initialize the gradient
  Vec3 grad(0, 0, 0);

  // Extract the keep-out zone and apply a buffer for clearance
  Eigen::AlignedBox3d box = keep_out_zones_[0];
  Vec3 ko_min_original = box.min();
  Vec3 ko_max_original = box.max();
  Vec3 ko_min = ko_min_original - Vec3(obs_clearance, obs_clearance, obs_clearance);
  Vec3 ko_max = ko_max_original + Vec3(obs_clearance, obs_clearance, obs_clearance);

  // Clip ko_min and ko_max to be within position bounds
  ko_min = ko_min.cwiseMax(MinPos());
  ko_max = ko_max.cwiseMin(MaxPos());

  // Compute the signed distance for each dimension (x, y, z)
  decimal_t dist_x = std::max(0.0, std::max(ko_min[0] - point[0], point[0] - ko_max[0]));
  decimal_t dist_y = std::max(0.0, std::max(ko_min[1] - point[1], point[1] - ko_max[1]));
  decimal_t dist_z = std::max(0.0, std::max(ko_min[2] - point[2], point[2] - ko_max[2]));

  // Identify the maximum signed distance
  decimal_t max_dist = std::max({dist_x, dist_y, dist_z});

  // Compute the gradient based on the max distance
  if (max_dist == dist_x) {
      if (point[0] < ko_min[0]) {
        grad[0] = -1.0;  // Point is on the left side of the box
      } else if (point[0] > ko_max[0]) {
          grad[0] = 1.0;  // Point is on the right side of the box
      }
  } else if (max_dist == dist_y) {
      if (point[1] < ko_min[1]) {
        grad[1] = -1.0;  // Point is below the box
      } else if (point[1] > ko_max[1]) {
          grad[1] = 1.0;  // Point is above the box
      }
  } else if (max_dist == dist_z) {
      if (point[2] < ko_min[2]) {
        grad[2] = -1.0;  // Point is behind the box
      } else if (point[2] > ko_max[2]) {
          grad[2] = 1.0;  // Point is in front of the box
      }
  }

  return grad;  // Return the gradient vector
}

// Function to calculate Q_mat
Mat4x3 TOP::CalculateQMat(const Eigen::Quaterniond& quaternion) {
  // Extract quaternion components
  double q_x = quaternion.x();
  double q_y = quaternion.y();
  double q_z = quaternion.z();
  double q_w = quaternion.w();

  Mat4x3 Q_mat;
  std::string frame = "body";  // "body" or "fixed"

  if (frame == "fixed") {
    // Construct ET matrix from equation 16 in https://arxiv.org/pdf/0811.2889
    // Mat4x3 E_T_mat;
    // E_T_mat << -q_x, -q_y, -q_z,
    //            q_w, q_z,  -q_y,
    //            -q_z,  q_w, q_x,
    //            q_y,  -q_x,  q_w;
    // This produces \dot{q} = 0.5 * E_T_mat * omega = \dot{q_0, q_1, q_2, q_3}
    // We want \dot{q_1, q_2, q_3, q_0}
    // Thefore, the matrix is:
    Q_mat <<  q_w,  q_z, -q_y,
              -q_z,  q_w,  q_x,
              q_y, -q_x,  q_w,
              -q_x, -q_y, -q_z;
  } else if (frame == "body") {
    // In body frame
    // Construct GT matrix from equation 18 in https://arxiv.org/pdf/0811.2889
    // Mat4x3 G_T_mat;
    // G_T_mat << -q_x, -q_y, -q_z,
    //            q_w, -q_z,  q_y,
    //            q_z,  q_w, -q_x,
    //            -q_y,  q_x,  q_w;
    // This produces \dot{q} = 0.5 * G_T_mat * omega' = \dot{q_0, q_1, q_2, q_3}
    // We want \dot{q_1, q_2, q_3, q_0}
    // Thefore, the matrix is:
    Q_mat <<  q_w,  -q_z, -q_y,
              q_z,  q_w,  -q_x,
              -q_y, q_x,  q_w,
              -q_x, -q_y, -q_z;
  }

  return Q_mat;
}

void TOP::SetSimpleConstraints() {
  std::cout << "[TOP::SetSimpleConstraints] Setting simple constraints..." << std::endl;

  Mat7 eye;
  eye.setIdentity();

  size_t row_idx = 0;

  std::vector<Eigen::Triplet<double>> triplets;

  auto start_time = std::chrono::high_resolution_clock::now();

  // Initial state
  if (enforce_init_cond) {
    for (size_t ii = 0; ii < state_dim; ii++) {
      triplets.emplace_back(row_idx, ii, 1.0);  // Constrain x[0][i] (initial state component)
      lower_bound(row_idx) = x0(ii);
      upper_bound(row_idx) = x0(ii);
      row_idx++;
    }
  }

  // Goal state
  if (enforce_final_cond) {
    for (size_t ii = 0; ii < state_dim; ii++) {
      triplets.emplace_back(row_idx, state_dim * (N - 1) + ii, 1.0);  // Constrain x[N-1][i] (final state component)
      lower_bound(row_idx) = xg(ii);
      upper_bound(row_idx) = xg(ii);
      row_idx++;
    }
  }

  if (enforce_lin_dynamics) {
    for (size_t ii = 0; ii < N-1; ii++) {
      // Double integrator dynamics
      for (size_t jj = 0; jj < pos_dim; jj++) {
        // Position update: x_{i+1} = x_i + v_i * dt
        triplets.emplace_back(row_idx, ii * state_dim + jj, -1.0);       // -x_i
        triplets.emplace_back(row_idx, ii * state_dim + pos_dim + jj, -dh);   // -v_i * dt
        triplets.emplace_back(row_idx, (ii + 1) * state_dim + jj, 1.0);  // x_{i+1}
        lower_bound(row_idx) = 0.0;
        upper_bound(row_idx) = 0.0;
        ++row_idx;

        // Velocity update: v_{i+1} = v_i + u_j * dt / mass
        triplets.emplace_back(row_idx, ii * state_dim + pos_dim + jj, -1.0);       // -v_i
        triplets.emplace_back(row_idx, N * state_dim + ii * control_dim + jj, -dh/mass);    // -u_j * dt/mass
        triplets.emplace_back(row_idx, (ii + 1) * state_dim + pos_dim + jj, 1.0);  // v_{i+1}
        lower_bound(row_idx) = 0.0;
        upper_bound(row_idx) = 0.0;
        ++row_idx;
      }
    }
  }

  if (enforce_rot_dynamics) {
    NormalizeQuaternions();
    for (size_t ii = 0; ii < N - 1; ii++) {
      // Quaternion kinematics update
      // Equation 29 in https://arxiv.org/pdf/0811.2889
      // Quaternion update: q_{i+1} = q_i + 0.5 * Q(q_i) * omega_i * dt
      // Compute QMat dynamically for quaternion at time step `ii`
      Eigen::Quaterniond q_i(Xprev[ii](9), Xprev[ii](6), Xprev[ii](7), Xprev[ii](8));
      Eigen::Matrix<double, 4, 3> QMat = CalculateQMat(q_i);
      for (size_t jj = 0; jj < 4; jj++) {
        for (size_t kk = 0; kk < 3; kk++) {
          triplets.emplace_back(row_idx, ii * state_dim + pos_dim + lin_vel_dim + jj, -1.0);  // -q_i
          triplets.emplace_back(row_idx, ii * state_dim + pos_dim + lin_vel_dim + quat_dim + kk,
                                         -0.5 * dh * QMat(jj, kk));  // -0.5 * Q_mat * omega_i * dt
          triplets.emplace_back(row_idx, (ii + 1) * state_dim + pos_dim + lin_vel_dim + jj,
                                         1.0);  // q_{i+1}
        }
        // Enforce quaternion normalization at time step `ii+1`
        if (jj == 3) {  // After processing all quaternion components
          decimal_t q_norm = Xprev[ii + 1].segment(6, 4).norm();
          if (q_norm > 1e-6) {
            Xprev[ii].segment(6, 4) /= q_norm;
          }
        }
        lower_bound(row_idx) = 0.0;
        upper_bound(row_idx) = 0.0;
        ++row_idx;
      }

      // Angular velocity update
      // Calculate frot_mat = J^{-1} * (-omega_i cross (J * omega_i))
      Eigen::Vector3d omega_i = Xprev[ii].segment(10, 3);
      Eigen::Vector3d frot_mat = J.inverse() * (-omega_i.cross(J * omega_i));
      for (size_t jj = 0; jj < 3; jj++) {
        // omega_{i+1} = omega_i + J^{-1} * (u_torque - omega_i cross (J * omega_i)) * dt
        triplets.emplace_back(row_idx, ii * state_dim + pos_dim + lin_vel_dim + quat_dim + jj,
                                       -1.0);  // -omega_i
        triplets.emplace_back(row_idx, N * state_dim + ii * control_dim + control_dim_lin + jj,
                                       -dh / J(jj, jj));  // -u_torque * dt / J
        triplets.emplace_back(row_idx, (ii + 1) * state_dim + pos_dim + lin_vel_dim + quat_dim + jj,
                                       1.0);  // omega_{i+1}

        lower_bound(row_idx) = frot_mat(jj);
        upper_bound(row_idx) = frot_mat(jj);
        ++row_idx;
      }
    }
  }

  if (enforce_force_norm) {
    // Force constraints
    for (size_t ii = 0; ii < N-1; ii++) {
      for (size_t jj = 0; jj < control_dim_lin; jj++) {
        size_t s_slack_var_idx = state_dim*N+control_dim*(N-1)+ii*num_force_norm_slack_vars_per_iter+jj;
        size_t this_control_idx = state_dim*N+control_dim*ii+jj;
        // -s <=0
        triplets.emplace_back(row_idx, s_slack_var_idx, -1.0);
        upper_bound(row_idx) = 0.0;
        row_idx++;
        // -s - a <= 0
        triplets.emplace_back(row_idx, s_slack_var_idx, -1.0);
        triplets.emplace_back(row_idx, this_control_idx, -1.0);
        upper_bound(row_idx) = 0.0;
        row_idx++;
        // a - s <= 0
        triplets.emplace_back(row_idx, s_slack_var_idx, -1.0);
        triplets.emplace_back(row_idx, this_control_idx, 1.0);
        upper_bound(row_idx) = 0.0;
        row_idx++;
      }

      double eps = 0.0;
      size_t F_max_ = mass * desired_accel_ - eps;
      // penalize_total_force ?  sum(s_ik)-zk <= F_max_ : sum(s_ik) <= F_max
      if (penalize_total_force) {  // sum(s_ik)-zk <= F_max
        size_t z_slack_var_idx = state_dim*N+control_dim*(N-1)+ii*num_force_norm_slack_vars_per_iter+4;
        for (size_t jj = 0; jj < control_dim_lin; jj++) {
          size_t s_slack_var_idx = state_dim*N+control_dim*(N-1)+ii*num_force_norm_slack_vars_per_iter+jj;
          triplets.emplace_back(row_idx, s_slack_var_idx, 1.0);
        }
        triplets.emplace_back(row_idx, z_slack_var_idx, -1.0);
        upper_bound(row_idx) = F_max_;
        row_idx++;

        // -zk <= 0
        triplets.emplace_back(row_idx, z_slack_var_idx, -1.0);
        upper_bound(row_idx) = 0;
        row_idx++;
      } else {  // sum(s_ik) <= F_max
        for (size_t jj = 0; jj < control_dim_lin; jj++) {
          size_t s_slack_var_idx = state_dim*N+control_dim*(N-1)+ii*num_force_norm_slack_vars_per_iter+jj;
          triplets.emplace_back(row_idx, s_slack_var_idx, 1.0);
        }
        upper_bound(row_idx) = F_max_;
        row_idx++;
      }
    }
  }

  if (enforce_moment_norm) {
    // Moment constraints
    for (size_t ii = 0; ii < N-1; ii++) {
      for (size_t jj = 0; jj < control_dim_nlin; jj++) {
        size_t s_slack_var_idx = state_dim*N+control_dim*(N-1)+num_force_norm_slack_vars+
        ii*num_moment_norm_slack_vars_per_iter+jj;
        size_t this_control_idx = state_dim*N+control_dim*ii+control_dim_lin+jj;
        // -s <=0
        triplets.emplace_back(row_idx, s_slack_var_idx, -1.0);
        upper_bound(row_idx) = 0.0;
        row_idx++;
        // -s - a <= 0
        triplets.emplace_back(row_idx, s_slack_var_idx, -1.0);
        triplets.emplace_back(row_idx, this_control_idx, -1.0);
        upper_bound(row_idx) = 0.0;
        row_idx++;
        // a - s <= 0
        triplets.emplace_back(row_idx, s_slack_var_idx, -1.0);
        triplets.emplace_back(row_idx, this_control_idx, 1.0);
        upper_bound(row_idx) = 0.0;
        row_idx++;
      }

      // pre-calculate M_max
      Vec3 alpha_;
      alpha_.setOnes();
      alpha_ *= desired_alpha_;
      Vec3 M_ = J*alpha_;
      decimal_t M_max_ = M_.minCoeff();
      // penalize_total_moment ?  sum(s_ik)-zk <= M_max : sum(s_ik) <= M_max
      if (penalize_total_moment) {  // sum(s_ik)-zk <= M_max
        size_t z_slack_var_idx = state_dim*N+control_dim*(N-1)+num_force_norm_slack_vars+
        ii*num_moment_norm_slack_vars_per_iter+4;
        for (size_t jj = 0; jj < control_dim_nlin; jj++) {
          size_t s_slack_var_idx = state_dim*N+control_dim*(N-1)+num_force_norm_slack_vars+
          ii*num_moment_norm_slack_vars_per_iter+jj;
          triplets.emplace_back(row_idx, s_slack_var_idx, 1.0);
        }
        triplets.emplace_back(row_idx, z_slack_var_idx, -1.0);
        upper_bound(row_idx) = M_max_;
        row_idx++;

        // -zk <= 0
        triplets.emplace_back(row_idx, z_slack_var_idx, -1.0);
        upper_bound(row_idx) = 0;
        row_idx++;

      } else {  // sum(s_ik) <= M_max
        for (size_t jj = 0; jj < control_dim_nlin; jj++) {
          size_t s_slack_var_idx = state_dim*N+control_dim*(N-1)+num_force_norm_slack_vars+
            ii*num_moment_norm_slack_vars_per_iter+jj;
          triplets.emplace_back(row_idx, s_slack_var_idx, 1.0);
        }
        upper_bound(row_idx) = M_max_;
        row_idx++;
      }
    }
  }

  if (enforce_state_LB) {
    // State LB
    for (size_t ii = 0; ii < N-1; ii++) {
      for (size_t jj = 0; jj < state_bd_dim; jj++) {
        size_t slack_var_idx = state_dim*N+control_dim*(N-1)+
          num_force_norm_slack_vars+num_moment_norm_slack_vars+
          state_bd_dim*ii+jj;
        // -z_ik <= 0
        triplets.emplace_back(row_idx, slack_var_idx, -1.0);
        upper_bound(row_idx) = 0;
        row_idx++;

        // -x_ik -z_ik <= -x_min(i) ignoring initial condition ii=0
        triplets.emplace_back(row_idx, slack_var_idx, -1.0);
        if (jj <= 2) {
          // position jj[0..2] maps to x_min[0...2]
          triplets.emplace_back(row_idx, state_dim*(ii+1)+jj, -1.0);
          upper_bound(row_idx) = -x_min(jj);
        } else {
          // quaternion jj[3...6] maps to x_min[6...9]
          triplets.emplace_back(row_idx, state_dim*(ii+1)+jj+3, -1.0);
          upper_bound(row_idx) = -x_min(jj+3);
        }
        row_idx++;
      }
    }
  }

  if (enforce_state_UB) {
    // State UB
    for (size_t ii = 0; ii < N-1; ii++) {
      for (size_t jj = 0; jj < state_bd_dim; jj++) {
        size_t slack_var_idx = state_dim*N+control_dim*(N-1)+
          num_force_norm_slack_vars+num_moment_norm_slack_vars+
          state_bd_dim*(N-1)+
          state_bd_dim*ii+jj;
        // -z_ik <= 0
        triplets.emplace_back(row_idx, slack_var_idx, -1.0);
        upper_bound(row_idx) = 0;
        row_idx++;

        // x_ik -z_ik <= x_max(i) ignoring initial condition ii=0
        triplets.emplace_back(row_idx, slack_var_idx, -1.0);
        if (jj <= 2) {
          // position jj[0..2] maps to x_min[0...2]
          triplets.emplace_back(row_idx, state_dim*(ii+1)+jj, 1.0);
          upper_bound(row_idx) = x_max(jj);
        } else {
          // quaternion jj[3...6] maps to x_min[6...9]
          triplets.emplace_back(row_idx, state_dim*(ii+1)+jj+3, 1.0);
          upper_bound(row_idx) = x_max(jj+3);
        }
        row_idx++;
      }
    }
  }

  if (enforce_lin_vel_norm) {
    // Lin velocity norm
    for (size_t ii = 0; ii < N-1; ii++) {
      size_t z_slack_var_idx = state_dim*N+control_dim*(N-1)+
        num_force_norm_slack_vars+num_moment_norm_slack_vars+
        state_bd_dim*(N-1)+
        state_bd_dim*(N-1) + 4*ii + 3;
      // -z_k <=0
      triplets.emplace_back(row_idx, z_slack_var_idx, -1.0);
      upper_bound(row_idx) = 0;
      row_idx++;
      for (size_t jj = 0; jj < lin_vel_dim; jj++) {
        size_t s_slack_var_idx = state_dim*N+control_dim*(N-1)+
          num_force_norm_slack_vars+num_moment_norm_slack_vars+
          state_bd_dim*(N-1)+
          state_bd_dim*(N-1) + 4*ii + jj;
        size_t this_state_idx = state_dim*(ii+1)+ 3 + jj;  // skip ii=0, skip 3 position states
        // -s_ik <= 0
        triplets.emplace_back(row_idx, s_slack_var_idx, -1.0);
        upper_bound(row_idx) = 0;
        row_idx++;
        // -x_ik -s_ik <= 0
        triplets.emplace_back(row_idx, this_state_idx, -1.0);
        triplets.emplace_back(row_idx, s_slack_var_idx, -1.0);
        upper_bound(row_idx) = 0;
        row_idx++;
        // x_ik -s_ik <= 0
        triplets.emplace_back(row_idx, this_state_idx, 1.0);
        triplets.emplace_back(row_idx, s_slack_var_idx, -1.0);
        upper_bound(row_idx) = 0;
        row_idx++;
      }
      // sum(s_ik) - zk <= v_max
      for (size_t jj = 0; jj < lin_vel_dim; jj++) {
        size_t s_slack_var_idx = state_dim*N+control_dim*(N-1)+
          num_force_norm_slack_vars+num_moment_norm_slack_vars+
          state_bd_dim*(N-1)+state_bd_dim*(N-1) +
          4*ii + jj;
        triplets.emplace_back(row_idx, s_slack_var_idx, 1.0);
      }
      triplets.emplace_back(row_idx, z_slack_var_idx, -1.0);
      upper_bound(row_idx) = desired_vel_;
      row_idx++;
    }
  }

  if (enforce_ang_vel_norm) {
    // Ang velocity norm
    for (size_t ii = 0; ii < N-1; ii++) {
      size_t z_slack_var_idx = state_dim*N+control_dim*(N-1)+
        num_force_norm_slack_vars+num_moment_norm_slack_vars+
        state_bd_dim*(N-1)+state_bd_dim*(N-1)+
        4*(N-1)+ 4*ii + 3;
      // -z_k <=0
      triplets.emplace_back(row_idx, z_slack_var_idx, -1.0);
      upper_bound(row_idx) = 0;
      row_idx++;
      for (size_t jj = 0; jj < ang_vel_dim; jj++) {
        size_t s_slack_var_idx = state_dim*N+control_dim*(N-1)+
          num_force_norm_slack_vars+num_moment_norm_slack_vars+
          state_bd_dim*(N-1)+state_bd_dim*(N-1)+
          4*(N-1)+ 4*ii + jj;
        size_t this_state_idx = state_dim*(ii+1)+ 10 + jj;  // skip ii=0, skip position,linvel,quat states
        // -s_ik <= 0
        triplets.emplace_back(row_idx, s_slack_var_idx, -1.0);
        upper_bound(row_idx) = 0;
        row_idx++;
        // -x_ik -s_ik <= 0
        triplets.emplace_back(row_idx, this_state_idx, -1.0);
        triplets.emplace_back(row_idx, s_slack_var_idx, -1.0);
        upper_bound(row_idx) = 0;
        row_idx++;
        // x_ik -s_ik <= 0
        triplets.emplace_back(row_idx, this_state_idx, 1.0);
        triplets.emplace_back(row_idx, s_slack_var_idx, -1.0);
        upper_bound(row_idx) = 0;
        row_idx++;
      }
      // sum(s_ik) - zk <= v_max
      for (size_t jj = 0; jj < ang_vel_dim; jj++) {
        size_t s_slack_var_idx = state_dim*N+control_dim*(N-1)+
          num_force_norm_slack_vars+num_moment_norm_slack_vars+state_bd_dim*(N-1)+state_bd_dim*(N-1)+
          4*(N-1) + 4*ii + jj;
        triplets.emplace_back(row_idx, s_slack_var_idx, 1.0);
      }
      triplets.emplace_back(row_idx, z_slack_var_idx, -1.0);
      upper_bound(row_idx) = desired_vel_;
      row_idx++;
    }
  }

  if (enforce_state_bounds || enforce_obs_avoidance_const) {
    if (enforce_state_bounds) {
      std::cout << "[TOP::SetSimpleConstraints] keep-in min pos: " << MinPos().transpose() << std::endl;
      std::cout << "[TOP::SetSimpleConstraints] keep-in max pos: " << MaxPos().transpose() << std::endl;
    }
    if (enforce_obs_avoidance_const) {
      if (keep_out_zones_.size() == 0) {
        std::cout << "ERROR: No keep-out zones provided for obstacle avoidance constraints. Ignoring." << std::endl;
      } else {
        std::cout << "[TOP::SetSimpleConstraints] keep-out box min: " << keep_out_zones_.back().min().transpose()
                  << std::endl;
        std::cout << "[TOP::SetSimpleConstraints] keep-out box max: " << keep_out_zones_.back().max().transpose()
                  << std::endl;
      }
    }
    for (size_t ii = 0; ii < N; ii++) {
      for (size_t jj = 0; jj < 3; jj++) {  // x, y, z only for now
        decimal_t lb = MinPos()[jj];
        decimal_t ub = MaxPos()[jj];

        if (enforce_obs_avoidance_const && (keep_out_zones_.size() > 0) && (!is_granite || (jj != 2))) {
          // Obstacle avoidance logic
          Eigen::AlignedBox3d box = keep_out_zones_.back();
          Eigen::Vector3d ko_min = box.min();
          Eigen::Vector3d ko_max = box.max();

          // Clip ko_min and ko_max to be within pose min and max
          ko_min = ko_min.cwiseMax(MinPos());  // clip ko_min to be >= pose min
          ko_max = ko_max.cwiseMin(MaxPos());  // clip ko_max to be <= pose max

          Eigen::Vector3d ko_center = (ko_min + ko_max) / 2;

          // Check if the current state needs an obstacle avoidance constraint
          bool active_proj = true;
          for (size_t kk = 0; kk < 3; kk++) {
            if (kk == jj) continue;
            if ((Xprev[ii](kk) > ko_max[kk]) || (Xprev[ii](kk) < ko_min[kk])) {
              active_proj = false;
              break;
            }
          }

          if (active_proj) {
            if (Xprev[ii](jj) >= ko_center[jj]) {
              lb = ko_max[jj];  // move lb to avoid obstacle
            } else {
              ub = ko_min[jj];  // move ub to avoid obstacle
            }
          }
        }

        // Add the combined constraint (whether it's from state bounds or obstacle avoidance)
        triplets.emplace_back(row_idx, state_dim * ii + jj, 1.0);
        lower_bound(row_idx) = lb;
        upper_bound(row_idx) = ub;
        row_idx++;
      }
    }
  }

  if (enforce_lin_vel_limit) {
    for (size_t ii = 0; ii < N; ii++) {
      for (size_t jj = 0; jj < lin_vel_dim; jj++) {
        triplets.emplace_back(row_idx, state_dim * ii + pos_dim + jj, 1.0);
        lower_bound(row_idx) = -lin_vel_limit;
        upper_bound(row_idx) = lin_vel_limit;
        row_idx++;
      }
    }
  }

  if (enforce_ang_vel_limit) {
    for (size_t ii = 0; ii < N; ii++) {
      for (size_t jj = 0; jj < ang_vel_dim; jj++) {
        triplets.emplace_back(row_idx, state_dim * ii + pos_dim + lin_vel_dim + quat_dim + jj, 1.0);
        lower_bound(row_idx) = -ang_vel_limit;
        upper_bound(row_idx) = ang_vel_limit;
        row_idx++;
      }
    }
  }


  // Update linear_con_mat all at once with triplets
  linear_con_mat.setFromTriplets(triplets.begin(), triplets.end());

  size_t num_vars = GetNumTOPVariables();
  size_t num_cons = GetNumTOPConstraints(/*verbose=*/ false);

  // Check that row_idx is equal to num_cons
  if (row_idx != num_cons) {
    std::cout << "row_idx: " << row_idx << " num_cons: " << num_cons << std::endl;
    throw std::runtime_error("row_idx is not equal to num_cons");
  }
  // Check that linear_con_mat is of size num_cons x num_vars
  if (linear_con_mat.rows() != num_cons || linear_con_mat.cols() != num_vars) {
    std::cout << "linear_con_mat.rows(): " << linear_con_mat.rows() << " num_cons: " << num_cons << std::endl;
    std::cout << "linear_con_mat.cols(): " << linear_con_mat.cols() << " num_vars: " << num_vars << std::endl;
    throw std::runtime_error("linear_con_mat is not of size num_cons x num_vars");
  }
  // Check that lower_bound and upper_bound are of size num_cons
  if (lower_bound.size() != num_cons || upper_bound.size() != num_cons) {
    std::cout << "lower_bound.size(): " << lower_bound.size() << " num_cons: " << num_cons << std::endl;
    std::cout << "upper_bound.size(): " << upper_bound.size() << " num_cons: " << num_cons << std::endl;
    throw std::runtime_error("lower_bound and upper_bound are not of size num_cons");
  }
  // Check that all lower bounds are less than or equal to upper bounds
  for (size_t ii = 0; ii < num_cons; ii++) {
    if (lower_bound(ii) > upper_bound(ii)) {
      std::cout << "lower_bound[" << ii << "]: " << lower_bound(ii) << " upper_bound[" << ii << "]: " << upper_bound(ii)
                << std::endl;
      throw std::runtime_error("lower_bound is greater than upper_bound");
    }
  }
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
  std::cout << "Finished setting simple constraints in : " << duration << " ms." << std::endl;
  if (save_constraints_to_file) {
    PrettyPrintConstraints();
  }
}

void TOP::PrettyPrintConstraints() {
  std::string timestamp = getCurrentTimestamp();
  std::string fname = output_dir + "/pretty_constraints" + "_" + timestamp + ".txt";
  CreateDirectoryIfNotExists(output_dir);
  std::ofstream outFile(fname);
  if (!outFile.is_open()) {
      std::cerr << "Error opening file " << fname << " for writing!" << std::endl;
      return;
  }
  std::cout << "[TOP::PrettyPrintConstraints] Printing constraints to file " << fname << std::endl;

  char full_path[PATH_MAX];
  if (realpath(fname.c_str(), full_path)) {
    std::cout << "Full path: " << full_path << std::endl;
  } else {
    std::cerr << "Error resolving path: " << full_path << " " << strerror(errno) << std::endl;
    throw std::runtime_error("Error resolving path: " + std::string(strerror(errno)));
  }


  size_t num_vars = GetNumTOPVariables();
  size_t num_cons = GetNumTOPConstraints(/*verbose=*/ false);
  size_t max_print = 15000;

  for (size_t cc = 0; cc < num_cons; cc++) {
    if (cc > max_print) {
      break;
    }
    // TODO(somrita): add label for constraint type
    outFile << "Constraint " << cc << ": ";
    outFile << lower_bound(cc) << " <= ";
    bool started_printing = false;
    for (size_t ii = 0; ii < num_vars; ii++) {
      if (linear_con_mat.coeff(cc, ii) != 0) {
        if (started_printing) {
          outFile << " + ";
        }
        if (linear_con_mat.coeff(cc, ii) == 1) {
          outFile << ConvertiiToString(ii) << " ";
          started_printing = true;
        } else if (linear_con_mat.coeff(cc, ii) == -1) {
          outFile << "-" << ConvertiiToString(ii) << " ";
          started_printing = true;
        } else {
          outFile << linear_con_mat.coeff(cc, ii) << " " << ConvertiiToString(ii) << " ";
          started_printing = true;
        }
      }
    }
    outFile << " <= " << upper_bound(cc) << std::endl;
  }

  outFile.close();
  return;
}

void TOP::CreateDirectoryIfNotExists(const std::string& path) {
  struct stat info;
  if (stat(path.c_str(), &info) != 0) {
      // Directory does not exist, attempt to create it
      if (mkdir(path.c_str(), 0755) == 0) {
          std::cout << "Directory created successfully: " << path << std::endl;
      } else {
          std::perror("Failed to create directory");
      }
  } else if (info.st_mode & S_IFDIR) {
      std::cout << "Directory already exists: " << path << std::endl;
  } else {
      std::cerr << "Path exists but is not a directory: " << path << std::endl;
  }
  return;
}

std::string TOP::ConvertiiToString(size_t ii) {
  size_t num_vars = GetNumTOPVariables();
  std::string var_name = "";
  if (ii < state_dim * N) {
    size_t ts = ii / state_dim;
    size_t jj = ii % state_dim;
    if (jj == 0) {
      var_name += "x";
    } else if (jj == 1) {
      var_name += "y";
    } else if (jj == 2) {
      var_name += "z";
    } else if (jj == 3) {
      var_name += "vx";
    } else if (jj == 4) {
      var_name += "vy";
    } else if (jj == 5) {
      var_name += "vz";
    } else if (jj == 6) {
      var_name += "qx";
    } else if (jj == 7) {
      var_name += "qy";
    } else if (jj == 8) {
      var_name += "qz";
    } else if (jj == 9) {
      var_name += "qw";
    } else if (jj == 10) {
      var_name += "wx";
    } else if (jj == 11) {
      var_name += "wy";
    } else if (jj == 12) {
      var_name += "wz";
    } else {
      std::cerr << "Unknown variable index: " << jj << std::endl;
    }
    var_name += " (t = " + std::to_string(ts) + ")";
    return var_name;
  } else if (ii < state_dim * N + control_dim * (N - 1)) {
    size_t ts = (ii - state_dim * N) / control_dim;
    size_t jj = (ii - state_dim * N) % control_dim;
    if (jj == 0) {
      var_name += "f1";
    } else if (jj == 1) {
      var_name += "f2";
    } else if (jj == 2) {
      var_name += "f3";
    } else if (jj == 3) {
      var_name += "m1";
    } else if (jj == 4) {
      var_name += "m2";
    } else if (jj == 5) {
      var_name += "m3";
    } else {
      std::cerr << "Unknown variable index: " << jj << std::endl;
    }
    var_name += " (t = " + std::to_string(ts) + ")";
    return var_name;
  } else {
    std::cerr << "ii is out of bounds: " << ii << " max variables = " << num_vars << std::endl;
  }
  return "";
}

void TOP::SetSimpleCosts() {
  // Weights for control effort minimization
  double force_weight = 1.0;
  double torque_weight = 1.0;
  std::vector<Eigen::Triplet<double>> hessian_triplets;
  // Penalize control inputs (u1, u2, u3)
  for (size_t ii = 0; ii < (N - 1); ++ii) {
    for (size_t jj = 0; jj < control_dim; ++jj) {
      size_t idx = N * state_dim + ii * control_dim + jj;
      double control_weight = (jj < control_dim_lin) ? force_weight : torque_weight;
      hessian_triplets.emplace_back(idx, idx, control_weight);
    }
  }
  hessian.setFromTriplets(hessian_triplets.begin(), hessian_triplets.end());
}

bool TOP::Solve() {
  std::string hdr = "TOP::Solve";
  printBoxHeader(hdr);
  solved_ = false;
  ResetSCPParams();
  UpdateProblemDimension(N);

  // TODO(somrita): Reset max_iter
  max_iter = 1;
  for (size_t kk = 0; kk < max_iter; kk++) {
    if (!solver->updateLinearConstraintsMatrix(linear_con_mat)) {
      solver_ready_ = false;
    } else if (!solver->updateGradient(gradient)) {
      solver_ready_ = false;
    } else if (!solver->updateHessianMatrix(hessian)) {
      solver_ready_ = false;
    } else if (!solver->updateBounds(lower_bound, upper_bound)) {
      solver_ready_ = false;
    } else if (!solver->setPrimalVariable(qp_soln)) {
      solver_ready_ = false;
    }

    solved_ = true;
    if (!solver_ready_) {
      solved_ = false;
      return false;
    } else if (!solver->solve()) {
      solved_ = false;
    }

    qp_soln = solver->getSolution();

    // if (!TrustRegionSatisfied()) {
    //   // Reject solution
    //   omega = gamma_fail*omega;
    //   continue;
    // }

    // decimal_t rho = AccuracyRatio();
    // if (rho > rho_1) {
    //   // Reject solution
    //   Delta = beta_fail * Delta;
    //   continue;
    // }

    // // Accept solution
    // if (rho < rho_0) {
    //   // Increase trust region region only if good quality solution
    //   Delta = std::min(beta_succ*Delta, Delta_0);
    // }

    // bool state_ineq_con_satisfied = SatisfiesStateInequalityConstraints();
    // if (state_ineq_con_satisfied) {
    //   omega = omega_0;
    // } else {
    //   // Increase penalty for state inequality constraints
    //   omega = std::min(gamma_fail*omega, omega_max);
    // }
    // if (omega > omega_max) {
    //   solved_ = false;
    //   return false;
    // }

    // if (ConvergenceMetric() < convergence_threshold) {
    //   solved_ = true;
    // }

    NormalizeQuaternions();

    // Update cached solution
    for (size_t ii = 0; ii < N; ii++) {
      Xprev[ii] = qp_soln.block(state_dim*ii, 0, state_dim, 1);
    }
    for (size_t ii = 0; ii < N-1; ii++) {
      Uprev[ii] = qp_soln.block(state_dim*N + control_dim*ii, 0, control_dim, 1);
    }

    ValidationChecks();

    if (solved_) {
      break;
    }
  }

  if (save_trajectory_to_file) {
    std::string fname = std::string((is_granite) ? "granite" : "iss") + "_optim_trajectory";
    WriteTrajectoryToFile(fname);
  }

  return solved_;
}

// Function to check if a sparse matrix is positive semidefinite (PSD)
bool TOP::IsPositiveSemidefinite(const SparseMatD& matrix) {
  if (matrix.rows() != matrix.cols()) {
    throw std::runtime_error("Matrix is not square.");
  }
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(matrix.toDense());
  return eigen_solver.eigenvalues().minCoeff() >= 0.0;
}

// Main validation logic
void TOP::ValidateQPProblem() {
  // Check if Hessian is square
  if (hessian.rows() != hessian.cols()) {
    throw std::runtime_error("Hessian matrix is not square.");
  }

  // Check if Hessian is positive semidefinite
  if (!IsPositiveSemidefinite(hessian)) {
    throw std::runtime_error("Hessian matrix is not positive semidefinite.");
  }

  // Check if bounds are valid
  if (!((lower_bound.array() <= upper_bound.array()).all())) {
    throw std::runtime_error("Bounds are invalid: lower_bound must be <= upper_bound.");
  }

  // Check if dimensions of the constraint matrix match the bounds
  if (!(linear_con_mat.rows() == lower_bound.size() && linear_con_mat.rows() == upper_bound.size())) {
    throw std::runtime_error("Linear constraints matrix dimensions do not match bounds.");
  }

  // Check if gradient vector matches the size of the Hessian
  if (hessian.rows() != gradient.size()) {
    throw std::runtime_error("Gradient vector size does not match the number of variables (Hessian dimension).");
  }

  std::cout << "All checks passed. The problem is well-posed and convex." << std::endl;
}

void TOP::ValidationChecks() {
  // Check maximum and minimum positions
  Vec3 min_pos = Xprev[0].segment(0, 3);
  Vec3 max_pos = Xprev[0].segment(0, 3);
  for (size_t ii = 0; ii < N; ii++) {
    Vec3 pos = Xprev[ii].segment(0, 3);
    min_pos = pos.cwiseMin(min_pos);
    max_pos = pos.cwiseMax(max_pos);
  }
  std::cout << "[TOP::ValidationChecks] Minimum position in solution: " << min_pos.transpose() << std::endl;
  std::cout << "[TOP::ValidationChecks] Maximum position in solution: " << max_pos.transpose() << std::endl;

  // Check boundary conditions
  decimal_t eps = 1e-5;
  Vec13 soln_x0 = Xprev[0];
  Vec13 soln_xg = Xprev[N-1];

  // Initial state
  decimal_t max_diff_x0 = (soln_x0 - x0).cwiseAbs().maxCoeff();
  std::string result_x0 = (max_diff_x0 < eps) ? "PASSED" : "FAILED";
  std::cout << "Initial state constraint:         " << result_x0 << "\tMax violation:" << max_diff_x0 << std::endl;

  // Goal state
  decimal_t max_diff_xg = (soln_xg - xg).cwiseAbs().maxCoeff();
  std::string result_xg = (max_diff_xg < eps) ? "PASSED" : "FAILED";
  std::cout << "Goal state constraint:            " << result_xg << "\tMax violation:" << max_diff_xg << std::endl;

  // Linear dynamics
  Vec6Vec lin_dyn_resid;
  lin_dyn_resid.resize(N-1);
  decimal_t max_diff_lin_dyn = 0.0;
  for (size_t ii = 0; ii < N-1; ii++) {
    Vec6 x = Xprev[ii].segment(0, state_dim_lin);
    Vec6 xp = Xprev[ii+1].segment(0, state_dim_lin);
    Vec3 u = Uprev[ii].segment(0, control_dim_lin);
    Eigen::Matrix<scp::decimal_t, 6, 6>  A;
    A << 1, 0, 0, dh, 0, 0,
         0, 1, 0, 0, dh, 0,
         0, 0, 1, 0, 0, dh,
         0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 1;
    Eigen::Matrix<scp::decimal_t, 6, 3> B;
    B << 0, 0, 0,
         0, 0, 0,
         0, 0, 0,
         dh/mass, 0, 0,
         0, dh/mass, 0,
         0, 0, dh/mass;
    lin_dyn_resid[ii] = A*x + B*u - xp;
    decimal_t max_ii = lin_dyn_resid[ii].cwiseAbs().maxCoeff();
    max_diff_lin_dyn = std::max(max_diff_lin_dyn, max_ii);
  }
  std::string result_lin_dyn = (max_diff_lin_dyn < eps) ? "PASSED" : "FAILED";
  std::cout << "Linear dynamics constraint:       " << result_lin_dyn
    << "\tMax violation:" << max_diff_lin_dyn << std::endl;

  // Nonlinear dynamics
  Vec7Vec nonlin_dyn_resid;
  nonlin_dyn_resid.resize(N-1);
  decimal_t max_diff_nonlin_dyn = 0.0;
  for (size_t ii = 0; ii < N-1; ii++) {
    Vec7 x = Xprev[ii].segment(state_dim_lin, state_dim_nlin);
    Vec7 xp = Xprev[ii+1].segment(state_dim_lin, state_dim_nlin);
    Vec3 u = Uprev[ii].segment(control_dim_lin, control_dim_nlin);
    Vec7 fk = fs[ii];
    Vec7 ck = dh*(As[ii]*x + Bs[ii]*u - fk);
    Mat7 eye;
    eye.setIdentity();
    nonlin_dyn_resid[ii] = (eye + dh*As[ii])*x + (dh*Bs[ii])*u - xp - ck;
    decimal_t max_ii = nonlin_dyn_resid[ii].cwiseAbs().maxCoeff();
    max_diff_nonlin_dyn = std::max(max_diff_nonlin_dyn, max_ii);
    bool print = false;
    if (print && max_ii > eps) {
      std::cout << "Violation of rotational dynamics at time " << ii << std::endl;
      std::cout << nonlin_dyn_resid[ii] << std::endl;
    }
  }
  std::string result_nonlin_dyn = (max_diff_nonlin_dyn < eps) ? "PASSED" : "FAILED";
  std::cout << "Nonlinear dynamics constraint:    " << result_nonlin_dyn
    << "\tMax violation:" << max_diff_nonlin_dyn << std::endl;

  // Force and moment constraints
  VecD force_resid;
  force_resid.resize(N-1);
  VecD moment_resid;
  moment_resid.resize(N-1);
  decimal_t max_resid_force = 0.0;  // stays 0 if all L1 norms of force are less than max force
  decimal_t max_resid_moment = 0.0;  // stays 0 if all L1 norms of moment are less than max moment
  // Pre-calculate max moment
  Vec3 alpha_;
  alpha_.setOnes();
  alpha_ *= desired_alpha_;
  Vec3 M_ = J*alpha_;
  decimal_t max_allowed_moment = M_.minCoeff();
  // At each time step, check if L1 norms violate max limits
  for (size_t ii = 0; ii < N-1; ii++) {
    // Check forces
    Vec3 u_lin = Uprev[ii].segment(0, control_dim_lin);
    decimal_t force_L1_norm = u_lin.cwiseAbs().sum();
    decimal_t force_resid_ii = 0.0;  // residual is 0 unless the L1 norm exceeds max force
    if (force_L1_norm > mass*desired_accel_) {
      force_resid_ii = std::abs(force_L1_norm - mass*desired_accel_);
    }
    force_resid[ii] = force_resid_ii;
    // Store max violation seen so far
    max_resid_force = std::max(max_resid_force, force_resid_ii);

    // Check moments
    Vec3 u_nlin = Uprev[ii].segment(control_dim_lin, control_dim_nlin);
    decimal_t moment_L1_norm = u_nlin.cwiseAbs().sum();
    decimal_t moment_resid_ii = 0.0;  // residual is 0 unless the L1 norm exceeds max moment
    if (moment_L1_norm > max_allowed_moment) {
      moment_resid_ii = std::abs(moment_L1_norm - max_allowed_moment);
    }
    moment_resid[ii] = moment_resid_ii;
    // Store max violation seen so far
    max_resid_moment = std::max(max_resid_moment, moment_resid_ii);
  }
  std::string result_force_cons = (max_resid_force == 0.0) ? "PASSED" : "FAILED";
  std::cout << "Force constraint:                 " << result_force_cons
    << "\tMax violation:" << max_resid_force << std::endl;
  std::string result_moment_cons = (max_resid_moment == 0.0) ? "PASSED" : "FAILED";
  std::cout << "Moment constraint:                " << result_moment_cons
    << "\tMax violation:" << max_resid_moment << std::endl;

  // Check state box constraints
  Vec7Vec state_box_resid;
  state_box_resid.resize(N);
  decimal_t max_state_box_resid = 0.0;
  for (size_t ii = 0; ii < N; ii++) {
    Vec3 pos = Xprev[ii].segment(0, 3);
    Vec3 pos_min = x_min.segment(0, 3);
    Vec3 pos_max = x_max.segment(0, 3);
    Vec4 quat = Xprev[ii].segment(6, 4);
    Vec4 quat_min = x_min.segment(6, 4);
    Vec4 quat_max = x_max.segment(6, 4);
    Vec7 state_box_resid_ii;
    state_box_resid_ii.setZero();  // residuals are 0 unless some state dimension falls outside box
    // Positions
    for (size_t jj = 0; jj < 3; jj++) {
      if (pos[jj] < pos_min[jj]) {
        state_box_resid_ii[jj] = pos[jj] - pos_min[jj];
      } else if (pos[jj] > pos_max[jj]) {
        state_box_resid_ii[jj] = pos[jj] - pos_max[jj];
      } else {
        state_box_resid_ii[jj] = 0.0;
      }
    }
    // Quaternions
    for (size_t jj = 0; jj < 4; jj++) {
      if (quat[jj] < quat_min[jj]) {
        state_box_resid_ii[3+jj] = quat[jj] - quat_min[jj];
      } else if (quat[jj] > quat_max[jj]) {
        state_box_resid_ii[3+jj] = quat[jj] - quat_max[jj];
      } else {
        state_box_resid_ii[3+jj] = 0.0;
      }
    }
    state_box_resid[ii] = state_box_resid_ii;
    max_state_box_resid = std::max(max_state_box_resid, state_box_resid_ii.cwiseAbs().maxCoeff());
  }
  std::string result_state_box = (max_state_box_resid == 0.0) ? "PASSED" : "FAILED";
  std::cout << "State box constraints:            " << result_state_box
    << "\tMax violation:" << max_state_box_resid << std::endl;

  // Check speed norm constraints
  VecD linvel_resid;
  linvel_resid.resize(N);
  VecD angvel_resid;
  angvel_resid.resize(N);
  decimal_t max_resid_linvel = 0.0;  // stays 0 if all L1 norms of lin vel are less than max lin vel
  decimal_t max_resid_angvel = 0.0;  // stays 0 if all L1 norms of angvel are less than max angvel
  // At each time step, check if L1 norms violate max limits
  for (size_t ii = 0; ii < N-1; ii++) {
    // Check linear velocities
    Vec3 v = Xprev[ii].segment(3, 3);
    decimal_t linvel_L1_norm = v.cwiseAbs().sum();
    decimal_t linvel_resid_ii = 0.0;  // residual is 0 unless the L1 norm exceeds max vel
    if (linvel_L1_norm > desired_vel_) {
      linvel_resid_ii = linvel_L1_norm - desired_vel_;
    }
    linvel_resid[ii] = linvel_resid_ii;
    // Store max violation seen so far
    max_resid_linvel = std::max(max_resid_linvel, linvel_resid_ii);

    // Check ang velocities
    Vec3 omega = Xprev[ii].segment(10, 3);
    decimal_t angvel_L1_norm = omega.cwiseAbs().sum();
    decimal_t angvel_resid_ii = 0.0;  // residual is 0 unless the L1 norm exceeds max omega
    if (angvel_L1_norm > desired_omega_) {
      angvel_resid_ii = std::abs(angvel_L1_norm - desired_omega_);
    }
    angvel_resid[ii] = angvel_resid_ii;
    // Store max violation seen so far
    max_resid_angvel = std::max(max_resid_angvel, angvel_resid_ii);
  }
  std::string result_linvel_cons = (max_resid_linvel == 0.0) ? "PASSED" : "FAILED";
  std::cout << "Lin vel constraint:               " << result_linvel_cons
    << "\tMax violation:" << max_resid_linvel << std::endl;
  std::string result_angvel_cons = (max_resid_angvel == 0.0) ? "PASSED" : "FAILED";
  std::cout << "Ang vel constraint:               " << result_angvel_cons
    << "\tMax violation:" << max_resid_angvel << std::endl;

  // // Check obstacle avoidance constraints
  if ((keep_out_zones_.size() == 0) || (!enforce_obs_avoidance_const)) {
    std::cout << "Obstacle avoidance constraint:    NOT CHECKED" << std::endl;
  } else {
    Eigen::AlignedBox3d box = keep_out_zones_.back();
    Eigen::Vector3d ko_min = box.min();
    Eigen::Vector3d ko_max = box.max();
    bool violated = false;
    for (size_t ii = 0; ii < N-1; ii++) {
      Eigen::Vector3d pos = Xprev[ii].segment(0, 3);
      if ((pos.array() >= ko_min.array()).all() && (pos.array() <= ko_max.array()).all()) {
        violated = true;
        std::cout << "Obstacle avoidance constraint:    FAILED"
                  << "\t at t=" << ii << ", pos=" << pos.transpose() << std::endl;
        break;
      }
    }
    if (!violated) {
      std::cout << "Obstacle avoidance constraint:    PASSED" << std::endl;
    }
  }

  // TODO(somrita): Add checks for trust region
  std::cout << std::endl;
}

void TOP::NormalizeQuaternions() {
  // re-normalize quaternions
  for (size_t ii = 0; ii < N; ii++) {
    decimal_t q_norm = Xprev[ii].segment(6, 4).norm();
    if (q_norm > 1e-6) {
      Xprev[ii].segment(6, 4) /= q_norm;
    }
  }
}

void TOP::PolishSolution() {
  if (!solved_) {
    return;
  }
  // NormalizeQuaternions();
}

void TOP::WriteTrajectoryToFile(const std::string& fname, bool include_timestamp) {
  CreateDirectoryIfNotExists(output_dir);
  std::string full_fname;
  if (nn_training_mode) {
    CreateDirectoryIfNotExists(output_dir + "/nn_training");
    full_fname = output_dir + "/nn_training/" + fname + ".txt";
  } else if (include_timestamp) {
    std::string timestamp = getCurrentTimestamp();
    full_fname = output_dir + "/" + fname + "_" + timestamp + ".txt";
  } else {
    full_fname = output_dir + "/" + fname + ".txt";
  }
  std::ofstream file(full_fname);
  char full_path[PATH_MAX];
  if (!file.is_open()) {
    std::cerr << "Error: Unable to open file " << full_fname << " for writing trajectory." << std::endl;
    return;
  }
  if (!(realpath(full_fname.c_str(), full_path))) {
    std::cerr << "Error resolving path: " << full_fname << " " << strerror(errno) << std::endl;
    return;
  }
  std::cout << "[TOP::WriteTrajectoryToFile] Writing trajectory to: " << full_fname << std::endl;
  std::cout << "Full path: " << full_path << std::endl;

  // Solved status
  file << "Solved: " << solved_ << std::endl;

  // Write the initial state (x0) and goal state (xg), each of length 13
  for (int i = 0; i < 13; ++i) {
    file << x0[i] << " ";
  }
  file << std::endl;

  for (int i = 0; i < 13; ++i) {
    file << xg[i] << " ";
  }
  file << std::endl;

  // Write the number of time steps (N)
  file << N << std::endl;

  // Write the Xprev data (N lines of length 13)
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j < 13; ++j) {
      file << Xprev[i][j] << " ";
    }
    file << std::endl;
  }

  // Write the Uprev data (N-1 lines of length 6)
  for (int i = 0; i < N - 1; ++i) {
    for (int j = 0; j < 6; ++j) {
      file << Uprev[i][j] << " ";
    }
    file << std::endl;
  }

  // Close the file stream
  file.close();
  std::cout << "Trajectory data writing complete." << std::endl;
}

Vec13 TOP::ForwardDynamics(Vec13 x, Vec6 u) {
  Vec13 xp = Vec13::Zero();
  // Extract position, velocity, and quaternion from x
  Vec3 pos = x.segment(0, 3);
  Vec3 vel = x.segment(3, 3);
  Vec4 quat = x.segment(6, 4);
  Vec3 ang_vel = x.segment(10, 3);
  // Extract force and torque from u
  Vec3 force = u.segment(0, 3);
  Vec3 torque = u.segment(3, 3);
  // Position update: x_{i+1} = x_i + v_i * dt
  xp.segment(0, 3) = pos + vel * dh;

  // Velocity update: v_{i+1} = v_i + u_j * dt / mass
  xp.segment(3, 3) = vel + force * dh / mass;

  // // Quaternion update: q_{i+1} = q_i + 0.5 * Q(q_i) * omega_i * dt
  // Eigen::Matrix<double, 4, 3> QMat = CalculateQMat(quat);
  // xp.segment(6, 4) = quat + 0.5 * quat * QMat * ang_vel * dh;
  xp.segment(6, 4) = quat;  // Placeholder: no update

  // // Angular velocity update: omega_{i+1} = omega_i + J^{-1} * (u_torque - omega_i cross (J * omega_i)) * dt
  // Eigen::Vector3d frot_mat = J.inverse() * (torque - ang_vel.cross(J * ang_vel));
  // xp.segment(10, 3) = ang_vel + frot_mat * dh;
  xp.segment(10, 3) = ang_vel;  // Placeholder: no update
  return xp;
}

std::tuple<Vec6, Vec6> TOP::InferenceNN(Vec13 x0, Vec13 xg) {
  std::cout << "[TOP::InferenceNN]" << std::endl;
  // Create input of length 26 from x0 and xg
  torch::Tensor input = torch::zeros({1, 26});
  for (size_t i = 0; i < 13; ++i) {
    input[0][i] = x0[i];
    input[0][i + 13] = xg[i];
  }
  std::cout << "[TOP::InferenceNN] Input tensor: " << input << std::endl;
  // Perform inference
  net->eval();
  torch::Tensor output = net->forward(input);
  std::cout << "[TOP::InferenceNN] Output tensor: " << output << std::endl;
  // Extract U0 and Uf from output
  Vec6 U0, Uf;
  for (size_t i = 0; i < 6; ++i) {
    U0[i] = output[0][i].item<float>();
    Uf[i] = output[0][i + 6].item<float>();
  }
  return std::make_tuple(U0, Uf);
}

std::tuple<Vec4, Vec4, Vec4> TOP::InferenceNNSpline(Vec13 x0, Vec13 xg) {
  std::cout << "[TOP::InferenceNNSpline]" << std::endl;

  std::cout << "[TOP::InferenceNNSpline] x0: " << x0.head(3).transpose() << std::endl;
  std::cout << "[TOP::InferenceNNSpline] xg: " << xg.head(3).transpose() << std::endl;

  // Create input tensor of shape {1,6} from the first three coordinates of x0 and xg.
  torch::Tensor input = torch::zeros({1, 6});
  for (size_t i = 0; i < 3; ++i) {
    input[0][i] = x0[i];
    input[0][i + 3] = xg[i];
  }
  std::cout << "[TOP::InferenceNNSpline] Input tensor: " << input << std::endl;

  // Perform inference
  spline_net->eval();
  torch::Tensor output = spline_net->forward(input);
  std::cout << "[TOP::InferenceNNSpline] Output tensor: " << output << std::endl;

  // Extract spline coefficients from output (assumed shape {1,12}) and split them into three Vec4:
  Vec4 coeff_x, coeff_y, coeff_z;
  for (size_t i = 0; i < 4; ++i) {
    coeff_x(i) = output[0][i].item<decimal_t>();
    coeff_y(i) = output[0][i + 4].item<decimal_t>();
    coeff_z(i) = output[0][i + 8].item<decimal_t>();
  }

  // Reconstruct x0 and xg from the spline coefficients (for accuracy check)
  Vec3 x0_reconstructed, xg_reconstructed;
  x0_reconstructed << coeff_x(0), coeff_y(0), coeff_z(0);
  xg_reconstructed << coeff_x(0) + coeff_x(1) + coeff_x(2) + coeff_x(3),
    coeff_y(0) + coeff_y(1) + coeff_y(2) + coeff_y(3), coeff_z(0) + coeff_z(1) + coeff_z(2) + coeff_z(3);
  std::cout << "[TOP::InferenceNNSpline] x0_reconstructed pose: " << x0_reconstructed.transpose() << std::endl;
  std::cout << "[TOP::InferenceNNSpline] xg_reconstructed pose: " << xg_reconstructed.transpose() << std::endl;

  return std::make_tuple(coeff_x, coeff_y, coeff_z);
}

/* Function to warm start from neural network */
std::tuple<Vec13Vec, Vec6Vec> TOP::WarmStartFromNN(Vec13 x0, Vec13 xg) {
  std::cout << "[TOP::WarmStartFromNN]" << std::endl;
  bool simplify = false;
  if (simplify) {
    // Simple case: just set to initial and final states
    Vec13Vec Xprev;
    Vec6Vec Uprev;
    Xprev.push_back(x0);
    Xprev.push_back(xg);
    Uprev.push_back(Vec6::Zero());
    Uprev.push_back(Vec6::Zero());
    return std::make_tuple(Xprev, Uprev);
  }
  if (nn_spline_mode) {
    // Get the spline coefficients from inference.
    Vec4 coeff_x, coeff_y, coeff_z;
    std::tie(coeff_x, coeff_y, coeff_z) = InferenceNNSpline(x0, xg);

    // Create Xprev using the spline for x, y, and z and linear interpolation for the remaining state.
    Vec13Vec X_inter;
    for (size_t i = 0; i < N; ++i) {
      double t = (N > 1) ? static_cast<double>(i) / (N - 1) : 0.0;
      Vec13 X;

      // Evaluate cubic splines for x, y, and z.
      X(0) = coeff_x(0) + coeff_x(1) * t + coeff_x(2) * t * t + coeff_x(3) * t * t * t;
      X(1) = coeff_y(0) + coeff_y(1) * t + coeff_y(2) * t * t + coeff_y(3) * t * t * t;
      X(2) = coeff_z(0) + coeff_z(1) * t + coeff_z(2) * t * t + coeff_z(3) * t * t * t;

      // For remaining state indices (3 to 12), linearly interpolate between x0 and xg.
      for (int j = 3; j < 13; ++j) {
        X(j) = x0(j) + (xg(j) - x0(j)) * t;
      }
      X_inter.push_back(X);
    }
    Xprev = X_inter;

    // Initialize Uprev to all zeros.
    Vec6Vec U_inter(N, Vec6::Zero());
    Uprev = U_inter;
  } else {
    // Call InferenceNN(x0, xg) to get U0, Uf
    Vec6 U0, Uf;
    std::tie(U0, Uf) = InferenceNN(x0, xg);
    // Interpolate linearly for N steps to get Uprev
    Vec6Vec Uprev;
    for (size_t i = 0; i < N; ++i) {
      Vec6 U = U0 + (i/(N-1))*(Uf - U0);
      Uprev.push_back(U);
    }
    // Use dynamics to get Xprev
    Vec13Vec Xprev;
    Xprev.push_back(x0);
    for (size_t i = 0; i < N; ++i) {
      Vec13 X = ForwardDynamics(Xprev[i], Uprev[i]);
      Xprev.push_back(X);
    }
  }
  return std::make_tuple(Xprev, Uprev);
}

std::tuple<torch::Tensor, torch::Tensor> TOP::ReadData(const std::string& filename) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    throw std::runtime_error("Unable to open file: " + filename);
  }

  std::string line;
  Vec13 x0, xg;
  Vec13Vec Xprev;
  Vec6Vec Uprev;
  int N;

  // Read x0
  std::getline(file, line);
  std::istringstream iss(line);
  for (int i = 0; i < 13; ++i) {
    iss >> x0[i];
  }

  // Read xg
  std::getline(file, line);
  iss.clear();
  iss.str(line);
  for (int i = 0; i < 13; ++i) {
    iss >> xg[i];
  }

  // Read N
  std::getline(file, line);
  N = std::stoi(line);

  // Read Xprev (N lines, each 13 values) and skip
  for (int i = 0; i < N; ++i) {
    std::getline(file, line);
    std::istringstream iss(line);
    Vec13 vec;
    float value;
    for (int j = 0; j < 13; ++j) {
      iss >> vec[j];
    }
    Xprev.push_back(vec);
  }

  // Read Uprev (N-1 lines, each 6 values)
  for (int i = 0; i < N - 1; ++i) {
    std::getline(file, line);
    std::istringstream iss(line);
    Vec6 vec;
    float value;
    for (int j = 0; j < 6; ++j) {
      iss >> vec[j];
    }
    Uprev.push_back(vec);
  }

  // Create input and output tensors
  std::vector<float> input_vector(x0.data(), x0.data() + x0.size());
  input_vector.insert(input_vector.end(), xg.data(), xg.data() + xg.size());

  std::vector<float> output_vector(Uprev[0].data(), Uprev[0].data() + Uprev[0].size());
  output_vector.insert(output_vector.end(), Uprev[N - 2].data(), Uprev[N - 2].data() + Uprev[N - 2].size());

  torch::Tensor input_tensor = torch::from_blob(input_vector.data(), {1, 26}).clone();
  torch::Tensor output_tensor = torch::from_blob(output_vector.data(), {1, 12}).clone();

  return std::make_tuple(input_tensor, output_tensor);
}

std::tuple<torch::Tensor, torch::Tensor> TOP::ReadDataSpline(const std::string& filename) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    throw std::runtime_error("Unable to open file: " + filename);
  }

  std::string line;
  // x0 and xg are now Eigen column vectors of size 13.
  scp::Vec13 x0, xg;
  scp::Vec13Vec Xprev;
  int N;

  // First line: Solved status
  std::getline(file, line);
  std::string solved = line.substr(8);

  // Read x0 (next line)
  std::getline(file, line);
  std::istringstream iss(line);
  for (int i = 0; i < 13; ++i) {
    iss >> x0(i);
  }

  // Read xg (next line)
  std::getline(file, line);
  iss.clear();
  iss.str(line);
  for (int i = 0; i < 13; ++i) {
    iss >> xg(i);
  }

  // Read N (next line)
  std::getline(file, line);
  N = std::stoi(line);

  // Read Xprev: N lines, each with 13 values
  for (int i = 0; i < N; ++i) {
    std::getline(file, line);
    std::istringstream iss_line(line);
    scp::Vec13 vec;
    for (int j = 0; j < 13; ++j) {
      iss_line >> vec(j);
    }
    Xprev.push_back(vec);
  }
  file.close();

  // ---------------------------
  // Create input tensor (length 6) using only the first three coordinates from x0 and xg.
  std::vector<float> input_vector;
  for (int i = 0; i < 3; ++i) {
    input_vector.push_back(static_cast<float>(x0(i)));
  }
  for (int i = 0; i < 3; ++i) {
    input_vector.push_back(static_cast<float>(xg(i)));
  }
  torch::Tensor input_tensor = torch::from_blob(input_vector.data(), {1, 6}).clone();

  // ---------------------------
  // Fit cubic polynomials for x(t), y(t), and z(t)
  // t is evenly spaced in [0,1]: t_i = i/(N-1)
  Eigen::MatrixXd A(N, 4);
  Eigen::VectorXd bx(N), by(N), bz(N);
  for (int i = 0; i < N; ++i) {
    double t = (N > 1) ? static_cast<double>(i) / (N - 1) : 0.0;
    A(i, 0) = 1.0;
    A(i, 1) = t;
    A(i, 2) = t * t;
    A(i, 3) = t * t * t;
    // Extract x, y, z from Xprev (columns 0, 1, 2)
    bx(i) = static_cast<double>(Xprev[i](0));
    by(i) = static_cast<double>(Xprev[i](1));
    bz(i) = static_cast<double>(Xprev[i](2));
  }

  // Solve for coefficients using least-squares: c = (AᵀA)⁻¹ Aᵀb
  Eigen::Vector4d coeff_x = (A.transpose() * A).ldlt().solve(A.transpose() * bx);
  Eigen::Vector4d coeff_y = (A.transpose() * A).ldlt().solve(A.transpose() * by);
  Eigen::Vector4d coeff_z = (A.transpose() * A).ldlt().solve(A.transpose() * bz);

  // Pack coefficients into a single vector: first x, then y, then z coefficients.
  std::vector<float> output_vector;
  for (int i = 0; i < 4; ++i) {
    output_vector.push_back(static_cast<float>(coeff_x(i)));
  }
  for (int i = 0; i < 4; ++i) {
    output_vector.push_back(static_cast<float>(coeff_y(i)));
  }
  for (int i = 0; i < 4; ++i) {
    output_vector.push_back(static_cast<float>(coeff_z(i)));
  }
  // output_vector now has length 12

  torch::Tensor output_tensor = torch::from_blob(output_vector.data(), {1, 12}).clone();

  return std::make_tuple(input_tensor, output_tensor);
}

void TOP::TrainModel(const std::vector<std::string>& files, int epochs) {
  std::cout << "Training model..." << std::endl;
  std::vector<torch::Tensor> inputs, outputs;

  // Read all data files
  for (const std::string& file : files) {
    std::tuple<torch::Tensor, torch::Tensor> data;
    if (nn_spline_mode) {
      data = ReadDataSpline(file);
    } else {
      data = ReadData(file);
    }
    inputs.push_back(std::get<0>(data));
    outputs.push_back(std::get<1>(data));
  }

  std::cout << "Data loaded. Size of inputs: " << inputs.size() << ", size of outputs: " << outputs.size() << std::endl;

  // Concatenate tensors for batch training
  torch::Tensor input_tensor = torch::cat(inputs, 0);
  torch::Tensor output_tensor = torch::cat(outputs, 0);

  // Training loop
  for (int epoch = 0; epoch < epochs; ++epoch) {
    if (nn_spline_mode) {
      spline_net->train();
      spline_optimizer.zero_grad();
      torch::Tensor predictions = spline_net->forward(input_tensor);
      torch::Tensor loss = torch::mse_loss(predictions, output_tensor);
      loss.backward();
      spline_optimizer.step();
      std::cout << "Epoch [" << epoch + 1 << "/" << epochs << "], Loss: " << loss.item<float>() << std::endl;
    } else {
      net->train();
      optimizer.zero_grad();
      torch::Tensor predictions = net->forward(input_tensor);
      torch::Tensor loss = torch::mse_loss(predictions, output_tensor);
      loss.backward();
      optimizer.step();
      std::cout << "Epoch [" << epoch + 1 << "/" << epochs << "], Loss: " << loss.item<float>() << std::endl;
    }
  }
}

void TOP::SaveModel(const std::string& model_path) {
  if (nn_spline_mode) {
    torch::save(spline_net, model_path);
  } else {
    torch::save(net, model_path);
  }
  std::cout << "Model saved to " << model_path << std::endl;
}

void TOP::LoadModel(const std::string& model_path) {
  std::cout << "[TOP::LoadModel] Attempting to load in NN model from " << model_path << std::endl;
  char full_path[PATH_MAX];
  if (realpath(model_path.c_str(), full_path)) {
    std::cout << "Full path: " << full_path << std::endl;
  } else {
    std::cerr << "Error resolving path: " << strerror(errno) << std::endl;
    throw std::runtime_error("Error resolving path: " + std::string(strerror(errno)));
  }
  if (nn_spline_mode) {
    torch::load(spline_net, model_path);
  } else {
    torch::load(net, model_path);
  }
  std::cout << "[TOP::LoadModel] Model successfully loaded from " << model_path << std::endl;
}

std::string TOP::getCurrentTimestamp() {
  // Get the current time as a time_point
  auto now = std::chrono::system_clock::now();

  // Convert it to a time_t to work with std::strftime
  std::time_t now_time = std::chrono::system_clock::to_time_t(now);

  // Get the milliseconds part
  auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

  // Convert to a string with a specific format (e.g., YYYY-MM-DD_HH-MM-SS_mmm)
  std::stringstream ss;
  ss << std::put_time(std::localtime(&now_time), "%Y-%m-%d_%H-%M-%S") << "_" << std::setw(3) << std::setfill('0')
     << milliseconds.count();  // Add milliseconds

  return ss.str();
}

}  //  namespace scp

// Generalized function to set all elements in a vector to zero
template <typename VecType>
void clearToZeros(std::vector<VecType, Eigen::aligned_allocator<VecType>>& vec) {
  for (auto& elem : vec) {
    elem.setZero();  // Set each element to zero
  }
}



// Function to initialize motion cases
std::tuple<scp::Vec13Vec, scp::Vec13Vec> initializeMotionCases(bool is_granite, bool nn_training_mode = false) {
  scp::Vec13Vec x0s;
  scp::Vec13Vec xgs;

  scp::Vec13 x0;
  scp::Vec13 xg;

  std::string case_mode = "with_obstacle";  // "nearby" or "single_x0" or "multiple_x0" or "with_obstacle"

  if (nn_training_mode) {
    if (is_granite) {
      throw std::runtime_error("Granite case not supported for NN training.");
      return std::make_tuple(x0s, xgs);
    } else {
      if (case_mode == "with_obstacle") {
        // x0 is 10.8 -9.5 4.8 0 0 0 0 0 0 1 0 0 0
        x0 << 10.8, -9.5, 4.8, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
        scp::Vec3 pos_min_(10.8, -9.4, 4.8);
        scp::Vec3 pos_max_(11.0, -8.9, 5.0);
        scp::decimal_t spacing = 0.1;
        // For any point in this grid (given spacing), generate a trajectory to the goal
        for (float x = pos_min_(0); x <= pos_max_(0); x += spacing) {
          for (float y = pos_min_(1); y <= pos_max_(1); y += spacing) {
            for (float z = pos_min_(2); z <= pos_max_(2); z += spacing) {
              x0 << x, y, z, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
              for (float a = pos_min_(0); a <= pos_max_(0); a += spacing) {
                for (float b = pos_min_(1); b <= pos_max_(1); b += spacing) {
                  for (float c = pos_min_(2); c <= pos_max_(2); c += spacing) {
                    xg << a, b, c, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
                    if (x0 != xg) {
                      x0s.push_back(x0);
                      xgs.push_back(xg);
                    }
                  }
                }
              }
            }
          }
        }
        return std::make_tuple(x0s, xgs);
      } else if (case_mode == "single_x0") {
        // Simple cases for ISS
        // x0 is 10.28, -9.81, 4.30, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
        x0 << 10.28, -9.81, 4.30, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
        // Cases with various motion in Y
        for (float dy = 0.1; dy <= 1.0; dy += 0.1) {
          xg << 10.28, -9.81 + dy, 4.30, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
          xgs.push_back(xg);
        }
        // Cases with various motion in X
        for (float dx = 0.1; dx <= 1.0; dx += 0.1) {
          xg << 10.28 + dx, -9.81, 4.30, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
          xgs.push_back(xg);
        }
        // Cases with various motion in Z
        for (float dz = 0.1; dz <= 1.0; dz += 0.1) {
          xg << 10.28, -9.81, 4.30 + dz, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
          xgs.push_back(xg);
        }
        for (size_t i = 0; i < xgs.size(); ++i) {
          x0s.push_back(x0);
        }
        return std::make_tuple(x0s, xgs);
      } else if (case_mode == "nearby") {
        std::cout << "Initializing nearby cases for ISS." << std::endl;
        // Nearby cases for ISS
        // x0 is 10.8, -9.5, 4.8, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
        float xi = 10.8;
        float yi = -9.5;
        float zi = 4.8;
        x0 << xi, yi, zi, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
        // Cases with motion within ISS bounds
        for (float xf = 10.8; xf <= 11.0; xf += 0.1) {
          for (float yf = -9.5; yf <= -9.4; yf += 0.1) {
            for (float zf = 4.3; zf <= 4.6; zf += 0.1) {
              xg << xf, yf, zf, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
              xgs.push_back(xg);
            }
          }
        }
        for (size_t i = 0; i < xgs.size(); ++i) {
          x0s.push_back(x0);
        }
        return std::make_tuple(x0s, xgs);
      } else if (case_mode == "multiple_x0") {
        scp::Vec3 pos_min_(10.28, -9.81, 4.30);
        scp::Vec3 pos_max_(11.28, -8.81, 5.30);
        scp::decimal_t spacing = 0.2;
        // For any point in this grid (0.1m spacing), generate a trajectory to the goal
        for (float x = pos_min_(0); x <= pos_max_(0); x += spacing) {
          for (float y = pos_min_(1); y <= pos_max_(1); y += spacing) {
            for (float z = pos_min_(2); z <= pos_max_(2); z += spacing) {
              x0 << x, y, z, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
              for (float a = pos_min_(0); a <= pos_max_(0); a += spacing) {
                for (float b = pos_min_(1); b <= pos_max_(1); b += spacing) {
                  for (float c = pos_min_(2); c <= pos_max_(2); c += spacing) {
                    xg << a, b, c, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
                    if (x0 != xg) {
                      x0s.push_back(x0);
                      xgs.push_back(xg);
                    }
                  }
                }
              }
            }
          }
        }
        return std::make_tuple(x0s, xgs);
      }
    }
  }

  if (is_granite) {
    // All x0s are the same
    x0 << -0.4, 0.4, -0.67, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;

    // Case 2: Motion in Y
    xg << -0.4, -0.4, -0.67, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    xgs.push_back(xg);

    // Case 1: Motion in X
    xg << 0.4, 0.4, -0.67, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    xgs.push_back(xg);

    // Case 3: Motion in XY
    xg << 0.4, -0.4, -0.67, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    xgs.push_back(xg);

    // Case 4: Asymmetric motion in XY
    xg << 0.5, -0.3, -0.67, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    xgs.push_back(xg);

    // Case 5: Rotation in place CCW about Z
    // (angle-axis) (-1.57 0 0 1) --> Quat x y z w (0 0 -0.7068252 0.7073883)
    xg << -0.4, 0.4, -0.67, 0, 0, 0, 0, 0, -0.7068252, 0.7073883, 0, 0, 0;
    xgs.push_back(xg);

    // Case 6: Rotation in place CW about Z
    // (angle-axis) (1.57 0 0 1) --> Quat x y z w (0 0 0.7068252 0.7073883)
    xg << -0.4, 0.4, -0.67, 0, 0, 0, 0, 0, 0.7068252, 0.7073883, 0, 0, 0;
    xgs.push_back(xg);

    // // Case 7: Rotation of 180 deg in place about Z
    // (angle-axis) (3.13 0 0 1) --> Quat x y z w (0 0 0.9999832, 0.0057963)
    xg << -0.4, 0.4, -0.67, 0, 0, 0, 0, 0, 0.9999832, 0.0057963, 0, 0, 0;
    xgs.push_back(xg);

    // Case 8: Rotation + translation in Y
    xg << -0.4, -0.4, -0.67, 0, 0, 0, 0, 0, 0.7068252, 0.7073883, 0, 0, 0;
    xgs.push_back(xg);

    // Case 9: Rotation + translation in X
    xg << 0.4, 0.4, -0.67, 0, 0, 0, 0, 0, 0.7068252, 0.7073883, 0, 0, 0;
    xgs.push_back(xg);

    // Case 10: Rotation + translation in XY
    xg << 0.4, -0.4, -0.67, 0, 0, 0, 0, 0, 0.7068252, 0.7073883, 0, 0, 0;
    xgs.push_back(xg);

    // Case 11: Rotation + translation asymmetric motion in XY
    xg << 0.5, -0.3, -0.67, 0, 0, 0, 0, 0, 0.7068252, 0.7073883, 0, 0, 0;
    xgs.push_back(xg);

    for (size_t i = 0; i < xgs.size(); ++i) {
      x0s.push_back(x0);
    }
  } else {
    // All x0s are the same
    x0 << 10.28, -9.81, 4.30, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;

    // Case 1: Motion in Y
    xg << 10.28, -8.81, 4.30, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    xgs.push_back(xg);
    // Case 2: Rotation in place
    // (angle-axis) (1.57 0 0 1) --> Quat x y z w (0 0 0.7068252 0.7073883)
    xg << 10.28, -9.81, 4.30, 0, 0, 0, 0, 0, 0.7068252, 0.7073883, 0, 0, 0;
    xgs.push_back(xg);
    // Case 3: Translation in 3 axes
    xg << 11.00, -8.81, 5.30, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    xgs.push_back(xg);
    // Case 4: Translation + rotation
    xg << 10.28, -8.81, 4.30, 0, 0, 0, 0, 0, 0.7068252, 0.7073883, 0, 0, 0;
    xgs.push_back(xg);
    // Case 5: Translation in 2 axes + rotation
    xg << 11.00, -8.81, 4.30, 0, 0, 0, 0, 0, 0.7068252, 0.7073883, 0, 0, 0;
    xgs.push_back(xg);
    // Case 6: Translation in 3 axes + rotation
    xg << 11.00, -8.81, 5.30, 0, 0, 0, 0, 0, 0.7068252, 0.7073883, 0, 0, 0;
    xgs.push_back(xg);
    // Case 7: Motion in YZ
    xg << 10.28, -8.81, 5.30, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    xgs.push_back(xg);

    for (size_t i = 0; i < xgs.size(); ++i) {
      x0s.push_back(x0);
    }
  }

  return std::make_tuple(x0s, xgs);
}

// Function to process a single problem instance
bool processProblemInstance(scp::TOP& top_eg, const scp::Vec13& x0, const scp::Vec13& xg,
                            const Eigen::AlignedBox3d& vbox, int problemIndex) {
  top_eg.x0 = x0;
  top_eg.xg = xg;
  // clearToZeros(top_eg.Xprev);
  // clearToZeros(top_eg.Uprev);
  top_eg.save_constraints_to_file = false;
  top_eg.save_trajectory_to_file = false;

  std::cout << "Checking for vbox..." << std::endl;

  if (!(vbox.isEmpty())) {
    std::cout << "vbox is not empty..." << std::endl;
    if ((top_eg.keep_out_zones_.size() == 0) || (!((top_eg.keep_out_zones_.back().min() == vbox.min()) &&
                                                   (top_eg.keep_out_zones_.back().max() == vbox.max())))) {
      std::cout << "Adding vbox..." << std::endl;
      top_eg.keep_out_zones_.push_back(vbox);
      std::cout << "Added vbox to keep_out_zones_" << std::endl;
    } else {
      std::cout << "vbox already exists in keep_out_zones_" << std::endl;
    }
  } else {
    std::cout << "vbox is empty..." << std::endl;
    top_eg.keep_out_zones_.clear();
  }

  if (!top_eg.Solve()) {
    std::cout << "Failure: Problem " << problemIndex << " could not be solved!" << std::endl;
    std::cout << "--------------------------------------------" << std::endl;
    if (!(top_eg.nn_training_mode)) {
      top_eg.WriteTrajectoryToFile("output_" + std::to_string(problemIndex), /*include_timestamp=*/ false);
    }
    return false;
  }

  std::cout << "Success: Problem " << problemIndex << " solved!" << std::endl;
  std::cout << "--------------------------------------------" << std::endl;
  top_eg.WriteTrajectoryToFile("output_" + std::to_string(problemIndex), /*include_timestamp=*/ false);
  return true;
}

void debugObsAvoidance() {
  // original ko_min:    0 -9.2  4.4
  // original ko_max: 1500   -8  4.8
  // pose min: -1e+30 -1e+30 -1e+30
  // pose max: 1e+30 1e+30 1e+30
  // ko_min: -0.1 -9.3  4.3
  // ko_max: 1500.1   -7.9    4.9
  // ko_center:  750 -8.6  4.6
  scp::Vec3 min_pos = scp::Vec3(-1e+30, -1e+30, -1e+30);
  scp::Vec3 max_pos = scp::Vec3(1e+30, 1e+30, 1e+30);
  // scp::Vec3 ko_min = scp::Vec3(-0.1, -9.3, 4.3);
  // scp::Vec3 ko_max = scp::Vec3(1500.1, -7.9, 4.9);
  // scp::Vec3 ko_center = scp::Vec3(750, -8.6, 4.6);

  scp::Vec3 ko_min = scp::Vec3(-3.0, -2.0, -1.0);
  scp::Vec3 ko_max = scp::Vec3(3.0, 2.0, 1.0);
  scp::Vec3 ko_center = scp::Vec3(0.0, 0.0, 0.0);

  std::cout << "ko_min: " << ko_min.transpose() << std::endl;
  std::cout << "ko_max: " << ko_max.transpose() << std::endl;

  std::vector<scp::Vec3> test_points;
  test_points.push_back(scp::Vec3(-4, -0.5, -0.5));  // Xprev[ii]
  test_points.push_back(scp::Vec3(-2.5, -0.5, -0.5));  // Xprev[ii]
  test_points.push_back(scp::Vec3(-2.5, -2.5, -0.5));  // Xprev[ii]

  test_points.push_back(scp::Vec3(2.5, -3.0, 0.5));  // Xprev[ii]
  test_points.push_back(scp::Vec3(2.5, 1.5, 0.5));  // Xprev[ii]
  test_points.push_back(scp::Vec3(2.5, 1.5, 1.5));  // Xprev[ii]

  // test_points.push_back(scp::Vec3(0.5, -2, 0.5));  // Xprev[ii]
  // test_points.push_back(scp::Vec3(0.5, -0.5, -2));  // Xprev[ii]
  // test_points.push_back(scp::Vec3(0.5, -0.5, -0.5));  // Xprev[ii]

  // test_points.push_back(scp::Vec3(10.28, -9.25, 5.0));  // Xprev[ii]

  // scp::Vec3 test_point = scp::Vec3(10.28, -9.25, 5.0); // Xprev[ii]

  for (size_t i = 0; i < test_points.size(); ++i) {
    scp::Vec3 test_point = test_points[i];

    std::cout << " ------------- " << std::endl;
    std::cout << " test point: " << test_point.transpose() << std::endl;


    int row_idx = 0;
    for (size_t jj = 0; jj < 3; jj++) {
      scp::decimal_t lb = min_pos[jj];
      scp::decimal_t ub = max_pos[jj];
      // lb < x < ub
      // Either ko_max < x < ub or lb < x < ko_min
      bool active_proj = true;
      for (size_t kk = 0; kk < 3; kk++) {
        if (kk == jj) {
          continue;
        }
        if ((test_point(kk) > ko_max[kk]) || (test_point(kk) < ko_min[kk])) {
          // not active projection
          active_proj = false;
          // std::cout << "Seeing x y z " << std::to_string(Xprev[ii](0)) << ", " << std::to_string(Xprev[ii](1)) <<
          // ", " << std::to_string(Xprev[ii](2)) << " and judging that no constraint is required." << std::endl;
          break;
        }
      }
      if (active_proj) {
        if (test_point(jj) >= ko_center[jj]) {
          lb = ko_max[jj];
        } else {
          ub = ko_min[jj];
        }
        // std::cout << "Seeing x y z " << std::to_string(Xprev[ii](0)) << ", " << std::to_string(Xprev[ii](1)) << ",
        // " << std::to_string(Xprev[ii](2)) << " and judging that " << std::to_string(jj) << " needs CONSTRAINT " <<
        // std::to_string(lb) << ", " << std::to_string(ub) << "." << std::endl;
      }
      // lb < x < ub
      std::cout << "Constraint " << std::to_string(row_idx) << std::endl;
      std::string var = (jj == 0) ? "x" : (jj == 1) ? "y" : "z";
      std::cout << " " << std::to_string(lb) << " <= " << var << " <= " <<
                  std::to_string(ub) << std::endl;
      // linear_con_mat.coeffRef(row_idx, state_dim * ii + jj) = 1.0;
      // lower_bound(row_idx) = lb;
      // upper_bound(row_idx) = ub;
      row_idx++;
    }
    std::cout << " ------------- " << std::endl;
  }
}

bool fileExists(const std::string& filename) {
  std::ifstream file(filename);
  return file.good();  // Returns true if file can be opened
}

int main() {
  bool test_granite_no_obs = false;
  bool test_granite_large_obs = false;
  bool test_granite_small_obs = false;
  bool test_iss_no_obs = false;
  bool test_iss_small_obs = false;
  bool test_iss_large_obs = false;

  bool test_debug_obs_avoidance = false;

  bool create_training_data = false;
  bool train_and_save_model = false;
  bool train_and_save_model_spline = false;
  bool load_and_run_inference = false;
  bool test_warm_start = false;
  bool test_cold_start_spline = false;
  bool test_warm_start_spline = false;
  bool test_lin_ang_vel_limits = false;

  bool test_state_bound_constraints = false;
  bool test_obs_avoidance_translation = false;

  bool granite_obs_avoidance = false;

  int num_problems = 0;

  if (test_granite_no_obs || test_granite_large_obs || test_granite_small_obs) {
    scp::TOP top_eg(20., 801);
    top_eg.is_granite = true;
    if (top_eg.is_granite) {
      top_eg.x_min(2) = -0.675;  // z coordinate
      top_eg.x_max(2) = -0.67;
      top_eg.x_min(6) = -0.05;  // qx
      top_eg.x_max(6) = 0.05;
      top_eg.x_min(7) = -0.05;  // qy
      top_eg.x_max(7) = 0.05;
    }

    // Initialize motion cases
    scp::Vec13Vec x0s, xgs;
    std::tie(x0s, xgs) = initializeMotionCases(top_eg.is_granite);

    if (test_granite_no_obs) {
      // Process problems without obstacles
      for (size_t i = 0; i < xgs.size(); ++i) {
        num_problems++;
        processProblemInstance(top_eg, x0s[i], xgs[i], Eigen::AlignedBox3d(), num_problems);
      }
    }

    if (test_granite_large_obs) {
      // Process problems with a large obstacle
      Eigen::AlignedBox3d largeObstacle;
      largeObstacle.extend(Eigen::Vector3d(-0.4, 0., -2));
      largeObstacle.extend(Eigen::Vector3d(0., -0.4, 0));
      for (size_t i = 0; i < xgs.size(); ++i) {
        num_problems++;
        processProblemInstance(top_eg, x0s[i], xgs[i], largeObstacle, num_problems);
      }
    }

    if (test_granite_small_obs) {
      // Process problems with a smaller obstacle
      Eigen::AlignedBox3d smallObstacle;
      smallObstacle.extend(Eigen::Vector3d(-0.25, 0., -2));
      smallObstacle.extend(Eigen::Vector3d(0., -0.25, 0));
      for (size_t i = 0; i < xgs.size(); ++i) {
        num_problems++;
        processProblemInstance(top_eg, x0s[i], xgs[i], smallObstacle, num_problems);
      }
    }
  }

  num_problems = 0;  // Reset problem counter for ISS cases

  if (test_iss_no_obs || test_iss_small_obs) {
    scp::TOP top_eg(10., 201);
    // Set ISS environment
    top_eg.is_granite = false;
    top_eg.enforce_obs_avoidance_const = false;
    top_eg.enforce_state_bounds = true;

    // // Set (rough) ISS bounds
    // top_eg.x_min(0) = -100.0;
    // top_eg.x_max(0) = 100.0;
    // top_eg.x_min(1) = -100.0;
    // top_eg.x_max(1) = 100.0;
    // top_eg.x_min(2) = -100.0;
    // top_eg.x_max(2) = 100.0;

    // Initialize motion cases
    scp::Vec13Vec x0s, xgs;
    std::tie(x0s, xgs) = initializeMotionCases(top_eg.is_granite);

    if (test_iss_no_obs) {
      // Process problems without obstacles
      for (size_t i = 0; i < xgs.size(); ++i) {
        num_problems++;
        processProblemInstance(top_eg, x0s[i], xgs[i], Eigen::AlignedBox3d(), num_problems);
      }
    }
    if (test_iss_small_obs) {
      // Process problems with a smaller obstacle
      Eigen::AlignedBox3d smallObstacle;
      // smallObstacle.extend(Eigen::Vector3d(0.0, -9.2, 4.4));
      // smallObstacle.extend(Eigen::Vector3d(1500.0, -8.0, 4.8));
      // smallObstacle.extend(Eigen::Vector3d(10.0, -9.2, 4.6));
      // smallObstacle.extend(Eigen::Vector3d(10.6, -9.0, 4.7));
      // smallObstacle.extend(Eigen::Vector3d(10.0, -9.2, 100.0));
      // smallObstacle.extend(Eigen::Vector3d(10.6, -9.0, 200.0));
      // smallObstacle.extend(Eigen::Vector3d(10.0, -9.3, 4.0));
      // smallObstacle.extend(Eigen::Vector3d(10.6, -8.8, 4.8));
      smallObstacle.extend(Eigen::Vector3d(10.2, -9.3, 4.0));
      smallObstacle.extend(Eigen::Vector3d(10.3, -8.8, 4.8));
      top_eg.enforce_obs_avoidance_const = true;
      for (size_t i = 0; i < xgs.size(); ++i) {
        num_problems++;
        processProblemInstance(top_eg, x0s[i], xgs[i], smallObstacle, num_problems);
      }
    }
    if (test_iss_large_obs) {
      // Process problems with a larger obstacle
      Eigen::AlignedBox3d obstacle;
      obstacle.extend(Eigen::Vector3d(-1500.0, -9.2, 4.4));
      obstacle.extend(Eigen::Vector3d(1500.0, -8.0, 4.8));
      top_eg.enforce_obs_avoidance_const = true;
      for (size_t i = 0; i < xgs.size(); ++i) {
        num_problems++;
        processProblemInstance(top_eg, x0s[i], xgs[i], obstacle, num_problems);
      }
    }
  }

  if (test_debug_obs_avoidance) {
    debugObsAvoidance();
  }

  if (create_training_data) {
    std::cout << "Creating training data..." << std::endl;
    scp::TOP top(20., 401);
    top.is_granite = false;
    top.enforce_obs_avoidance_const = true;
    top.nn_training_mode = true;
    top.max_iter_solver_ = 10000;

    Eigen::AlignedBox3d smallObstacle;
    smallObstacle.extend(Eigen::Vector3d(10.5, -9.5, 4.9));
    smallObstacle.extend(Eigen::Vector3d(10.82, -9.2, 5.0));

    // Initialize motion cases
    scp::Vec13Vec x0s, xgs;
    std::cout << "Initializing motion cases..." << std::endl;
    std::tie(x0s, xgs) = initializeMotionCases(top.is_granite, /* nn_training_mode= */ top.nn_training_mode);
    std::cout << "Number of motion cases: " << xgs.size() << std::endl;

    // Process problems
    std::cout << "Processing problems..." << std::endl;
    int num_successes = 0;
    for (size_t i = 0; i < xgs.size(); ++i) {
      num_problems++;
      bool status = processProblemInstance(top, x0s[i], xgs[i], smallObstacle, num_problems);
      if (status) {
        num_successes++;
      }
    }
    std::cout << "Number of successful problems: " << num_successes << " out of " << num_problems << std::endl;
  }

  if (train_and_save_model) {
    scp::TOP top(20., 801);
    top.is_granite = false;
    // Get the first train_set_size files in output_trajs_for_NN
    int train_set_size = 27;
    int num_epochs = 50000;
    std::vector<std::string> files;
    for (int i = 1; i <= train_set_size; i++) {
      files.push_back("output_trajs_for_NN/iss_optim_trajectory_" + std::to_string(i) + ".txt");
    }
    top.TrainModel(files, num_epochs);

    std::string timestamp = top.getCurrentTimestamp();
    std::string filename = "saved_NN_models/trained_model_" + std::to_string(train_set_size) + "_" + timestamp + ".pt";
    top.SaveModel(filename);
  }

  if (train_and_save_model_spline) {
    scp::TOP top(20., 401);
    top.is_granite = false;
    top.nn_spline_mode = true;
    top.nn_training_mode = true;
    int num_epochs = 500000;
    std::vector<std::string> files;
    std::string directory_path = "/home/enceladus/astrobee/src/planner_scp_gusto_outputs/nn_training/";
    int max_suffix = 890;
    for (int i = 1; i <= max_suffix; i++) {
      std::string pot_file = directory_path + "output_" + std::to_string(i) + ".txt";
      if (fileExists(pot_file)) {
        files.push_back(pot_file);
      }
    }
    std::cout << "Number of files: " << files.size() << std::endl;

    top.TrainModel(files, num_epochs);

    std::string timestamp = top.getCurrentTimestamp();
    std::string filename = "saved_NN_models/trained_model_" + std::to_string(files.size()) + "_" + timestamp + ".pt";
    top.SaveModel(filename);
  }

  if (load_and_run_inference) {
    scp::TOP top(20., 801);
    top.is_granite = false;

    // First try with init model
    std::cout << "Using init model" << std::endl;
    scp::Vec13 x0;
    scp::Vec13 xg;
    x0 << 10.28, -9.81, 4.30, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    xg << 10.48, -9.81, 4.30, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    std::cout << "x0: " << x0.transpose() << std::endl;
    std::cout << "xg: " << xg.transpose() << std::endl;
    scp::Vec13Vec Xprev;
    scp::Vec6Vec Uprev;
    std::tie(Xprev, Uprev) = top.WarmStartFromNN(x0, xg);
    std::cout << "Warm start from neural network:" << std::endl;
    std::cout << "Xprev initial: " << Xprev[0].transpose() << std::endl;
    std::cout << "Xprev final: " << Xprev[Xprev.size() - 1].transpose() << std::endl;
    std::cout << "Uprev initial: " << Uprev[0].transpose() << std::endl;
    std::cout << "Uprev final: " << Uprev[Uprev.size() - 1].transpose() << std::endl;

    // Load model
    std::string fname = "saved_NN_models/trained_model_5_2025-01-03_00-03-15.pt";
    std::cout << "Using " << fname << " model" << std::endl;
    top.LoadModel(fname);

    x0.setZero();
    xg.setZero();
    Xprev.clear();
    Uprev.clear();
    x0 << 10.28, -9.81, 4.30, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    xg << 10.48, -9.81, 4.30, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    std::cout << "x0: " << x0.transpose() << std::endl;
    std::cout << "xg: " << xg.transpose() << std::endl;
    std::tie(Xprev, Uprev) = top.WarmStartFromNN(x0, xg);
    std::cout << "Warm start from neural network:" << std::endl;
    std::cout << "Xprev initial: " << Xprev[0].transpose() << std::endl;
    std::cout << "Xprev final: " << Xprev[Xprev.size() - 1].transpose() << std::endl;
    std::cout << "Uprev initial: " << Uprev[0].transpose() << std::endl;
    std::cout << "Uprev final: " << Uprev[Uprev.size() - 1].transpose() << std::endl;

    // Load model
    fname = "saved_NN_models/trained_model_27_2025-01-03_00-19-49.pt";
    std::cout << "Using " << fname << " model" << std::endl;
    top.LoadModel(fname);

    x0.setZero();
    xg.setZero();
    Xprev.clear();
    Uprev.clear();
    x0 << 10.28, -9.81, 4.30, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    xg << 10.48, -9.81, 4.30, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    std::cout << "x0: " << x0.transpose() << std::endl;
    std::cout << "xg: " << xg.transpose() << std::endl;
    std::tie(Xprev, Uprev) = top.WarmStartFromNN(x0, xg);
    std::cout << "Warm start from neural network:" << std::endl;
    std::cout << "Xprev initial: " << Xprev[0].transpose() << std::endl;
    std::cout << "Xprev final: " << Xprev[Xprev.size() - 1].transpose() << std::endl;
    std::cout << "Uprev initial: " << Uprev[0].transpose() << std::endl;
    std::cout << "Uprev final: " << Uprev[Uprev.size() - 1].transpose() << std::endl;

    // Load model
    fname = "saved_NN_models/trained_model_27_2025-01-03_00-27-55.pt";
    std::cout << "Using " << fname << " model" << std::endl;
    top.LoadModel(fname);

    x0.setZero();
    xg.setZero();
    Xprev.clear();
    Uprev.clear();
    x0 << 10.28, -9.81, 4.30, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    xg << 10.48, -9.81, 4.30, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    std::cout << "x0: " << x0.transpose() << std::endl;
    std::cout << "xg: " << xg.transpose() << std::endl;
    std::tie(Xprev, Uprev) = top.WarmStartFromNN(x0, xg);
    std::cout << "Warm start from neural network:" << std::endl;
    std::cout << "Xprev initial: " << Xprev[0].transpose() << std::endl;
    std::cout << "Xprev final: " << Xprev[Xprev.size() - 1].transpose() << std::endl;
    std::cout << "Uprev initial: " << Uprev[0].transpose() << std::endl;
    std::cout << "Uprev final: " << Uprev[Uprev.size() - 1].transpose() << std::endl;

    // Load model
    fname = "saved_NN_models/trained_model_27_2025-01-03_00-34-39.pt";
    std::cout << "Using " << fname << " model" << std::endl;
    top.LoadModel(fname);

    x0.setZero();
    xg.setZero();
    Xprev.clear();
    Uprev.clear();
    x0 << 10.28, -9.81, 4.30, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    xg << 10.48, -9.81, 4.30, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    std::cout << "x0: " << x0.transpose() << std::endl;
    std::cout << "xg: " << xg.transpose() << std::endl;
    std::tie(Xprev, Uprev) = top.WarmStartFromNN(x0, xg);
    std::cout << "Warm start from neural network:" << std::endl;
    std::cout << "Xprev initial: " << Xprev[0].transpose() << std::endl;
    std::cout << "Xprev final: " << Xprev[Xprev.size() - 1].transpose() << std::endl;
    std::cout << "Uprev initial: " << Uprev[0].transpose() << std::endl;
    std::cout << "Uprev final: " << Uprev[Uprev.size() - 1].transpose() << std::endl;
  }

  if (test_warm_start) {
    // Cold start with straight line initialization
    scp::TOP top_cold(20., 801);
    top_cold.use_nn_warm_start = false;

    // Warm start from NN
    scp::TOP top_warm(20., 801);
    top_warm.use_nn_warm_start = true;

    // Set common parameters
    top_cold.is_granite = false;
    top_warm.is_granite = false;
    top_cold.x0 << 10.28, -9.81, 4.30, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    top_cold.xg << 10.48, -9.81, 4.30, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    top_warm.x0 << 10.28, -9.81, 4.30, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    top_warm.xg << 10.48, -9.81, 4.30, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;

    // Solve problems
    if (!top_cold.Solve()) {
      std::cout << "Cold start: Problem could not be solved!" << std::endl;
    } else {
      std::cout << "Cold start: Problem solved!" << std::endl;
      // TODO(somrita): Log number of iterations or time to solve and quality of solution
    }
    if (!top_warm.Solve()) {
      std::cout << "Warm start: Problem could not be solved!" << std::endl;
    } else {
      std::cout << "Warm start: Problem solved!" << std::endl;
      // TODO(somrita): Log number of iterations or time to solve and quality of solution
    }
    std::cout << "--------------------------------------------" << std::endl;
  }

  if (test_cold_start_spline || test_warm_start_spline) {
    // Set common parameters
    bool is_granite = false;
    int N = 401;
    double Tf = 20.0;
    // ISS params
    double radius = 0.26;
    double mass = 9.583788668;
    Eigen::Matrix3d J;
    J << 0.153427995, 0.0, 0.0, 0.0, 0.14271405, 0.0, 0.0, 0.0, 0.162302759;
    Eigen::Matrix3d Jinv = J.inverse();
    // Create obstacle
    Eigen::AlignedBox3d smallObstacle;
    smallObstacle.extend(Eigen::Vector3d(10.5, -9.5, 4.9));
    smallObstacle.extend(Eigen::Vector3d(10.82, -9.2, 5.0));
    // Set minmax bounds
    Eigen::VectorXd x_min(3);
    Eigen::VectorXd x_max(3);
    x_min << 9.53589, -11.6365, 3.75059;
    x_max << 12.3359, -2.7532, 5.95059;
    // Set x0 and xg
    Eigen::VectorXd x0(13);
    Eigen::VectorXd xg(13);
    x0 << 10.8, -9.4, 4.8, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    xg << 10.9, -9.0, 5.0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    if (test_cold_start_spline) {
      // Cold start with straight line initialization
      scp::TOP top_cold(Tf, N);
      top_cold.use_nn_warm_start = false;
      top_cold.is_granite = is_granite;
      top_cold.radius_ = radius;
      top_cold.mass = mass;
      top_cold.J = J;
      top_cold.Jinv = Jinv;
      top_cold.enforce_obs_avoidance_const = true;
      top_cold.keep_out_zones_.push_back(smallObstacle);
      for (int i = 0; i < 3; i++) {
        top_cold.x_min(i) = x_min(i);
        top_cold.x_max(i) = x_max(i);
      }
      top_cold.x0 = x0;
      top_cold.xg = xg;
      // Solve problems
      if (!top_cold.Solve()) {
        std::cout << "Cold start: Problem could not be solved!" << std::endl;
      } else {
        std::cout << "Cold start: Problem solved!" << std::endl;
        // TODO(somrita): Log number of iterations or time to solve and quality of solution
      }
      std::cout << "--------------------------------------------" << std::endl;
    }
    if (test_warm_start_spline) {
      // Warm start from NN
      scp::TOP top_warm(Tf, N);
      top_warm.use_nn_warm_start = true;
      top_warm.nn_spline_mode = true;
      top_warm.nn_spline_model_path =
      "/home/enceladus/astrobee/src/saved_NN_models/trained_model_625_2025-02-03_00-50-21_003.pt";
      top_warm.is_granite = is_granite;
      top_warm.radius_ = radius;
      top_warm.mass = mass;
      top_warm.J = J;
      top_warm.Jinv = Jinv;
      top_warm.enforce_obs_avoidance_const = true;
      top_warm.keep_out_zones_.push_back(smallObstacle);
      for (int i = 0; i < 3; i++) {
        top_warm.x_min(i) = x_min(i);
        top_warm.x_max(i) = x_max(i);
      }
      top_warm.x0 = x0;
      top_warm.xg = xg;
      // Solve problems
      if (!top_warm.Solve()) {
        std::cout << "Warm start: Problem could not be solved!" << std::endl;
      } else {
        std::cout << "Warm start: Problem solved!" << std::endl;
        // TODO(somrita): Log number of iterations or time to solve and quality of solution
      }
      std::cout << "--------------------------------------------" << std::endl;
    }
  }

  if (test_lin_ang_vel_limits) {
    // Set common parameters
    bool is_granite = false;
    int N = 401;
    double Tf = 20.0;
    // ISS params
    double radius = 0.26;
    double mass = 9.583788668;
    Eigen::Matrix3d J;
    J << 0.153427995, 0.0, 0.0, 0.0, 0.14271405, 0.0, 0.0, 0.0, 0.162302759;
    Eigen::Matrix3d Jinv = J.inverse();
    // Create obstacle
    Eigen::AlignedBox3d smallObstacle;
    smallObstacle.extend(Eigen::Vector3d(10.5, -9.5, 4.9));
    smallObstacle.extend(Eigen::Vector3d(10.82, -9.2, 5.0));
    // Set minmax bounds
    Eigen::VectorXd x_min(3);
    Eigen::VectorXd x_max(3);
    x_min << 9.53589, -11.6365, 3.75059;
    x_max << 12.3359, -2.7532, 5.95059;
    // Set x0 and xg
    Eigen::VectorXd x0(13);
    Eigen::VectorXd xg(13);
    x0 << 10.8, -9.4, 4.8, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    xg << 10.9, -9.0, 5.0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;

    // Create TOP
    scp::TOP top_warm(Tf, N);
    top_warm.use_nn_warm_start = true;
    top_warm.nn_spline_mode = true;
    top_warm.nn_spline_model_path =
    "/home/enceladus/astrobee/src/saved_NN_models/trained_model_625_2025-02-03_00-50-21_003.pt";
    top_warm.is_granite = is_granite;
    top_warm.radius_ = radius;
    top_warm.mass = mass;
    top_warm.J = J;
    top_warm.Jinv = Jinv;
    top_warm.enforce_obs_avoidance_const = true;
    top_warm.enforce_lin_vel_limit = true;
    top_warm.enforce_ang_vel_limit = true;
    top_warm.keep_out_zones_.push_back(smallObstacle);
    for (int i = 0; i < 3; i++) {
      top_warm.x_min(i) = x_min(i);
      top_warm.x_max(i) = x_max(i);
    }
    top_warm.x0 = x0;
    top_warm.xg = xg;
    // Solve problems
    if (!top_warm.Solve()) {
      std::cout << "Warm start: Problem could not be solved!" << std::endl;
    } else {
      std::cout << "Warm start: Problem solved!" << std::endl;
      // TODO(somrita): Log number of iterations or time to solve and quality of solution
    }
    std::cout << "--------------------------------------------" << std::endl;
  }

  if (test_state_bound_constraints) {
    scp::TOP top(10., 201);
    top.is_granite = false;
    top.enforce_obs_avoidance_const = false;
    top.nn_training_mode = false;
    top.enforce_state_bounds = true;

    scp::TOP top2(10., 201);
    top2.is_granite = false;
    top2.enforce_obs_avoidance_const = false;
    top2.nn_training_mode = false;
    top2.enforce_state_bounds = false;

    // Initialize motion
    top.x0 << 9.5, -9.8, 4.3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    top.xg << 9.5, -6.8, 4.3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    top2.x0 << 9.5, -9.8, 4.3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    top2.xg << 9.5, -6.8, 4.3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;

    if (!top.Solve()) {
      std::cout << "With state bounds: problem could not be solved!" << std::endl;
    } else {
      std::cout << "With state bounds: problem solved!" << std::endl;
    }
    std::cout << "--------------------------------------------" << std::endl;

    if (!top2.Solve()) {
      std::cout << "Without state bounds: problem could not be solved!" << std::endl;
    } else {
      std::cout << "Without state bounds: problem solved!" << std::endl;
    }
    std::cout << "--------------------------------------------" << std::endl;
  }

  if (test_obs_avoidance_translation) {
    std::vector<int> N_vals = {11, 201, 401, 401};
    std::vector<bool> obs_avoids = {true, true, false, true};
    for (size_t i = 0; i < N_vals.size(); ++i) {
      int N = N_vals[i];
      bool obs_avoid = obs_avoids[i];

      scp::TOP* top;
      top = new scp::TOP(20., N);
      top->is_granite = true;
      top->enforce_obs_avoidance_const = obs_avoid;
      top->use_nn_warm_start = false;
      // top->x0 << -0.390941, 0.385616, -0.678817, 0, 0, 0, -0.00158839, 0.00167167, -0.00057889, 0.999997, 0, 0, 0;
      // top->xg << 0.5, -0.3, -0.67, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
      top->x0 << 0.302588, -0.274509, -0.674623, 0, 0, 0, -0.000435765, 0.0014564, -0.000290585, 0.999999, 0, 0, 0;
      top->xg << 0.4, -0.3, -0.67, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;

      top->radius_ = 0.26;
      top->mass = 18.9715;
      top->J << 0.2517, 0.0, 0.0, 0.0, 0.2517, 0.0, 0.0, 0.0, 0.0025;
      top->Jinv = top->J.inverse();

      top->keep_in_zones_.clear();
      Eigen::AlignedBox3d kiz;
      kiz.extend(Eigen::Vector3d(-1, -1, -0.75));
      kiz.extend(Eigen::Vector3d(1, 1, -0.6));
      top->keep_in_zones_.push_back(kiz);

      // Set the xmin and xmax using the keepin zones
      for (size_t ii = 0; ii < 3; ii++) {
        top->x_min(ii) = kiz.min()(ii);
        top->x_max(ii) = kiz.max()(ii);
      }

      top->keep_out_zones_.clear();
      Eigen::AlignedBox3d smallObstacle;
      smallObstacle.extend(Eigen::Vector3d(-0.25, -0.25, -2));
      smallObstacle.extend(Eigen::Vector3d(0., 0., 0));
      top->keep_out_zones_.push_back(smallObstacle);

      if (!top->Solve()) {
        std::cout << "Problem (with N = " << top->N << " could not be solved!" << std::endl;
      } else {
        std::cout << "Problem (with N = " << top->N << " solved!" << std::endl;
      }
      std::cout << "--------------------------------------------" << std::endl;
    }
  }

  if (granite_obs_avoidance) {
    scp::TOP* top;
    top = new scp::TOP(20., 401);
    top->is_granite = true;
    top->enforce_obs_avoidance_const = true;
    top->use_nn_warm_start = false;
    top->x0 << -0.3, 0.3, -0.67, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    top->xg << 0.2, -0.3, -0.67, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
    if (top->is_granite) {
      top->radius_ = 0.26;
      top->mass = 18.9715;
      top->J << 0.2517, 0.0, 0.0, 0.0, 0.2517, 0.0, 0.0, 0.0, 0.2517;
      top->Jinv = top->J.inverse();

      Eigen::AlignedBox3d kiz;
      kiz.extend(Eigen::Vector3d(-1, -1, -0.75));
      kiz.extend(Eigen::Vector3d(1, 1, -0.6));
      top->keep_in_zones_.push_back(kiz);

      top->x_min(0) = -1.0;
      top->x_max(0) = 1.0;
      top->x_min(1) = -1.0;
      top->x_max(1) = 1.0;
      top->x_min(2) = -0.75;
      top->x_max(2) = -0.6;
    }

    Eigen::AlignedBox3d smallObstacle;
    smallObstacle.extend(Eigen::Vector3d(-0.25, -0.25, -2));
    smallObstacle.extend(Eigen::Vector3d(0., 0., 0));
    top->keep_out_zones_.push_back(smallObstacle);

    if (!top->Solve()) {
      std::cout << "Problem could not be solved!" << std::endl;
    } else {
      std::cout << "Problem solved!" << std::endl;
      top->WriteTrajectoryToFile("output_granite_obs_avoidance");
    }
    std::cout << "--------------------------------------------" << std::endl;
  }

  // scp::TOP* top;
  // top = new scp::TOP(20., 801);
  // for (int ii = 0; ii < 100; ii++) {
  //   top->nn_model_path = "/home/enceladus/astrobee/src/saved_NN_models/trained_model_27_2025-01-03_00-34-39.pt";
  //   top->use_nn_warm_start = true;
  //   top->is_granite = false;
  //   top->x0 << 9.5, -9.8, 4.3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
  //   top->xg << 9.5, -6.8, 4.3, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;

  //   if (!top->Solve()) {
  //     std::cout << "Warm start: " << ii << " Problem could not be solved!" << std::endl;
  //   } else {
  //     std::cout << "Warm start: " << ii << " Problem solved!" << std::endl;
  //   }
  //   std::cout << "--------------------------------------------" << std::endl;
  // }

  // scp::TOP* top;
  // top = new scp::TOP(20., 801);
  // top->nn_model_path = "/home/enceladus/astrobee/src/saved_NN_models/trained_model_27_2025-01-03_00-34-39.pt";
  // top->use_nn_warm_start = true;
  // top->Solve();

  return 0;
}
