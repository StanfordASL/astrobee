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
  : N(N_), Tf(Tf_), net(std::make_shared<Net>()), optimizer(net->parameters(), torch::optim::AdamOptions(0.001)) {
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
  // Set weights to zero
  // net.initializeWeightsToZero();
  // OR Load weights from file
  // net.loadWeights("path/to/net_weights.pt");

  // Folder to save outputs
  output_dir = "planner_scp_gusto_outputs";

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
  enforce_state_bounds = false;

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

  // TODO(acauligi): read off params server
  radius_ = 0.26;
  mass = 9.583788668;
  J << 0.153427995, 0, 0,
    0, 0.14271405, 0,
    0, 0, 0.162302759;
  Jinv = J.inverse();

  // TODO(somrita): freeze these parameters once debugging is complete
  desired_vel_ = 0.2000;
  // desired_accel_ = 0.0175;
  desired_accel_ = 1e2;
  // desired_accel_ = 1e-2;
  // desired_accel_ = 0.0;
  // desired_accel_ = 0.1;
  // desired_accel_ = 100.0;
  desired_omega_ = 0.1745;
  desired_alpha_ = 0.1745;
  // desired_alpha_ = 0.600;

  // TODO(acauligi): process keep-in+keep-out data
  keep_in_zones_.clear();
  keep_out_zones_.clear();

  x_max << 100.0, 100.0, 100.0,
    desired_vel_, desired_vel_, desired_vel_,
    1, 1, 1, 1,
    desired_omega_, desired_omega_, desired_omega_;
  x_min = -x_max;

  // TODO(somrita): remove
  std::cout << "[TOP constructor] Min position" << x_min(0) << x_min(1) << x_min(2) << std::endl;
  std::cout << "[TOP constructor] Min quaternion" << x_min(6) << x_min(7) << x_min(8) << x_min(9) <<std::endl;

  ResetSCPParams();
  UpdateProblemDimension(N);

  // Run warm start for OSQP initialization on demo problem
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

size_t TOP::GetNumTOPConstraints() {
  size_t num_init_cond_constr = state_dim;
  size_t num_final_cond_constr = state_dim;
  size_t num_lin_dynamics_constr = state_dim_lin * (N - 1);   // (x,y,z) and (vx,vy,vz) for each time step
  size_t num_rot_dynamics_constr = state_dim_nlin * (N - 1);  // (q0,q1,q2,q3) and (wx,wy,wz) for each time step
  size_t num_obs_avoidance_const = (N - 1) * pos_dim;               // Exactly 3 (XYZ) constraints per time step
  size_t num_state_bounds_const = N * pos_dim;                      // 3 (XYZ) constraints per time step
  size_t num_total_constr = (enforce_init_cond ? num_init_cond_constr : 0) +
                           (enforce_final_cond ? num_final_cond_constr : 0) +
                           (enforce_lin_dynamics ? num_lin_dynamics_constr : 0) +
                           (enforce_rot_dynamics ? num_rot_dynamics_constr : 0) +
                           (enforce_obs_avoidance_const ? num_obs_avoidance_const : 0) +
                           (enforce_state_bounds ? num_state_bounds_const : 0);
  if (enforce_force_norm || enforce_moment_norm || enforce_state_LB || enforce_state_UB || enforce_lin_vel_norm ||
      enforce_ang_vel_norm) {
    throw std::runtime_error("Error: Constraints not implemented yet!");
    return false;
  }
  // std::cout << "Init cond: " << num_init_cond_constr << std::endl;
  // std::cout << "Final cond: " << num_final_cond_constr << std::endl;
  // std::cout << "Lin dynamics: " << num_lin_dynamics_constr << std::endl;
  // std::cout << "Rot dynamics: " << num_rot_dynamics_constr << std::endl;
  // std::cout << "Obs avoidance: " << num_obs_avoidance_const << std::endl;
  // std::cout << "State bounds: " << num_state_bounds_const << std::endl;

  // // Print which constraints are enabled and corresponding number of constraints
  // std::cout << "enforce_init_cond: " << enforce_init_cond << " (" << num_init_cond_constr << " constraints)"
  //           << std::endl;
  // std::cout << "enforce_final_cond: " << enforce_final_cond << " (" << num_final_cond_constr << " constraints)"
  //           << std::endl;
  // std::cout << "enforce_lin_dynamics: " << enforce_lin_dynamics << " (" << num_lin_dynamics_constr << " constraints)"
  //           << std::endl;
  // std::cout << "enforce_rot_dynamics: " << enforce_rot_dynamics << " (" << num_rot_dynamics_constr << " constraints)"
  //           << std::endl;
  // std::cout << "enforce_obs_avoidance_const: " << enforce_obs_avoidance_const << " (" << num_obs_avoidance_const
  //           << " constraints)" << std::endl;
  // std::cout << "enforce_state_bounds: " << enforce_state_bounds << " (" << num_state_bounds_const << " constraints)"
  //           << std::endl;
  // std::cout << "Total constraints: " << num_total_constr << std::endl;

  return num_total_constr;

  // // TODO(somrita): Continue updating these
  // size_t num_force_norm_cons = penalize_total_force ? 11*(N-1) : 10*(N-1);
  // size_t num_moment_norm_cons = penalize_total_moment ? 11*(N-1) : 10*(N-1);
  // return state_dim*(N-1)  // Dynamics
  // + 2*state_dim  // Initial and final boundary conditions
  // + num_force_norm_cons  // Force norm constraints
  // + num_moment_norm_cons  // Moment norm constraints
  // + 2*state_bd_dim*(N-1)  // State LB constraints
  // + 2*state_bd_dim*(N-1)  // State UB constraints
  // + 11*(N-1)  // Linear velocity norm constraints
  // + 11*(N-1)  // Angular velocity norm constraints
  // + 3*(N-1);  // Obstacle avoidance constraints
}

Vec3 TOP::MinPos() { return Vec3(x_min(0), x_min(1), x_min(2)); }

Vec3 TOP::MaxPos() { return Vec3(x_max(0), x_max(1), x_max(2)); }

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
  size_t num_cons = GetNumTOPConstraints();

  std::cout << "[TOP::UpdateProblemDimension] Num vars: " << num_vars << " Num cons: " << num_cons << std::endl;

  hessian.resize(num_vars, num_vars);
  linear_con_mat.resize(num_cons, num_vars);
  gradient.resize(num_vars);
  lower_bound.resize(num_cons);
  upper_bound.resize(num_cons);
  qp_soln.resize(num_vars);

  // UpdateDoubleIntegrator();
  // UpdateRotationalDynamics();

  if (use_nn_warm_start) {
    std::cout << "[TOP::UpdateProblemDimension] Using NN warm start" << std::endl;
    InitTrajWarmStart();
  } else {
    std::cout << "[TOP::UpdateProblemDimension] Using straight line cold start" << std::endl;
    InitTrajStraightline();
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

  // Set up solver
  abs_tol_ = 1e-5;  // default 1e-03
  rel_tol_ = 1e-5;  // default 1e-03
  primal_tol_ = 1e-8;  // default 1e-04
  dual_tol_ = 1e-8;  // default 1e-04
  rho_ = 0.3;  // default 0.1
  sigma_ = 1e-8;  // default 1e-06
  // max_iter_solver_ = 200;  // default 4000
  max_iter_solver_ = 4000;  // default 4000
  verbose_ = true;  // TODO(somrita): Change back to false
  warm_start_ = true;
  solver->settings()->setWarmStart(warm_start_);
  solver->settings()->setAbsoluteTolerance(abs_tol_);
  // TODO(somrita) : Figure out how to echo the solver settings
  // std::cout << "Set abs tol to " << solver->settings()->eps_rel <<std::endl;
  solver->settings()->setRelativeTolerance(rel_tol_);
  solver->settings()->setPrimalInfeasibilityTollerance(primal_tol_);
  solver->settings()->setDualInfeasibilityTollerance(dual_tol_);
  // which justifies using a low value of sigma and a high value of rho for equality constraints.
  solver->settings()->setRho(rho_);
  solver->settings()->setSigma(sigma_);
  solver->settings()->setMaxIteraction(max_iter_solver_);


  solver->settings()->setVerbosity(verbose_);
  solver->data()->setNumberOfVariables(num_vars);
  solver->data()->setNumberOfConstraints(num_cons);

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
  // TODO(acauligi): check quaternion convention
  // http://wiki.ros.org/tf2/Tutorials/Quaternions#Components_of_a_quaternion
  Quat q0 = Quat(x0(9), x0(6), x0(7), x0(8));
  Quat qg = Quat(xg(9), xg(6), xg(7), xg(8));
  std::cout << "[TOP::InitTrajStraightLine] x0: " << x0(0) << " " << x0(1) << " " << x0(2) << " " << x0(3) << " "
            << x0(4) << " " << x0(5) << " " << x0(6) << " " << x0(7) << " " << x0(8) << " " << x0(9) << " " << x0(10)
            << " " << x0(11) << " " << x0(12) << std::endl;
  std::cout << "[TOP::InitTrajStraightLine] xg: " << xg(0) << " " << xg(1) << " " << xg(2) << " " << xg(3) << " "
            << xg(4) << " " << xg(5) << " " << xg(6) << " " << xg(7) << " " << xg(8) << " " << xg(9) << " " << xg(10)
            << " " << xg(11) << " " << xg(12) << std::endl;
  std::cout << "[TOP::InitTrajStraightLine] q0: " << q0.x() << " " << q0.y() << " " << q0.z() << " " << q0.w()
            << std::endl;
  std::cout << "[TOP::InitTrajStraightLine] qg: " << qg.x() << " " << qg.y() << " " << qg.z() << " " << qg.w()
            << std::endl;

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
  std::string timestamp = getCurrentTimestamp();
  std::string fname = output_dir + "/" + std::string(is_granite ? "granite" : "iss") +
                      "_initial_straight_line_trajectory" + "_" + timestamp + ".txt";
  WriteTrajectoryToFile(Xprev, Uprev, fname);
}

void TOP::InitTrajWarmStart() {
  // Settings
  bool U_linear_only = true;
  std::string Xinit_method = "linear_interpolation";  // "forward_dynamics" or "linear_interpolation"

  // Load model
  LoadModel(nn_model_path);

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
  std::string timestamp = getCurrentTimestamp();
  std::string fname = output_dir + "/" + std::string(is_granite ? "granite" : "iss") +
                      "_initial_nn_warm_start_trajectory" + "_" + timestamp + ".txt";
  WriteTrajectoryToFile(Xprev, Uprev, fname);
  return;
}

// void TOP::UpdateF(Vec7& f, Vec13& X, Vec6& U) {
//   f.setZero();

//   decimal_t Jxx = J(0, 0);
//   decimal_t Jyy = J(1, 1);
//   decimal_t Jzz = J(2, 2);

//   decimal_t wx = X(10);
//   decimal_t wy = X(11);
//   decimal_t wz = X(12);

//   Vec3 negJinvOmegaJomega;
//   negJinvOmegaJomega << (Jyy-Jzz)*wz*wy/Jxx,
//                         (Jzz-Jxx)*wx*wz/Jyy,
//                         (Jxx-Jyy)*wy*wx/Jzz;

//   f.segment(4, 3) = negJinvOmegaJomega;
// }

// void TOP::UpdateA(Mat7& A, Vec13& X, Vec6& U) {
//   A.setZero();

//   decimal_t wx = X(10);
//   decimal_t wy = X(11);
//   decimal_t wz = X(12);

//   Mat4 df_dq;
//   df_dq << 0, -wz, wy, wx,
//                 wz, 0, -wx, wy,
//                 -wy, wx, 0, wz,
//                 -wx, -wy, -wz, 0;
//   A.block(0, 0, 4, 4) = 0.5*df_dq;
// }

// void TOP::UpdateB(Mat7x3& B, Vec13& X, Vec6& U) {
//   B.setZero();

//   B.block(4, 0, 3, 3) = Jinv;
// }

// void TOP::UpdateRotationalDynamics() {
//   // re-normalize quaternions between iterations
//   NormalizeQuaternions();

//   for (size_t ii = 0; ii < N-1; ii++) {
//     UpdateF(fs[ii], Xprev[ii], Uprev[ii]);
//     UpdateA(As[ii], Xprev[ii], Uprev[ii]);
//     UpdateB(Bs[ii], Xprev[ii], Uprev[ii]);
//   }
// }

// Eigen::Matrix<double, 4, 3> TOP::CalculateQMat(const Eigen::Vector4d& quaternion) {
//     // Extract quaternion components
//     double q_w = quaternion(0);
//     double q_x = quaternion(1);
//     double q_y = quaternion(2);
//     double q_z = quaternion(3);

//     // Construct Q_mat
//     Eigen::Matrix<double, 4, 3> Q_mat;
//     Q_mat << -q_x, -q_y, -q_z,
//               q_w, -q_z,  q_y,
//               q_z,  q_w, -q_x,
//              -q_y,  q_x,  q_w;

//     return Q_mat;
// }

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
Mat4x3 TOP::CalculateQMat(const Vec4& quaternion) {
    // Extract quaternion components
    double q_x = quaternion(0);
    double q_y = quaternion(1);
    double q_z = quaternion(2);
    double q_w = quaternion(3);  // Last element is q_w

    // Construct Q_mat
    Mat4x3 Q_mat;
    // Q_mat << -q_x, -q_y, -q_z,
    //           q_w, -q_z,  q_y,
    //           q_z,  q_w, -q_x,
    //          -q_y,  q_x,  q_w;
    Q_mat <<  q_w,  q_z, -q_y,
             -q_z,  q_w,  q_x,
              q_y, -q_x,  q_w,
             -q_x, -q_y, -q_z;

    return Q_mat;
}

void TOP::SetSimpleConstraints() {
  std::cout << "Setting simple constraints..." << std::endl;
  std::cout << "[TOP::SetSimpleConstraints] enforce_init_cond: " << enforce_init_cond << std::endl;
  std::cout << "[TOP::SetSimpleConstraints] enforce_final_cond: " << enforce_final_cond << std::endl;
  std::cout << "[TOP::SetSimpleConstraints] enforce_lin_dynamics: " << enforce_lin_dynamics << std::endl;
  std::cout << "[TOP::SetSimpleConstraints] enforce_rot_dynamics: " << enforce_rot_dynamics << std::endl;
  std::cout << "[TOP::SetSimpleConstraints] enforce_obs_avoidance_const: " << enforce_obs_avoidance_const << std::endl;
  std::cout << "[TOP::SetSimpleConstraints] enforce_state_bounds: " << enforce_state_bounds << std::endl;

  Mat7 eye;
  eye.setIdentity();

  size_t row_idx = 0;
  auto start_time = std::chrono::high_resolution_clock::now();

  // Initial state
  if (enforce_init_cond) {
    for (size_t ii = 0; ii < state_dim; ii++) {
      linear_con_mat.coeffRef(row_idx, ii) = 1.0;  // Constrain x[0][i] (initial state component)
      lower_bound(row_idx) = x0(ii);
      upper_bound(row_idx) = x0(ii);
      row_idx++;
    }
  }

  // Goal state
  if (enforce_final_cond) {
    for (size_t ii = 0; ii < state_dim; ii++) {
      linear_con_mat.coeffRef(row_idx, state_dim * (N - 1) + ii) = 1.0;  // Constrain x[N-1][i] (final state component)
      lower_bound(row_idx) = xg(ii);
      upper_bound(row_idx) = xg(ii);
      row_idx++;
    }
  }

  if (row_idx != 2*state_dim) {
    std::cerr << "Error: Expected " << 2 * state_dim << " constraints, but added " << row_idx << " constraints."
              << std::endl;
  }

  if (enforce_lin_dynamics) {
    std::vector<Eigen::Triplet<double>> dynamics_triplets;
    for (size_t ii = 0; ii < N-1; ii++) {
      // Double integrator dynamics
      for (size_t jj = 0; jj < pos_dim; jj++) {
        // Position update: x_{i+1} = x_i + v_i * dt
        dynamics_triplets.emplace_back(row_idx, ii * state_dim + jj, -1.0);       // -x_i
        dynamics_triplets.emplace_back(row_idx, ii * state_dim + pos_dim + jj, -dh);   // -v_i * dt
        dynamics_triplets.emplace_back(row_idx, (ii + 1) * state_dim + jj, 1.0);  // x_{i+1}
        lower_bound(row_idx) = 0.0;
        upper_bound(row_idx) = 0.0;
        ++row_idx;

        // Velocity update: v_{i+1} = v_i + u_j * dt / mass
        dynamics_triplets.emplace_back(row_idx, ii * state_dim + pos_dim + jj, -1.0);       // -v_i
        dynamics_triplets.emplace_back(row_idx, N * state_dim + ii * control_dim + jj, -dh/mass);    // -u_j * dt/mass
        dynamics_triplets.emplace_back(row_idx, (ii + 1) * state_dim + pos_dim + jj, 1.0);  // v_{i+1}
        lower_bound(row_idx) = 0.0;
        upper_bound(row_idx) = 0.0;
        ++row_idx;
      }
    }
    // Add dynamics constraints to A matrix
    for (const auto& triplet : dynamics_triplets) {
      linear_con_mat.coeffRef(triplet.row(), triplet.col()) = triplet.value();
    }
  }

  // if (enforce_rot_dynamics) {
  //   UpdateRotationalDynamics();
  //   for (size_t ii = 0; ii < N-1; ii++) {
  //     // Nonlinear attitude dynamics
  //     // q[ii+1] = q[ii] + dh*As.block(0,0,4,4)*q[ii]
  //     // omega[ii+1] = omega[ii] + dh*Bs.block(4,4,3,3)*u[ii] + dh*fs.segment(4,3)
  //     // ==> x[ii+1] = x[ii] + dh*As*x[ii] + dh*Bs*u[ii] + dh*fs
  //     // ==> -dh*fs = -x[ii+1] + (eye + dh*As)*x[ii] + dh*Bs*u[ii]
  //     // As[ii] is a 7x7 matrix
  //     // Bs[ii] is a 7x3 matrix
  //     // fs[ii] is a 7x1 vector
  //     // As.block(0,0,4,4) is a 4x4 matrix (rest is zeros)
  //     // Bs.block(4,0,3,3) is a 3x3 matrix  (rest is zeros)
  //     // fs.segment(4,3) is a 3x1 vector (rest is zeros)

  //     for (size_t jj = 0; jj < state_dim_nlin; jj++) {
  //       lower_bound(row_idx) = -dh*fs[ii](jj);
  //       upper_bound(row_idx) = -dh*fs[ii](jj);

  //       // Simple explicit Euler integration scheme
  //       linear_con_mat.coeffRef(row_idx, state_dim*(ii+1)+state_dim_lin+jj) = -1.0;
  //       for (size_t kk = 0; kk < state_dim_nlin; kk++) {
  //         linear_con_mat.coeffRef(row_idx, state_dim*ii+state_dim_lin+kk) =
  //           (eye(jj, kk)+dh*As[ii](jj, kk) );
  //       }
  //       for (size_t kk = 0; kk < control_dim_nlin; kk++) {
  //         linear_con_mat.coeffRef(row_idx,
  //           state_dim*N+control_dim*ii+control_dim_lin+kk) = dh*Bs[ii](jj, kk);
  //       }
  //       row_idx++;
  //     }
  //     if (ii == 0) {
  //       std::cout << "Rotational constraints matrix As part: \n"
  //                 << As[ii] << std::endl;
  //       std::cout << "Rotational constraints matrix Bs part: \n"
  //                 << Bs[ii] << std::endl;
  //       std::cout << "Rotational constraints matrix fs part: \n"
  //                 << fs[ii] << std::endl;
  //     }
  //   }
  // }

  if (enforce_rot_dynamics) {
    NormalizeQuaternions();
    std::vector<Eigen::Triplet<double>> dynamics_triplets;
    for (size_t ii = 0; ii < N - 1; ii++) {
      // Quaternion kinematics update
      // Quaternion update: q_{i+1} = q_i + 0.5 * Q(q_i) * omega_i * dt
      // Compute QMat dynamically for quaternion at time step `ii`
      Eigen::Vector4d q_i = Xprev[ii].segment(6, 4);
      Eigen::Matrix<double, 4, 3> QMat = CalculateQMat(q_i);
      for (size_t jj = 0; jj < 4; jj++) {
        for (size_t kk = 0; kk < 3; kk++) {
          dynamics_triplets.emplace_back(row_idx, ii * state_dim + pos_dim + lin_vel_dim + jj, -1.0);  // -q_i
          dynamics_triplets.emplace_back(row_idx, ii * state_dim + pos_dim + lin_vel_dim + quat_dim + kk,
                                         -0.5 * dh * QMat(jj, kk));  // -0.5 * Q_mat * omega_i * dt
          dynamics_triplets.emplace_back(row_idx, (ii + 1) * state_dim + pos_dim + lin_vel_dim + jj,
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
        dynamics_triplets.emplace_back(row_idx, ii * state_dim + pos_dim + lin_vel_dim + quat_dim + jj,
                                       -1.0);  // -omega_i
        dynamics_triplets.emplace_back(row_idx, N * state_dim + ii * control_dim + control_dim_lin + jj,
                                       -dh / J(jj, jj));  // -u_torque * dt / J
        dynamics_triplets.emplace_back(row_idx, (ii + 1) * state_dim + pos_dim + lin_vel_dim + quat_dim + jj,
                                       1.0);  // omega_{i+1}

        lower_bound(row_idx) = frot_mat(jj);
        upper_bound(row_idx) = frot_mat(jj);
        ++row_idx;
      }
    }

    // Add rotational dynamics constraints to A matrix
    for (const auto& triplet : dynamics_triplets) {
      linear_con_mat.coeffRef(triplet.row(), triplet.col()) = triplet.value();
    }
  }

  if (enforce_force_norm) {
    // Force constraints
    for (size_t ii = 0; ii < N-1; ii++) {
      for (size_t jj = 0; jj < control_dim_lin; jj++) {
        size_t s_slack_var_idx = state_dim*N+control_dim*(N-1)+ii*num_force_norm_slack_vars_per_iter+jj;
        size_t this_control_idx = state_dim*N+control_dim*ii+jj;
        // -s <=0
        linear_con_mat.coeffRef(row_idx, s_slack_var_idx) = -1.0;
        upper_bound(row_idx) = 0.0;
        row_idx++;
        // -s - a <= 0
        linear_con_mat.coeffRef(row_idx, s_slack_var_idx) = -1.0;
        linear_con_mat.coeffRef(row_idx, this_control_idx) = -1.0;
        upper_bound(row_idx) = 0.0;
        row_idx++;
        // a - s <= 0
        linear_con_mat.coeffRef(row_idx, s_slack_var_idx) = -1.0;
        linear_con_mat.coeffRef(row_idx, this_control_idx) = 1.0;
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
          linear_con_mat.coeffRef(row_idx, s_slack_var_idx) = 1.0;
        }
        linear_con_mat.coeffRef(row_idx, z_slack_var_idx) = -1.0;
        upper_bound(row_idx) = F_max_;
        row_idx++;

        // -zk <= 0
        linear_con_mat.coeffRef(row_idx, z_slack_var_idx) = -1.0;
        upper_bound(row_idx) = 0;
        row_idx++;
      } else {  // sum(s_ik) <= F_max
        for (size_t jj = 0; jj < control_dim_lin; jj++) {
          size_t s_slack_var_idx = state_dim*N+control_dim*(N-1)+ii*num_force_norm_slack_vars_per_iter+jj;
          linear_con_mat.coeffRef(row_idx, s_slack_var_idx) = 1.0;
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
        linear_con_mat.coeffRef(row_idx, s_slack_var_idx) = -1.0;
        upper_bound(row_idx) = 0.0;
        row_idx++;
        // -s - a <= 0
        linear_con_mat.coeffRef(row_idx, s_slack_var_idx) = -1.0;
        linear_con_mat.coeffRef(row_idx, this_control_idx) = -1.0;
        upper_bound(row_idx) = 0.0;
        row_idx++;
        // a - s <= 0
        linear_con_mat.coeffRef(row_idx, s_slack_var_idx) = -1.0;
        linear_con_mat.coeffRef(row_idx, this_control_idx) = 1.0;
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
          linear_con_mat.coeffRef(row_idx, s_slack_var_idx) = 1.0;
        }
        linear_con_mat.coeffRef(row_idx, z_slack_var_idx) = -1.0;
        upper_bound(row_idx) = M_max_;
        row_idx++;

        // -zk <= 0
        linear_con_mat.coeffRef(row_idx, z_slack_var_idx) = -1.0;
        upper_bound(row_idx) = 0;
        row_idx++;

      } else {  // sum(s_ik) <= M_max
        for (size_t jj = 0; jj < control_dim_nlin; jj++) {
          size_t s_slack_var_idx = state_dim*N+control_dim*(N-1)+num_force_norm_slack_vars+
            ii*num_moment_norm_slack_vars_per_iter+jj;
          linear_con_mat.coeffRef(row_idx, s_slack_var_idx) = 1.0;
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
        linear_con_mat.coeffRef(row_idx, slack_var_idx) = -1.0;
        upper_bound(row_idx) = 0;
        row_idx++;

        // -x_ik -z_ik <= -x_min(i) ignoring initial condition ii=0
        linear_con_mat.coeffRef(row_idx, slack_var_idx) = -1.0;
        if (jj <= 2) {
          // position jj[0..2] maps to x_min[0...2]
          linear_con_mat.coeffRef(row_idx, state_dim*(ii+1)+jj) = -1.0;
          upper_bound(row_idx) = -x_min(jj);
        } else {
          // quaternion jj[3...6] maps to x_min[6...9]
          linear_con_mat.coeffRef(row_idx, state_dim*(ii+1)+jj+3) = -1.0;
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
        linear_con_mat.coeffRef(row_idx, slack_var_idx) = -1.0;
        upper_bound(row_idx) = 0;
        row_idx++;

        // x_ik -z_ik <= x_max(i) ignoring initial condition ii=0
        linear_con_mat.coeffRef(row_idx, slack_var_idx) = -1.0;
        if (jj <= 2) {
          // position jj[0..2] maps to x_min[0...2]
          linear_con_mat.coeffRef(row_idx, state_dim*(ii+1)+jj) = 1.0;
          upper_bound(row_idx) = x_max(jj);
        } else {
          // quaternion jj[3...6] maps to x_min[6...9]
          linear_con_mat.coeffRef(row_idx, state_dim*(ii+1)+jj+3) = 1.0;
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
      linear_con_mat.coeffRef(row_idx, z_slack_var_idx) = -1.0;
      upper_bound(row_idx) = 0;
      row_idx++;
      for (size_t jj = 0; jj < lin_vel_dim; jj++) {
        size_t s_slack_var_idx = state_dim*N+control_dim*(N-1)+
          num_force_norm_slack_vars+num_moment_norm_slack_vars+
          state_bd_dim*(N-1)+
          state_bd_dim*(N-1) + 4*ii + jj;
        size_t this_state_idx = state_dim*(ii+1)+ 3 + jj;  // skip ii=0, skip 3 position states
        // -s_ik <= 0
        linear_con_mat.coeffRef(row_idx, s_slack_var_idx) = -1.0;
        upper_bound(row_idx) = 0;
        row_idx++;
        // -x_ik -s_ik <= 0
        linear_con_mat.coeffRef(row_idx, this_state_idx) = -1.0;
        linear_con_mat.coeffRef(row_idx, s_slack_var_idx) = -1.0;
        upper_bound(row_idx) = 0;
        row_idx++;
        // x_ik -s_ik <= 0
        linear_con_mat.coeffRef(row_idx, this_state_idx) = 1.0;
        linear_con_mat.coeffRef(row_idx, s_slack_var_idx) = -1.0;
        upper_bound(row_idx) = 0;
        row_idx++;
      }
      // sum(s_ik) - zk <= v_max
      for (size_t jj = 0; jj < lin_vel_dim; jj++) {
        size_t s_slack_var_idx = state_dim*N+control_dim*(N-1)+
          num_force_norm_slack_vars+num_moment_norm_slack_vars+
          state_bd_dim*(N-1)+state_bd_dim*(N-1) +
          4*ii + jj;
        linear_con_mat.coeffRef(row_idx, s_slack_var_idx) = 1.0;
      }
      linear_con_mat.coeffRef(row_idx, z_slack_var_idx) = -1.0;
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
      linear_con_mat.coeffRef(row_idx, z_slack_var_idx) = -1.0;
      upper_bound(row_idx) = 0;
      row_idx++;
      for (size_t jj = 0; jj < ang_vel_dim; jj++) {
        size_t s_slack_var_idx = state_dim*N+control_dim*(N-1)+
          num_force_norm_slack_vars+num_moment_norm_slack_vars+
          state_bd_dim*(N-1)+state_bd_dim*(N-1)+
          4*(N-1)+ 4*ii + jj;
        size_t this_state_idx = state_dim*(ii+1)+ 10 + jj;  // skip ii=0, skip position,linvel,quat states
        // -s_ik <= 0
        linear_con_mat.coeffRef(row_idx, s_slack_var_idx) = -1.0;
        upper_bound(row_idx) = 0;
        row_idx++;
        // -x_ik -s_ik <= 0
        linear_con_mat.coeffRef(row_idx, this_state_idx) = -1.0;
        linear_con_mat.coeffRef(row_idx, s_slack_var_idx) = -1.0;
        upper_bound(row_idx) = 0;
        row_idx++;
        // x_ik -s_ik <= 0
        linear_con_mat.coeffRef(row_idx, this_state_idx) = 1.0;
        linear_con_mat.coeffRef(row_idx, s_slack_var_idx) = -1.0;
        upper_bound(row_idx) = 0;
        row_idx++;
      }
      // sum(s_ik) - zk <= v_max
      for (size_t jj = 0; jj < ang_vel_dim; jj++) {
        size_t s_slack_var_idx = state_dim*N+control_dim*(N-1)+
          num_force_norm_slack_vars+num_moment_norm_slack_vars+state_bd_dim*(N-1)+state_bd_dim*(N-1)+
          4*(N-1) + 4*ii + jj;
        linear_con_mat.coeffRef(row_idx, s_slack_var_idx) = 1.0;
      }
      linear_con_mat.coeffRef(row_idx, z_slack_var_idx) = -1.0;
      upper_bound(row_idx) = desired_vel_;
      row_idx++;
    }
  }

  if (enforce_obs_avoidance_const) {
    if (keep_out_zones_.size() == 0) {
      std::cout << "[TOP::SetSimpleConstraints] No keep out zones specified. Skipping obstacle avoidance constraints."
                << std::endl;
      return;
    }
    if (keep_out_zones_.size() > 1) {
      std::cout << "[TOP::SetSimpleConstraints] Can only account for 1 keep out zone currently. Found "
                << std::to_string(keep_out_zones_.size()) << std::endl;
      throw std::runtime_error("Can only account for 1 keep out zone currently. Found " +
                               std::to_string(keep_out_zones_.size()));
    }
    Eigen::AlignedBox3d box = keep_out_zones_[0];
    Eigen::Vector3d ko_min_original = box.min();
    Eigen::Vector3d ko_max_original = box.max();
    std::cout << "[TOP::SetSimpleConstraints] original ko_min: " << ko_min_original.transpose() << std::endl;
    std::cout << "[TOP::SetSimpleConstraints] original ko_max: " << ko_max_original.transpose() << std::endl;
    // Add buffer to obstacle
    Eigen::Vector3d ko_min = ko_min_original - Eigen::Vector3d(obs_clearance, obs_clearance, obs_clearance);
    Eigen::Vector3d ko_max = ko_max_original + Eigen::Vector3d(obs_clearance, obs_clearance, obs_clearance);

    // Clip ko_min and ko_max to be within pose min and max
    ko_min = ko_min.cwiseMax(MinPos());  // clip ko_min to be >= pose min
    ko_max = ko_max.cwiseMin(MaxPos());  // clip ko_max to be <= pose max

    std::cout << "[TOP::SetSimpleConstraints] pose min: " << MinPos().transpose() << std::endl;
    std::cout << "[TOP::SetSimpleConstraints] pose max: " << MaxPos().transpose() << std::endl;

    // Print updated ko_min and ko_max
    std::cout << "[TOP::SetSimpleConstraints] ko_min: " << ko_min.transpose() << std::endl;
    std::cout << "[TOP::SetSimpleConstraints] ko_max: " << ko_max.transpose() << std::endl;
    Eigen::Vector3d ko_center = (ko_min + ko_max)/2;
    std::cout << "[TOP::SetSimpleConstraints] ko_center: " << ko_center.transpose() << std::endl;
    for (size_t ii = 0; ii < N-1; ii++) {
      for (size_t jj = 0; jj < 3; jj++) {
        decimal_t lb = MinPos()[jj];
        decimal_t ub = MaxPos()[jj];
        // lb < x < ub
        // Either ko_max < x < ub or lb < x < ko_min
        bool active_proj = true;
        for (size_t kk = 0; kk < 3; kk++) {
          if (kk == jj) {
            continue;
          }
          if ((Xprev[ii](kk) > ko_max[kk]) || (Xprev[ii](kk) < ko_min[kk])) {
            // not active projection
            active_proj = false;
            // std::cout << "Seeing x y z " << std::to_string(Xprev[ii](0)) << ", " << std::to_string(Xprev[ii](1)) <<
            // ", " << std::to_string(Xprev[ii](2)) << " and judging that no constraint is required." << std::endl;
            break;
          }
        }
        if (active_proj) {
          if (Xprev[ii](jj) >= ko_center[jj]) {
            lb = ko_max[jj];
          } else {
            ub = ko_min[jj];
          }
          // std::cout << "Seeing x y z " << std::to_string(Xprev[ii](0)) << ", " << std::to_string(Xprev[ii](1)) << ",
          // " << std::to_string(Xprev[ii](2)) << " and judging that " << std::to_string(jj) << " needs CONSTRAINT " <<
          // std::to_string(lb) << ", " << std::to_string(ub) << "." << std::endl;
        }
        // lb < x < ub
        linear_con_mat.coeffRef(row_idx, state_dim*ii + jj) = 1.0;
        lower_bound(row_idx) = lb;
        upper_bound(row_idx) = ub;
        row_idx++;
      }
    }

    // decimal_t ko_x_min = ko_min(0);
    // decimal_t ko_x_max = ko_max(0);
    // decimal_t ko_y_min = ko_min(1);
    // decimal_t ko_y_max = ko_max(1);
    // decimal_t ko_center_x = (ko_x_max + ko_x_min)/2;
    // decimal_t ko_center_y = (ko_y_max + ko_y_min)/2;
    // std::cout << "ko_center_x: " << ko_center_x << std::endl;
    // std::cout << "ko_center_y: " << ko_center_y << std::endl;
    // for (size_t ii = 0; ii < N-1; ii++) {
    //   for (size_t kk = 0; kk < 2; kk++) {  // x and y
    //     size_t d_slack_var_idx = state_dim*N+control_dim*(N-1)+
    //     num_force_norm_slack_vars+num_moment_norm_slack_vars+
    //     state_bd_dim*(N-1)+state_bd_dim*(N-1)+
    //     (lin_vel_dim+1)*(N-1)+(ang_vel_dim+1)*(N-1)+2*ii+kk;
    //     // -d_ik <= 0
    //     linear_con_mat.coeffRef(row_idx, d_slack_var_idx) = -1.0;
    //     upper_bound(row_idx) = 0;
    //     row_idx++;
    //     // Using prev, determine active constraints
    //     decimal_t prev = Xprev[ii](kk);
    //     if (prev > (ko_min(kk) + ko_max(kk)) / 2) {  // If x > ko_center_x, then -x + d_i <= -ko_x_max
    //       linear_con_mat.coeffRef(row_idx, state_dim*(ii+1)+kk) = -1.0;
    //       linear_con_mat.coeffRef(row_idx, d_slack_var_idx) = 1.0;
    //       upper_bound(row_idx) = -ko_max(kk);
    //       row_idx++;
    //     } else {  // If x < ko_center_x, then x + d_i <= ko_x_min
    //       linear_con_mat.coeffRef(row_idx, state_dim*(ii+1)+kk) = 1.0;
    //       linear_con_mat.coeffRef(row_idx, d_slack_var_idx) = 1.0;
    //       upper_bound(row_idx) = ko_min(kk);
    //       row_idx++;
    //     }
    //   }
    // }
  }

  if (enforce_state_bounds) {
    for (size_t ii = 0; ii < N; ii++) {
      for (size_t jj = 0; jj < 3; jj++) {  // x, y, z only for now
        linear_con_mat.coeffRef(row_idx, ii * state_dim + jj) = 1.0;
        lower_bound(row_idx) = MinPos()[jj];
        upper_bound(row_idx) = MaxPos()[jj];
        // std::cout << "Setting state bounds for state " << ii << " dim " << jj << " to " << MinPos()[jj] << " and "
        //           << MaxPos()[jj] << std::endl;
        ++row_idx;
      }
    }
  }

  size_t num_vars = GetNumTOPVariables();
  size_t num_cons = GetNumTOPConstraints();

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
  PrettyPrintConstraints();
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
  size_t num_cons = GetNumTOPConstraints();
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

  // std::cout << "Setting gradient" << std::endl;

  // // Gradient to keep states away from obstacle
  // if (enforce_obs_avoidance_const) {
  //   if (Xprev.size() == N) {
  //     for (size_t ii = 0; ii < N; ii++) {
  //       Vec3 point = Xprev[ii].segment(0, 3);
  //       decimal_t dist = ComputeSignedDistance(point);
  //       Vec3 sd_grad = ComputeSignedDistanceGradient(point);
  //       gradient(ii * state_dim + 0) = sd_grad[0];
  //       gradient(ii * state_dim + 1) = sd_grad[1];
  //       gradient(ii * state_dim + 2) = sd_grad[2];
  //     }
  //   }
  // }
}

bool TOP::Solve() {
  solved_ = false;
  ResetSCPParams();
  UpdateProblemDimension(N);
  std::cout << "[TOP::Solve] Updated problem dimension" << std::endl;
  std::cout << "[TOP::Solve] linear_con_mat size: " << linear_con_mat.rows() << " x " << linear_con_mat.cols()
            << std::endl;
  bool add_custom_keep_out_zone = true;

  std::cout << "[TOP::Solve] start of init traj is " << Xprev[0].transpose() << std::endl;
  std::cout << "[TOP::Solve] end of init traj is " << Xprev[N-1].transpose() << std::endl;

  std::cout << "[TOP::Solve] mass: " << mass << std::endl;
  std::cout << "[TOP::Solve] inertia: " << J << std::endl;

  std::cout << "[TOP::Solve] desired accel: " << desired_accel_ << std::endl;

  std::cout << "[TOP::Solve] Keep in zones: " << std::endl;
  for (size_t i = 0; i < keep_in_zones_.size(); ++i) {
    std::cout << "Zone " << i << std::endl;
    std::cout << "min: " << keep_in_zones_[i].min().transpose() << std::endl;
    std::cout << "max: " << keep_in_zones_[i].max().transpose() << std::endl;
  }
  std::cout << "[TOP::Solve] Keep out zones: " << std::endl;
  for (size_t i = 0; i < keep_out_zones_.size(); ++i) {
    std::cout << "Zone " << i << std::endl;
    std::cout << "min: " << keep_out_zones_[i].min().transpose() << std::endl;
    std::cout << "max: " << keep_out_zones_[i].max().transpose() << std::endl;
  }


  // // Print init-traj states
  // std::cout << "After init traj straight line: " << std::endl;
  // for (size_t jj = 0; jj < N-1; jj++) {
  //     std::cout << "Quaternion and angular velocity at time " << jj << std::endl;
  //     for (size_t kk = state_dim_lin; kk < state_dim ; kk++){
  //       std::cout << Xprev[jj](kk) << " " ;
  //     }
  //     std::cout << std::endl;
  // }
  // std::cout << std::endl;

  // TODO(somrita): Reset max_iter
  max_iter = 1;
  for (size_t kk = 0; kk < max_iter; kk++) {
    // SetSimpleConstraints();
    // SetSimpleCosts();

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

    // Update cached solution
    for (size_t ii = 0; ii < N; ii++) {
      Xprev[ii] = qp_soln.block(state_dim*ii, 0, state_dim, 1);
    }
    for (size_t ii = 0; ii < N-1; ii++) {
      Uprev[ii] = qp_soln.block(state_dim*N + control_dim*ii, 0, control_dim, 1);
    }

    NormalizeQuaternions();

    ValidationChecks();

    bool printEveryIter = false;
    if (printEveryIter) {
      // Print statements
      for (size_t jj = 0; jj < N-1; jj++) {
        std::cout << "Quaternion and angular velocity at time " << jj << std::endl;
        for (size_t kk = state_dim_lin; kk < state_dim ; kk++) {
          std::cout << Xprev[jj](kk) << " ";
        }
        std::cout << std::endl;
        std::cout << "Control variables at time " << jj << std::endl;
        for (size_t kk = 0; kk < control_dim; kk++) {
          // if (kk >=3 && Uprev[jj](kk) != 0){
          //   std::cout << "Non-zero moment" << Uprev[jj](kk) << " " ;
          // }
          std::cout << Uprev[jj](kk) << " ";
        }
        std::cout << std::endl;
        std::cout << "Same thing another way " << jj << std::endl;
        for (size_t kk = 0; kk < control_dim; kk++) {
          std::cout << qp_soln(state_dim*N + control_dim*jj + kk) << " ";
        }
        std::cout << std::endl;
        // std::cout << "Slack control variables at time " << jj << std::endl;
        // for (size_t kk = 0; kk < control_dim; kk++){
        //   std::cout << qp_soln(state_dim*N + control_dim*(N-1) + control_dim*jj + kk) << " " ;
        // }
        std::cout << std::endl;
      }
      std::cout << std::endl;
    }
    // if (solved_ && state_ineq_con_satisfied) {
    //   return true;
    // }
    if (solved_) {
      return true;
    }
  }

  // if (SatisfiesStateInequalityConstraints()) {
  //   solved_ = true;
  //   return true;
  // } else {
  //   solved_ = false;
  //   return false;
  // }
  return solved_;
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
  if (keep_out_zones_.size() != 1) {
    std::cout << "Obstacle avoidance constraint:    NOT CHECKED" << std::endl;
  } else {
    Eigen::AlignedBox3d box = keep_out_zones_[0];
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

// NOTE: Functions below this point are not currently being used but may be good for future modularization.
/*
// void TOP::ComputeSignedDistances() {
//   size_t n_obs = keep_out_zones_->size();

//   collision_checker::SignedDistanceResult sd_result;
//   for (size_t ii = 0; ii < N-1; ii++) {
//     for (size_t jj = 0; jj < n_obs; jj++) {
//       // signed distance, point on co1, point on co2;
//       cc.ComputeDistance(jj, sd_result);
//       Vec3 nhat;
//       if (sd_result.sd >  0) {
//         nhat = (sd_result.co1_pt-sd_result.co2_pt);
//       } else {
//         nhat = (sd_result.co2_pt-sd_result.co1_pt);
//       }
//       nhat.normalize();
//       obs_ub[n_obs*ii+jj] = sd_result.sd - nhat.dot(Xprev[ii].segment(0, 3)) - obs_clearance;
//       support_vectors[n_obs*ii+jj] = -nhat;
//     }
//   }
// }

// void TOP::SetHessianMatrix() {
//   size_t num_vars = GetNumTOPVariables();
//   Qf.diagonal() << 1000, 1000, 1000, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
//   R.diagonal() << 1, 1, 1, 1, 1, 1;

//   int idx = state_dim*N;
//   for (size_t ii = 0; ii < N-1; ii++) {
//     for (size_t jj = 0; jj < control_dim; jj++) {
//       // hessian.insert(idx+jj, idx+jj) = R.diagonal()[jj];
//     }
//     idx += control_dim;
//   }

//   for (size_t ii = 0; ii < num_vars; ii++) hessian.insert(ii, ii) = 0.0;
// }

// void TOP::SetGradient() {
//   // Only slack variables associated with max(g(x),0) are penalized

//   size_t row_idx = state_dim*N + 2*control_dim*(N-1);

//   // Penalty for state upper and lower bounds
//   for (size_t ii = 0; ii < 2*state_bd_dim*(N-1); ii++) {
//     gradient(row_idx) = omega;
//     row_idx++;
//   }

//   // Penalty for linear velocity violation
//   for (size_t ii = 0; ii < N-1; ii++) {
//     gradient(row_idx + 3) = omega;
//     row_idx += 4;
//   }

//   // Penalty for angular velocity violation
//   for (size_t ii = 0; ii < N-1; ii++) {
//     gradient(row_idx + 3) = omega;
//     row_idx += 4;
//   }

//   // Penalty for trust region constraint violation
//   for (size_t ii = 0; ii < N-1; ii++) {
//     gradient(row_idx + state_dim) = omega;
//     row_idx += (state_dim+1);
//   }

//   // Penalty for collision avoidance
// }

// void TOP::SetBoundaryCons() {
//   lower_bound.segment(0, state_dim) = x0;
//   upper_bound.segment(0, state_dim) = x0;
//   for (size_t ii = 0; ii < state_dim; ii++) {
//     linear_con_mat.insert(ii, ii) = 1;
//   }

//   lower_bound.segment(state_dim, state_dim) = xg;
//   upper_bound.segment(state_dim, state_dim) = xg;
//   for (size_t ii = 0; ii < state_dim; ii++) {
//     linear_con_mat.insert(state_dim+ii, state_dim*(N-1)+ii) = 1;
//   }
// }

// void TOP::SetControlCons() {
//   size_t row_idx = 2*state_dim;

//   // col_idx tracks the slack variable under consideration
//   size_t col_idx = state_dim*N + control_dim*(N-1);

//   for (size_t ii = 0; ii < N-1; ii++) {
//     // Linear acceleration
//     // -sik-aik <= 0
//     upper_bound.segment(row_idx, 3).setZero();
//     for (size_t jj = 0; jj < 3; jj++) {
//       linear_con_mat.insert(row_idx+jj, state_dim*N + control_dim*ii + jj) = -1;
//       linear_con_mat.insert(row_idx+jj, col_idx+jj) = -1;
//     }

//     row_idx += 3;

//     // aik-sik <= 0
//     upper_bound.segment(row_idx, 3).setZero();
//     for (size_t jj = 0; jj < 3; jj++) {
//       linear_con_mat.insert(row_idx+jj, state_dim*N + control_dim*ii + jj) = 1;
//       linear_con_mat.insert(row_idx+jj, col_idx+jj) = -1;
//     }

//     row_idx += 3;

//     // sum(sik) <= a_max
//     upper_bound(row_idx) = mass*desired_accel_;
//     for (size_t jj = 0; jj < 3; jj++) {
//       linear_con_mat.insert(row_idx, col_idx+jj) = 1;
//     }

//     row_idx++;
//     col_idx += 3;

//     // Angular acceleration
//     // -sik-alpha_ik <= 0
//     upper_bound.segment(row_idx, 3).setZero();
//     for (size_t jj = 0; jj < 3; jj++) {
//       linear_con_mat.insert(row_idx+jj, state_dim*N + control_dim*ii + 3 + jj) = -1;
//       linear_con_mat.insert(row_idx+jj, col_idx+jj) = -1;
//     }

//     row_idx += 3;

//     // alpha_ik - sik <= 0
//     upper_bound.segment(row_idx, 3).setZero();
//     for (size_t jj = 0; jj < 3; jj++) {
//       linear_con_mat.insert(row_idx+jj, state_dim*N + control_dim*ii + 3 + jj) = 1;
//       linear_con_mat.insert(row_idx+jj, col_idx+jj) = -1;
//     }

//     row_idx += 3;

//     // sum(sik) <= alpha_max
//     Vec3 alpha_;
//     alpha_.setOnes();
//     alpha_ *= desired_alpha_;
//     Vec3 M_ = J*alpha_;
//     upper_bound(row_idx) = M_.minCoeff();
//     for (size_t jj = 0; jj < 3; jj++) {
//       linear_con_mat.insert(row_idx, col_idx+jj) = 1;
//     }

//     row_idx++;
//     col_idx += 3;
//   }

//   // Slack variables non-negative constraints
//   upper_bound.segment(row_idx, control_dim*(N-1)).setZero();
//   col_idx = state_dim*N + control_dim*(N-1);
//   for (size_t ii = 0; ii < control_dim*(N-1); ii++) {
//     linear_con_mat.insert(row_idx+ii, col_idx+ii) = -1;
//   }
// }

// void TOP::SetStateCons() {
//   // State LB
//   size_t row_idx = 2*state_dim + 20*(N-1);

//   // col_idx tracks the slack variable under consideration
//   size_t col_idx = state_dim*N + 2*control_dim*(N-1);
//   for (size_t ii = 0; ii < N-1; ii++) {
//     // State LB
//     for (size_t jj = 0; jj < 3; jj++) {
//       // Position limits: -xik-zik <= -x_min_i
//       upper_bound(row_idx+jj) = -x_min(jj);

//       linear_con_mat.insert(row_idx+jj, state_dim*(ii+1)+jj) = -1;
//       linear_con_mat.insert(row_idx+jj, col_idx+jj) = -1;
//     }

//     row_idx += 3;
//     col_idx += 3;

//     for (size_t jj = 0; jj < 4; jj++) {
//       // Quaternion limits: -xik-zik <= -x_min_i
//       upper_bound(row_idx+jj) = -x_min(6+jj);

//       linear_con_mat.insert(row_idx+jj, state_dim*(ii+1)+6+jj) = -1;
//       linear_con_mat.insert(row_idx+jj, col_idx+jj) = -1;
//     }

//     row_idx += 4;
//     col_idx += 4;
//   }

//   // State UB
//   for (size_t ii = 0; ii < N-1; ii++) {
//     for (size_t jj = 0; jj < 3; jj++) {
//       // Position limits: xik-zik <= x_max_i
//       upper_bound(row_idx+jj) = x_max(jj);

//       linear_con_mat.insert(row_idx+jj, state_dim*(ii+1)+jj) = 1;
//       linear_con_mat.insert(row_idx+jj, col_idx+jj) = -1;
//     }

//     row_idx += 3;
//     col_idx += 3;

//     for (size_t jj = 0; jj < 4; jj++) {
//       // Quaternion limits: xik-zik <= x_max_i
//       upper_bound(row_idx+jj) = x_max(6+jj);

//       linear_con_mat.insert(row_idx+jj, state_dim*(ii+1)+6+jj) = 1;
//       linear_con_mat.insert(row_idx+jj, col_idx+jj) = -1;
//     }

//     row_idx += 4;
//     col_idx += 4;
//   }

//   // Slack variables non-negative constraints
//   upper_bound.segment(row_idx, 2*state_bd_dim*(N-1)).setZero();
//   if (state_con_strict) {
//     lower_bound.segment(row_idx, 2*state_bd_dim*(N-1)).setZero();
//   }

//   col_idx = state_dim*N + 2*control_dim*(N-1);
//   for (size_t ii = 0; ii < 2*state_bd_dim*(N-1); ii++) {
//     linear_con_mat.insert(row_idx+ii, col_idx+ii) = -1;
//   }
// }

// void TOP::SetVelCons() {
//   size_t row_idx = 2*state_dim + 20*(N-1) + 4*state_bd_dim*(N-1);

//   // col_idx tracks the slack variable under consideration
//   size_t col_idx = state_dim*N + 2*control_dim*(N-1) + 2*state_bd_dim*(N-1);

//   // Linear velocity constraints
//   for (size_t ii = 0; ii < N-1; ii++) {
//     // -sk-vk <= 0.
//     upper_bound.segment(row_idx, 3).setZero();
//     for (size_t jj = 0; jj < 3; jj++) {
//       linear_con_mat.insert(row_idx+jj, state_dim*(ii+1)+3+jj) = -1;
//       linear_con_mat.insert(row_idx+jj, col_idx+jj) = -1;
//     }

//     row_idx += 3;

//     // vk-sk <= 0.
//     upper_bound.segment(row_idx, 3).setZero();
//     for (size_t jj = 0; jj < 3; jj++) {
//       linear_con_mat.insert(row_idx+jj, state_dim*(ii+1)+3+jj) = 1;
//       linear_con_mat.insert(row_idx+jj, col_idx+jj) = -1;
//     }

//     row_idx += 3;

//     // sum(sik) - zk <= v_max
//     upper_bound(row_idx) = desired_vel_;
//     for (size_t jj = 0; jj <3; jj++) {
//       linear_con_mat.insert(row_idx, col_idx+jj) = 1;
//     }
//     linear_con_mat.insert(row_idx, col_idx+3) = -1;

//     row_idx++;
//     col_idx += 4;
//   }

//   // Slack variables non-negative constraints
//   col_idx = state_dim*N + 2*control_dim*(N-1) + 2*state_bd_dim*(N-1);
//   upper_bound.segment(row_idx, 4*(N-1)).setZero();
//   if (lin_vel_strict) {
//     // slack vars are set to 0 if constraint is to be strictly enforced
//     lower_bound.segment(row_idx, 4*(N-1)).setZero();
//   }
//   for (size_t ii = 0; ii < 4*(N-1); ii++) {
//     linear_con_mat.insert(row_idx+ii, col_idx+ii) = -1;
//   }
// }

// void TOP::SetAngVelCons() {
//   size_t row_idx = 2*state_dim + 20*(N-1) + 4*state_bd_dim*(N-1) + 11*(N-1);

//   // col_idx tracks the slack variable under consideration
//   size_t col_idx = state_dim*N + 2*control_dim*(N-1) + 2*state_bd_dim*(N-1) + 4*(N-1);

//   for (size_t ii = 0; ii < N-1; ii++) {
//     // -sk-wk <= 0.
//     upper_bound.segment(row_idx, 3).setZero();
//     for (size_t jj = 0; jj < 3; jj++) {
//       linear_con_mat.insert(row_idx+jj, state_dim*(ii+1)+10+jj) = -1;
//       linear_con_mat.insert(row_idx+jj, col_idx+jj) = -1;
//     }

//     row_idx += 3;

//     upper_bound.segment(row_idx, 3).setZero();
//     // wk-sk <= 0.
//     for (size_t jj = 0; jj < 3; jj++) {
//       linear_con_mat.insert(row_idx+jj, state_dim*(ii+1)+10+jj) = 1;
//       linear_con_mat.insert(row_idx+jj, col_idx+jj) = -1;
//     }

//     row_idx += 3;

//     // sum(sik) - zk <= v_max
//     upper_bound(row_idx) = desired_omega_;
//     for (size_t jj = 0; jj < 3; jj++) {
//       linear_con_mat.insert(row_idx, col_idx+jj) = 1;
//     }
//     linear_con_mat.insert(row_idx, col_idx+3) = -1;

//     row_idx++;
//     col_idx += 4;
//   }

//   // Slack variables non-negative constraints
//   col_idx = state_dim*N + 2*control_dim*(N-1) + 2*state_bd_dim*(N-1) + 4*(N-1);
//   upper_bound.segment(row_idx, 4*(N-1)).setZero();
//   if (ang_vel_strict) {
//     // slack vars are set to 0 if constraint is to be strictly enforced
//     lower_bound.segment(row_idx, 4*(N-1)).setZero();
//   }
//   for (size_t ii = 0; ii < 4*(N-1); ii++) {
//     linear_con_mat.insert(row_idx+ii, col_idx+ii) = -1;
//   }
// }

// void TOP::SetLinearDynamicsCons() {
//   size_t row_idx = 2*state_dim + 20*(N-1) + 4*state_bd_dim*(N-1) + 2*11*(N-1);
//   row_idx = 2*state_dim + 20*(N-1) + 4*state_bd_dim*(N-1);
//   size_t control_dim_lin = static_cast<size_t>(control_dim/2);

//   // Ak*xk+Bk*uk - x_{k+1} = 0
//   for (size_t ii = 0; ii < N-1; ii++) {
//     for (size_t jj = 0; jj < state_dim_lin; jj++) {
//       linear_con_mat.coeffRef(row_idx+jj, state_dim*(ii+1)+jj) = -1.0;

//       for (size_t kk = 0; kk < state_dim_lin; kk++) {
//         if (Ak_di(jj, kk) == 0) {
//           continue;
//         }

//         linear_con_mat.coeffRef(row_idx+jj, state_dim*ii+kk)     = Ak_di(jj, kk);
//       }

//       for (size_t kk = 0; kk < control_dim_lin; kk++) {
//         if (Bk_di(jj, kk) == 0) {
//           continue;
//         }

//         linear_con_mat.coeffRef(row_idx+jj, state_dim*N+control_dim*ii+kk) = Bk_di(jj, kk);
//       }
//     }

//     lower_bound.segment(row_idx, state_dim_lin).setZero();
//     upper_bound.segment(row_idx, state_dim_lin).setZero();
//     row_idx += state_dim_lin;
//   }
// }

// void TOP::SetDynamicsCons() {
//   Mat7 eye;
//   eye.setIdentity();

//   Mat7 Ak;
//   Mat7 Akp1;
//   Mat7x3 Bk;
//   Mat7x3 Bkp1;
//   Vec7 ck;
//   Vec7 Xprev_k, Xprev_kp1;
//   Vec7 fk, fkp1;
//   Vec3 Uprev_k, Uprev_kp1;

//   size_t row_idx = 2*state_dim + 20*(N-1) + 4*state_bd_dim*(N-1) + 2*11*(N-1) + state_dim_lin*(N-1);
//   row_idx = 2*state_dim + 20*(N-1) + 4*state_bd_dim*(N-1) + state_dim_lin*(N-1);
//   size_t control_dim_nlin = static_cast<size_t>(control_dim/2);

//   // Trapezoidal integration for ii = 0,..,N-3
//   for (size_t ii = 0; ii < N-2; ii++) {
//     // Assign Ak matrices
//     Ak    = 0.5*dh*As[ii]+eye;
//     Akp1  = 0.5*dh*As[ii+1]-eye;

//     // Assign Bk matrices
//     Bk    = 0.5*dh*Bs[ii];
//     Bkp1  = 0.5*dh*Bs[ii+1];

//     for (size_t jj = 0; jj < state_dim_nlin; jj++) {
//       for (size_t kk = 0; kk < state_dim_nlin; kk++) {
//           linear_con_mat.coeffRef(row_idx+jj, state_dim*ii+kk)     = Ak(jj, kk);
//           linear_con_mat.coeffRef(row_idx+jj, state_dim*(ii+1)+kk) = Akp1(jj, kk);
//       }

//       for (size_t kk = 0; kk < control_dim_nlin; kk++) {
//         // add +3 to column index to grab (Mx,My,Mz) component
//         linear_con_mat.coeffRef(row_idx+jj, state_dim*N+control_dim*ii+3+kk)      = Bk(jj, kk);
//         linear_con_mat.coeffRef(row_idx+jj, state_dim*N+control_dim*(ii+1)+3+kk)  = Bkp1(jj, kk);
//       }
//     }

//     // Assign ck vectors
//     Xprev_k = Xprev[ii].segment(6, 7);
//     Xprev_kp1 = Xprev[ii+1].segment(6, 7);
//     Uprev_k = Uprev[ii].segment(3, 3);
//     Uprev_kp1 = Uprev[ii+1].segment(3, 3);
//     fk = fs[ii];
//     fkp1 = fs[ii+1];

//     ck = 0.5*dh*(
//       As[ii]*Xprev_k + Bs[ii]*Uprev_k
//       + As[ii+1]*Xprev_kp1 + Bs[ii+1]*Uprev_kp1
//       - fk - fkp1);
//     lower_bound.segment(row_idx, state_dim_nlin) = ck;
//     upper_bound.segment(row_idx, state_dim_nlin) = ck;

//     row_idx += state_dim_nlin;
//   }

//   // Euler integration for last step
//   Ak = dh*As[N-2] + eye;
//   Akp1 = -eye;
//   Bk = dh*Bs[N-2];
//   fk = fs[N-2];
//   Xprev_k = Xprev[N-2].segment(6, 7);
//   Uprev_k = Uprev[N-2].segment(3, 3);

//   ck = dh*(As[N-2]*Xprev_k + Bs[N-2]*Uprev_k - fk);
//   for (size_t jj = 0; jj < state_dim_nlin; jj++) {
//     for (size_t kk = 0; kk < state_dim_nlin; kk++) {
//       linear_con_mat.coeffRef(row_idx+jj, state_dim*(N-2)+kk)      = Ak(jj, kk);
//       linear_con_mat.coeffRef(row_idx+jj, state_dim*(N-1)+kk)      = Akp1(jj, kk);
//     }
//     for (size_t kk = 0; kk < control_dim_nlin; kk++) {
//       linear_con_mat.coeffRef(row_idx+jj, state_dim*N+control_dim*(N-2)+kk)   = Bk(jj, kk);
//     }
//   }
//   lower_bound.segment(row_idx, state_dim_nlin) = ck;
//   upper_bound.segment(row_idx, state_dim_nlin) = ck;
// }

// void TOP::SetTrustRegionCons() {
//   size_t row_idx = 2*state_dim + 20*(N-1) + 4*state_bd_dim*(N-1) + 2*11*(N-1) +
//     state_dim*(N-1);
//   size_t col_idx = state_dim*N + 2*control_dim*(N-1) + 2*state_bd_dim*(N-1) + 2*4*(N-1);

//   // Ordering of slack variables here is (s1,z1,...,s_{N-1},z_{N-1})
//   // where s1 \in R^{n_x} and z1 \in R

//   for (size_t ii = 0; ii < N-1; ii++) {
//     // -sk-Xk <= -Xkp
//     upper_bound.segment(row_idx, state_dim) = -Xprev[ii+1];
//     for (size_t jj = 0; jj < state_dim; jj++) {
//       linear_con_mat.insert(row_idx+jj, state_dim*(ii+1)+jj) = -1;
//       linear_con_mat.insert(row_idx+jj, col_idx+jj) = -1;
//     }

//     row_idx += (state_dim);

//     // Xk-sk <= Xkp
//     upper_bound.segment(row_idx, state_dim) = Xprev[ii+1];
//     for (size_t jj = 0; jj < state_dim; jj++) {
//       linear_con_mat.insert(row_idx+jj, state_dim*(ii+1)+jj) = 1;
//       linear_con_mat.insert(row_idx+jj, col_idx+jj) = -1;
//     }

//     row_idx += (state_dim);

//     // \sum(sk)-zk \leq Delta
//     upper_bound(row_idx) = Delta;
//     for (size_t jj = 0; jj < state_dim; jj++) {
//       linear_con_mat.insert(row_idx, col_idx+jj) = 1;
//     }
//     linear_con_mat.insert(row_idx, col_idx+state_dim) = -1;

//     row_idx++;
//     col_idx += (state_dim+1);
//   }

//   // Slack variables non-negative constraints
//   col_idx = state_dim*N + 2*control_dim*(N-1) + 2*state_bd_dim*(N-1) + 2*4*(N-1);
//   upper_bound.segment(row_idx, (state_dim+1)*(N-1)).setZero();
//   for (size_t ii = 0; ii < (N-1)*(state_dim+1); ii++) {
//     linear_con_mat.insert(row_idx, col_idx+ii) = -1;
//     row_idx++;
//   }
// }

// void TOP::SetObsCons() {
//   size_t row_idx = 2*state_dim + 20*(N-1) + 4*7*(N-1) + 2*11*(N-1) +
//     state_dim*(N-1) + (3*state_dim+2)*(N-1);
//   size_t col_idx = state_dim*N + 2*control_dim*(N-1) +
//     2*7*(N-1) + 2*4*(N-1)  + (state_dim+1)*(N-1);
//   size_t n_obs = keep_out_zones_->size();

//   Vec3 support_vec;

//   for (size_t ii = 0; ii < N-1; ii++) {
//     for (size_t jj = 0; jj < n_obs; jj++) {
//       support_vec = support_vectors[n_obs*ii+jj];

//       upper_bound(row_idx) = obs_ub[n_obs*ii+jj];
//       linear_con_mat.insert(row_idx, col_idx) = -1;    // -z_{k,m}
//       for (size_t kk = 0; kk < 3; kk++) {
//         linear_con_mat.coeffRef(row_idx, state_dim*(ii+1)+kk) = support_vec(kk);
//       }

//       row_idx++;
//       col_idx++;
//     }
//   }

//   // Slack variables non-negative constraints
//   col_idx = state_dim*N + control_dim*(N-1) +
//     2*7*(N-1) + 2*4*(N-1)  + (state_dim+1)*(N-1);
//   upper_bound.segment(row_idx, n_obs*(N-1)).setZero();
//   for (size_t ii = 0; ii < n_obs*(N-1); ii++) {
//     linear_con_mat.insert(row_idx, col_idx) = -1;
//     row_idx++;
//     col_idx++;
//   }
// }

// void TOP::UpdateBoundaryCons() {
//   lower_bound.segment(0, state_dim) = x0;
//   upper_bound.segment(0, state_dim) = x0;

//   lower_bound.segment(state_dim, state_dim) = xg;
//   upper_bound.segment(state_dim, state_dim) = xg;
// }

// void TOP::UpdateControlCons() {
//   size_t row_idx = 2*state_dim;

//   for (size_t ii = 0; ii < N-1; ii++) {
//     // Linear accelereation
//     row_idx += 6;
//     upper_bound(row_idx) = mass*desired_accel_;

//     row_idx++;

//     // Angular acceleration
//     row_idx += 6;
//     Vec3 alpha_;
//     alpha_.setOnes();
//     alpha_ *= desired_alpha_;
//     Vec3 M_ = J*alpha_;
//     upper_bound(row_idx) = M_.minCoeff();

//     row_idx++;
//   }
// }

// void TOP::UpdateStateCons() {
//   // Update any changes to params
//   x_max << pos_max_(0), pos_max_(1), pos_max_(2),
//     desired_vel_/std::sqrt(3), desired_vel_/std::sqrt(3), desired_vel_/std::sqrt(3),
//     1, 1, 1, 1,
//     desired_omega_/std::sqrt(3), desired_omega_/std::sqrt(3), desired_omega_/std::sqrt(3);
//   x_min = -x_max;
//   x_min(0) = pos_min_(0);
//   x_min(1) = pos_min_(1);
//   x_min(2) = pos_min_(2);

//   size_t row_idx = 2*state_dim + 20*(N-1);

//   // State LB
//   for (size_t ii = 0; ii < N-1; ii++) {
//     // Position limits
//     for (size_t jj = 0; jj < 3; jj++) {
//       upper_bound(row_idx+jj) = -x_min(jj);
//     }

//     row_idx += 3;

//     // Quaternion limits
//     for (size_t jj = 0; jj < 4; jj++) {
//       upper_bound(row_idx+jj) = -x_min(6+jj);
//     }

//     row_idx += 4;
//   }

//   // State UB
//   for (size_t ii = 0; ii < N-1; ii++) {
//     for (size_t jj = 0; jj < 3; jj++) {
//       upper_bound(row_idx+jj) = x_max(jj);
//     }

//     row_idx += 3;

//     for (size_t jj = 0; jj < 4; jj++) {
//       upper_bound(row_idx+jj) = x_max(6+jj);
//     }

//     row_idx += 4;
//   }

//   // Slack variables non-negative constraints
//   upper_bound.segment(row_idx, 2*state_bd_dim*(N-1)).setZero();
//   if (state_con_strict) {
//     lower_bound.segment(row_idx, 2*state_bd_dim*(N-1)).setZero();
//   }
// }

// void TOP::UpdateVelCons() {
//   size_t row_idx = 2*state_dim + 20*(N-1) + 4*state_bd_dim*(N-1);

//   // Linear velocity constraints
//   for (size_t ii = 0; ii < N-1; ii++) {
//     row_idx += 6;

//     upper_bound(row_idx) = desired_vel_;

//     row_idx++;
//   }

//   // Slack variables non-negative constraints
//   upper_bound.segment(row_idx, 4*(N-1)).setZero();
//   if (lin_vel_strict) {
//     // slack vars are set to 0 if constraint is to be strictly enforced
//     lower_bound.segment(row_idx, 4*(N-1)).setZero();
//   }
// }

// void TOP::UpdateAngVelCons() {
//   size_t row_idx = 2*state_dim + 20*(N-1) + 4*state_bd_dim*(N-1) + 11*(N-1);

//   for (size_t ii = 0; ii < N-1; ii++) {
//     row_idx += 6;

//     upper_bound(row_idx) = desired_omega_;

//     row_idx++;
//   }

//   upper_bound.segment(row_idx, 4*(N-1)).setZero();
//   if (ang_vel_strict) {
//     // slack vars are set to 0 if constraint is to be strictly enforced
//     lower_bound.segment(row_idx, 4*(N-1)).setZero();
//   }
// }

// void TOP::UpdateTrustRegionCons() {
//   size_t row_idx = 2*state_dim + 20*(N-1) + 4*state_bd_dim*(N-1) + 2*11*(N-1) +
//     state_dim*(N-1);

//   for (size_t ii = 0; ii < N-1; ii++) {
//     // -sk-Xk <= -Xkp
//     upper_bound.segment(row_idx, state_dim) = -Xprev[ii+1];
//     row_idx += (state_dim);

//     // Xk-sk <= Xkp
//     upper_bound.segment(row_idx, state_dim) = Xprev[ii+1];
//     row_idx += (state_dim);

//     // \sum(sk)-zk \leq Delta
//     upper_bound(row_idx) = Delta;
//     row_idx++;
//   }
// }

// void TOP::UpdateGradient() {
//   // TODO(acauligi)
// }

// void TOP::UpdateObsCons() {
//   size_t row_idx = 2*state_dim + 20*(N-1) + 4*7*(N-1) + 2*11*(N-1) +
//     state_dim*(N-1) + (3*state_dim+2)*(N-1);
//   size_t n_obs = keep_out_zones_->size();

//   Vec3 support_vec;

//   for (size_t ii = 0; ii < N-1; ii++) {
//     for (size_t jj = 0; jj < n_obs; jj++) {
//       support_vec = support_vectors[n_obs*ii+jj];

//       upper_bound(row_idx) = obs_ub[n_obs*ii+jj];
//       for (size_t kk = 0; kk < 3; kk++) {
//         linear_con_mat.coeffRef(row_idx, state_dim*(ii+1)+kk) = support_vec(kk);
//       }

//       row_idx++;
//     }
//   }
// }

// decimal_t TOP::ConvergenceMetric() {
//   decimal_t max_num = -OsqpEigen::INFTY;
//   decimal_t max_den = -OsqpEigen::INFTY;

//   for (size_t ii = 0; ii < N; ii++) {
//     decimal_t val = (qp_soln.block(state_dim*ii, 0, state_dim, 1) - Xprev[ii]).norm();
//     max_num = (val > max_num) ? val : max_num;

//     val = Xprev[ii].norm();
//     max_den = (val > max_den) ? val : max_den;
//   }
//   return max_num*100.0/max_den;
// }

// decimal_t TOP::AccuracyRatio() {
//   decimal_t num = 0;
//   decimal_t den = 0;

//   Vec13 X_k;
//   Vec6 U_k;
//   Vec7 f_k;
//   Vec7 linearized;

//   // dynamics
//   for (size_t ii = 0; ii < N-1; ii++) {
//     X_k = qp_soln.segment(state_dim*ii, state_dim);
//     decimal_t q_norm = X_k.segment(6, 4).norm();
//     X_k.segment(6, 4) /= q_norm;

//     U_k = qp_soln.segment(state_dim*N+control_dim*ii, control_dim);

//     UpdateF(f_k, X_k, U_k);

//     // TODO(acauligi): determine whether A_kp,B_kp,f_kp need to be recomputed

//   Vec7 ck;
//   Vec7 X_k, Xprev_k;
//   Vec7 f_k;
//   Vec3 U_k, Uprev_k;

//     linearized = fs[ii] +
//       As[ii]*(X_k.segment(7, 6)-Xprev[ii].segment(7, 6)) +
//       Bs[ii]*(U_k.segment(3, 3)-Uprev[ii].segment(3, 3));
//     num += (f_k - linearized).norm();
//     den += linearized.norm();
//   }

//   // TODO(acauligi)
//   // obstacles

//   return num*100.0/den;
// }

// bool TOP::TrustRegionSatisfied() {
//   Vec13 diff;
//   for (size_t ii = 0; ii < N; ii++) {
//     diff = qp_soln.segment(state_dim*ii, state_dim)-Xprev[ii];
//     // TODO(acauligi): use lp-norm here
//     if (diff.norm() > Delta) {
//       return false;
//     }
//   }
//   return true;
// }

// bool TOP::SatisfiesStateInequalityConstraints() {
//   // State box constraints
//   for (size_t ii = 0; ii < N; ii++) {
//     for (size_t jj = 0; jj < state_dim; jj++) {
//       if (qp_soln(state_dim*ii+jj) > x_max(jj)) {
//         return false;
//       } else if (qp_soln(state_dim*ii+jj) < x_min(jj)) {
//         return false;
//       }
//     }
//   }

//   // Linear and angular velocity norm constraints
//   for (size_t ii = 0; ii < N; ii++) {
//     if (qp_soln.segment(state_dim*ii, 3).lpNorm<1>() > desired_vel_) {
//       return false;
//     } else if (qp_soln.segment(state_dim*ii+10, 3).lpNorm<1>() > desired_omega_) {
//       return false;
//     }
//   }

//   // Trust region constraints: already checked in TrustRegionSatisfied()

//   // Obstacle avoidance constraints
//   size_t n_obs = keep_out_zones_->size();
//   for (size_t ii = 0; ii < N; ii++) {
//     for (size_t jj = 0 ; jj < n_obs; jj++) {
//       if (support_vectors[n_obs*ii+jj].dot(qp_soln.segment(state_dim*ii, 3)) >= obs_ub[n_obs*ii+jj]) {
//         return false;
//       }
//     }
//   }

//   return true;
// }

*/

void TOP::WriteTrajectoryToFile(const Vec13Vec& states, const Vec6Vec& controls, const std::string& filename) {
  CreateDirectoryIfNotExists(output_dir);
  std::ofstream traj_file(filename);
  if (!traj_file.is_open()) {
    std::cerr << "Failed to open trajectory file " << filename << "for writing." << std::endl;
    return;
  } else {
    std::cout << "[TOP::WriteTrajectoryToFile] Writing trajectory to: " << filename << std::endl;
  }
  char full_path[PATH_MAX];
  if (realpath(filename.c_str(), full_path)) {
    std::cout << "Full path: " << full_path << std::endl;
  } else {
    std::cerr << "Error resolving path: " << filename << " " << strerror(errno) << std::endl;
    throw std::runtime_error("Error resolving path: " + filename + " " + std::string(strerror(errno)));
  }
  for (size_t i = 0; i < states.size(); ++i) {
    traj_file << states[i].transpose();
    if (i < controls.size()) traj_file << " " << controls[i].transpose();
    traj_file << std::endl;
  }
  traj_file.close();
}

void TOP::WriteTrajectoryToFileForNN(const Vec13& x0, const Vec13& xg, int N, const Vec13Vec& Xsoln,
                                     const Vec6Vec& Usoln,
                                     const std::string& fname) {
  CreateDirectoryIfNotExists(output_dir);
  std::ofstream file(fname);
  if (!file.is_open()) {
    std::cerr << "Error: Unable to open file " << fname << " for writing." << std::endl;
    throw std::runtime_error("Error: Unable to open file " + fname + " for writing.");
    return;
  } else {
    std::cout << "Writing trajectory to: " << fname << std::endl;
  }
  char full_path[PATH_MAX];
  if (realpath(fname.c_str(), full_path)) {
    std::cout << "Full path: " << full_path << std::endl;
  } else {
    std::cerr << "Error resolving path: " << strerror(errno) << std::endl;
    throw std::runtime_error("Error resolving path: " + std::string(strerror(errno)));
  }

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
      file << Xsoln[i][j] << " ";
    }
    file << std::endl;
  }

  // Write the Uprev data (N-1 lines of length 6)
  for (int i = 0; i < N - 1; ++i) {
    for (int j = 0; j < 6; ++j) {
      file << Usoln[i][j] << " ";
    }
    file << std::endl;
  }

  // Close the file stream
  file.close();

  std::cout << "Trajectory data for training written to '" << fname << "'" << std::endl;
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

void TOP::TrainModel(const std::vector<std::string>& files, int epochs) {
  std::vector<torch::Tensor> inputs, outputs;

  // Read all data files
  for (const std::string& file : files) {
    std::tuple<torch::Tensor, torch::Tensor> data = ReadData(file);
    inputs.push_back(std::get<0>(data));
    outputs.push_back(std::get<1>(data));
  }

  // Concatenate tensors for batch training
  torch::Tensor input_tensor = torch::cat(inputs, 0);
  torch::Tensor output_tensor = torch::cat(outputs, 0);

  // Training loop
  for (int epoch = 0; epoch < epochs; ++epoch) {
    net->train();
    optimizer.zero_grad();

    torch::Tensor predictions = net->forward(input_tensor);
    torch::Tensor loss = torch::mse_loss(predictions, output_tensor);

    loss.backward();
    optimizer.step();

    std::cout << "Epoch [" << epoch + 1 << "/" << epochs << "], Loss: " << loss.item<float>() << std::endl;
  }
}

void TOP::SaveModel(const std::string& model_path) {
  torch::save(net, model_path);
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
  torch::load(net, model_path);
  std::cout << "[TOP::LoadModel] Model successfully loaded from " << model_path << std::endl;
}

std::string TOP::getCurrentTimestamp() {
  // Get the current time as a time_point
  auto now = std::chrono::system_clock::now();

  // Convert it to a time_t to work with std::strftime
  std::time_t now_time = std::chrono::system_clock::to_time_t(now);

  // Convert to a string with a specific format (e.g., YYYY-MM-DD_HH-MM-SS)
  std::stringstream ss;
  ss << std::put_time(std::localtime(&now_time), "%Y-%m-%d_%H-%M-%S");

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
std::tuple<scp::Vec13Vec, scp::Vec13Vec> initializeMotionCases(bool is_granite, bool saveForNNTraining = false) {
  scp::Vec13Vec x0s;
  scp::Vec13Vec xgs;

  scp::Vec13 x0;
  scp::Vec13 xg;

  bool single_x0 = false;  // true --> single x0, multiple xg. false --> multiple x0, multiple xg.

  if (saveForNNTraining) {
    if (is_granite) {
      throw std::runtime_error("Granite case not supported for NN training.");
      return std::make_tuple(x0s, xgs);
    } else {
      if (single_x0) {
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
      } else {
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
void processProblemInstance(scp::TOP& top_eg, const scp::Vec13& x0, const scp::Vec13& xg,
                            const Eigen::AlignedBox3d& vbox, int problemIndex, bool saveForNNTraining = false) {
  top_eg.x0 = x0;
  top_eg.xg = xg;
  clearToZeros(top_eg.Xprev);
  clearToZeros(top_eg.Uprev);

  if (top_eg.is_granite) {
    if (vbox.isEmpty()) {
        top_eg.keep_out_zones_.clear();
    } else {
        top_eg.keep_out_zones_.clear();
        top_eg.keep_out_zones_.push_back(vbox);
    }
  } else {
    std::cout << "Number of obstacles: " << top_eg.keep_out_zones_.size() << std::endl;
    top_eg.keep_out_zones_.clear();
    top_eg.keep_out_zones_.push_back(vbox);
    std::cout << "After adding 1, Number of obstacles: " << top_eg.keep_out_zones_.size() << std::endl;
  }
  std::string timestamp = top_eg.getCurrentTimestamp();
  std::string fname = top_eg.output_dir + std::string((saveForNNTraining) ? "/for_NN_training/" : "/") +
                      std::string((top_eg.is_granite) ? "granite" : "iss") + "_optim_trajectory_" +
                      std::to_string(problemIndex) + std::string((saveForNNTraining) ? "" : ("_" + timestamp)) + ".txt";

  if (!top_eg.Solve()) {
    if (saveForNNTraining) {
      std::cout << "Failure: Problem " << problemIndex << " could not be solved!" << std::endl;
      return;
    } else {
      scp::Vec13Vec empty_Xprev;
      scp::Vec6Vec empty_Uprev;
      top_eg.WriteTrajectoryToFile(empty_Xprev, empty_Uprev, fname);
      std::cout << "Failure: Problem " << problemIndex << " could not be solved!" << std::endl;
      std::cout << "Empty trajectory written to file: " << fname << std::endl;
      std::cout << "--------------------------------------------" << std::endl;
      return;
    }
  }

  std::cout << "Success: Problem " << problemIndex << " solved!" << std::endl;

  Eigen::VectorXd solution = top_eg.solver->getSolution();
  for (size_t ii = 0; ii < top_eg.N; ii++) {
      top_eg.Xprev[ii] = solution.segment(top_eg.state_dim * ii, top_eg.state_dim);
  }
  for (size_t ii = 0; ii < top_eg.N - 1; ii++) {
      top_eg.Uprev[ii] = solution.segment(top_eg.state_dim * top_eg.N + top_eg.control_dim * ii, top_eg.control_dim);
  }

  if (saveForNNTraining) {
    top_eg.WriteTrajectoryToFileForNN(top_eg.x0, top_eg.xg, top_eg.N, top_eg.Xprev, top_eg.Uprev, fname);
  } else {
    top_eg.WriteTrajectoryToFile(top_eg.Xprev, top_eg.Uprev, fname);
  }

  std::cout << "Trajectory written to file: " << fname << std::endl;
  std::cout << "--------------------------------------------" << std::endl;
  return;
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
  bool load_and_run_inference = false;
  bool test_warm_start = false;

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
    scp::TOP top_eg(20., 801);
    // Set ISS environment
    top_eg.is_granite = false;
    top_eg.enforce_obs_avoidance_const = false;

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
      smallObstacle.extend(Eigen::Vector3d(10.0, -9.2, 100.0));
      smallObstacle.extend(Eigen::Vector3d(10.6, -9.0, 200.0));
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
    bool saveForNNTraining = true;
    scp::TOP top(20., 801);
    top.is_granite = false;
    top.enforce_obs_avoidance_const = false;

    // Initialize motion cases
    scp::Vec13Vec x0s, xgs;
    std::tie(x0s, xgs) = initializeMotionCases(top.is_granite, saveForNNTraining);

    // Process problems
    for (size_t i = 0; i < xgs.size(); ++i) {
      num_problems++;
      processProblemInstance(top, x0s[i], xgs[i], Eigen::AlignedBox3d(), num_problems, saveForNNTraining);
    }
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

  scp::TOP* top;
  top = new scp::TOP(20., 801);
  top->nn_model_path = "/home/enceladus/astrobee/src/saved_NN_models/trained_model_27_2025-01-03_00-34-39.pt";
  top->use_nn_warm_start = true;
  top->Solve();

  return 0;
}
