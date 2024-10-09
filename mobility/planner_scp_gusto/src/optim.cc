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

#include <algorithm>
#include <iostream>
#include <vector>
#include <cmath>
#include <string>

namespace scp {

TOP::TOP(decimal_t Tf_, int N_)
  : N(N_), Tf(Tf_) {
  state_dim = 13;
  state_dim_lin = 6;
  state_dim_nlin = 7;
  state_bd_dim = 7;
  lin_vel_dim = 3;
  ang_vel_dim = 3;
  control_dim = 6;
  control_dim_lin = 3;
  control_dim_nlin = 3;
  dh = Tf / N;

  // TODO(somrita): Implement all of these
  enforce_init_cond = true;
  enforce_final_cond = true;
  enforce_lin_dynamics = true;
  enforce_rot_dynamics = true;
  enforce_force_norm = true;
  enforce_moment_norm = false;
  enforce_state_LB = false;
  enforce_state_UB = false;
  enforce_lin_vel_norm = false;
  enforce_ang_vel_norm = false;
  enforce_trust_region_const = false;
  enforce_obs_avoidance_const = false;

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

  // TODO(acauligi): determine what pos_min_ and pos_max_ should be correctly
  for (size_t ii = 0; ii < 3; ii++) {
    pos_min_(ii) = -OsqpEigen::INFTY;
    pos_max_(ii) = OsqpEigen::INFTY;
    // TODO(somrita): Remove after testing
    // pos_min_(ii) = -6;
    // pos_max_(ii) = 6;
  }

  x_max << pos_max_(0), pos_max_(1), pos_max_(2),
    desired_vel_, desired_vel_, desired_vel_,
    1, 1, 1, 1,
    desired_omega_, desired_omega_, desired_omega_;
  // TODO(somrita): Remove after testing
  // x_max << pos_max_(0), pos_max_(1), pos_max_(2),
  //   desired_vel_, desired_vel_, desired_vel_,
  //   0.5, 0.5, 0.5, 0.5,
  //   desired_omega_, desired_omega_, desired_omega_;
  x_min = -x_max;
  x_min(0) = pos_min_(0);
  x_min(1) = pos_min_(1);
  x_min(2) = pos_min_(2);

  // TODO(somrita): remove
  std::cout << "Min position" << x_min(0) << x_min(1) << x_min(2) << std::endl;
  std::cout << "Min quaternion" << x_min(6) << x_min(7) << x_min(8) << x_min(9) <<std::endl;

  ResetSCPParams();
  UpdateProblemDimension(N);

  // Run warm start
  if (!solver->solve()) {
    solver_ready_ = false;
  }

  if (solver_ready_) {
    std::cout << "Solver ready to solve!" << std::endl;
  } else {
    std::cout << "Solver failed to instantiate!" << std::endl;
  }
}

// TOP::~TOP() {
//   delete;
// }

size_t TOP::GetNumTOPVariables() {
  // TODO(somrita): Continue updating these
  size_t num_force_norm_slack_vars = penalize_total_force ? (control_dim_lin+1)*(N-1) : control_dim_lin*(N-1);
  size_t num_moment_norm_slack_vars = penalize_total_moment ? (control_dim_nlin+1)*(N-1) : control_dim_nlin*(N-1);
  return state_dim * N                  // State variables
         + control_dim * (N - 1)        // Control variables
         + num_force_norm_slack_vars    // Force norm slack variables
         + num_moment_norm_slack_vars   // Moment norm slack variables
         + state_bd_dim * (N - 1)       // State LB slack variables
         + state_bd_dim * (N - 1)       // State UB slack variables
         + (lin_vel_dim + 1) * (N - 1)  // Linear velocity norm slack variables
         + (ang_vel_dim + 1) * (N - 1)  // Angular velocity norm slack variables
         + 2 * (N - 1);                 // Obstacle avoidance slack variables
}

size_t TOP::GetNumTOPConstraints() {
  // TODO(somrita): Continue updating these
  size_t num_force_norm_cons = penalize_total_force ? 11*(N-1) : 10*(N-1);
  size_t num_moment_norm_cons = penalize_total_moment ? 11*(N-1) : 10*(N-1);
  return state_dim*(N-1)  // Dynamics
  + 2*state_dim  // Initial and final boundary conditions
  + num_force_norm_cons  // Force norm constraints
  + num_moment_norm_cons  // Moment norm constraints
  + 2*state_bd_dim*(N-1)  // State LB constraints
  + 2*state_bd_dim*(N-1)  // State UB constraints
  + 11*(N-1)  // Linear velocity norm constraints
  + 11*(N-1)  // Angular velocity norm constraints
  + 4*(N-1);  // Obstacle avoidance constraints
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
  if (solver && N == N_) {
    return;
  }

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

  std::cout << "cleared stuff" << std::endl;

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

  hessian.resize(num_vars, num_vars);
  linear_con_mat.resize(num_cons, num_vars);
  gradient.resize(num_vars);
  lower_bound.resize(num_cons);
  upper_bound.resize(num_cons);
  qp_soln.resize(num_vars);

  UpdateDoubleIntegrator();
  InitTrajStraightline();
  UpdateRotationalDynamics();

  for (size_t ii = 0; ii < num_cons; ii++) {
    lower_bound(ii) = -OsqpEigen::INFTY;
    // TODO(somrita): Remove
    // lower_bound(ii) = 0.0;
    upper_bound(ii) = OsqpEigen::INFTY;
  }

  SetSimpleConstraints();
  SetSimpleCosts();

  // Set up solver
  abs_tol_ = 1e-3;  // default 1e-03
  rel_tol_ = 1e-3;  // default 1e-03
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

  std::cout << "q0: " << q0.x() << " " << q0.y() << " " << q0.z() << " " << q0.w() << std::endl;
  std::cout << "qg: " << qg.x() << " " << qg.y() << " " << qg.z() << " " << qg.w() << std::endl;

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
}

void TOP::SetSimpleConstraints() {
  std::cout << "Setting simple constraints" << std::endl;
  std::cout << "enforce_init_cond: " << enforce_init_cond << std::endl;
  std::cout << "enforce_final_cond: " << enforce_final_cond << std::endl;
  std::cout << "enforce_lin_dynamics: " << enforce_lin_dynamics << std::endl;
  std::cout << "enforce_rot_dynamics: " << enforce_rot_dynamics << std::endl;
  std::cout << "enforce_obs_avoidance_const: " << enforce_obs_avoidance_const << std::endl;

  Mat7 eye;
  eye.setIdentity();

  size_t row_idx = 0;

  for (size_t ii = 0; ii < N-1; ii++) {
    if (enforce_lin_dynamics) {
      // Double integrator dynamics first
      for (size_t jj = 0; jj < state_dim_lin; jj++) {
        lower_bound(row_idx) = 0.0;
        upper_bound(row_idx) = 0.0;

        linear_con_mat.coeffRef(row_idx, state_dim*(ii+1)+jj) = -1.0;
        for (size_t kk = 0; kk < state_dim_lin; kk++) {
          linear_con_mat.coeffRef(row_idx, state_dim*ii+kk) = Ak_di(jj, kk);
        }

        for (size_t kk = 0; kk < control_dim_lin; kk++) {
          linear_con_mat.coeffRef(row_idx, state_dim*N+control_dim*ii+kk) = Bk_di(jj, kk);
        }
        row_idx++;
      }
    }
    if (ii == 0) {
      std::cout << "Linear constraints matrix Ak_di part: \n"
                << linear_con_mat.block(0, 0, state_dim_lin, state_dim_lin) << std::endl;
      std::cout << "Linear constraints matrix Bk_di part: \n"
                << linear_con_mat.block(0, state_dim * N, state_dim_lin, control_dim_lin) << std::endl;
    }

    if (enforce_rot_dynamics) {
      // Nonlinear attitude dynamics
      // q[ii+1] = q[ii] + dh*As.block(0,0,4,4)*q[ii]
      // omega[ii+1] = omega[ii] + dh*Bs.block(4,4,3,3)*u[ii] + dh*fs.segment(4,3)
      // x[ii+1] = x[ii] + dh*As*x[ii] + dh*Bs*u[ii] + dh*fs
      // -dh*fs = -x[ii+1] + (eye + dh*As)*x[ii] + dh*Bs*u[ii]
      // As[ii] is a 7x7 matrix
      // Bs[ii] is a 7x3 matrix
      // fs[ii] is a 7x1 vector
      // As.block(0,0,4,4) is a 4x4 matrix (rest is zeros)
      // Bs.block(4,0,3,3) is a 3x3 matrix  (rest is zeros)
      // fs.segment(4,3) is a 3x1 vector (rest is zeros)

      for (size_t jj = 0; jj < state_dim_nlin; jj++) {
        lower_bound(row_idx) = -dh*fs[ii](jj);
        upper_bound(row_idx) = -dh*fs[ii](jj);

        // Simple explicit Euler integration scheme
        linear_con_mat.coeffRef(row_idx, state_dim*(ii+1)+state_dim_lin+jj) = -1.0;
        for (size_t kk = 0; kk < state_dim_nlin; kk++) {
          linear_con_mat.coeffRef(row_idx, state_dim*ii+state_dim_lin+kk) =
            (eye(jj, kk)+dh*As[ii](jj, kk) );
        }
        for (size_t kk = 0; kk < control_dim_nlin; kk++) {
          linear_con_mat.coeffRef(row_idx,
            state_dim*N+control_dim*ii+control_dim_lin+kk) = dh*Bs[ii](jj, kk);
        }
        row_idx++;
      }
    }
    if (ii == 0) {
      std::cout << "Linear constraints matrix As part: \n"
                << As[ii] << std::endl;
      std::cout << "Linear constraints matrix Bs part: \n"
                << Bs[ii] << std::endl;
      std::cout << "Linear constraints matrix fs part: \n"
                << fs[ii] << std::endl;
    }
  }

  // Initial state
  if (enforce_init_cond) {
    for (size_t ii = 0; ii < state_dim; ii++) {
      linear_con_mat.coeffRef(row_idx+ii, ii) = 1.0;
      lower_bound(row_idx+ii) = x0(ii);
      upper_bound(row_idx+ii) = x0(ii);
    }
    row_idx += state_dim;
  }

  // Goal state
  if (enforce_final_cond) {
    for (size_t ii = 0; ii < state_dim; ii++) {
      linear_con_mat.coeffRef(row_idx+ii, state_dim*(N-1)+ii) = 1.0;
      lower_bound(row_idx+ii) = xg(ii);
      upper_bound(row_idx+ii) = xg(ii);
      // lower_bound(row_idx+ii) = xg(ii)-1;
      // upper_bound(row_idx+ii) = xg(ii)+1;
    }
    row_idx += state_dim;
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
      return;
    }
    Eigen::AlignedBox3d box = keep_out_zones_[0];
    Eigen::Vector3d ko_min = box.min();
    Eigen::Vector3d ko_max = box.max();
    std::cout << "ko_min: " << ko_min.transpose() << std::endl;
    std::cout << "ko_max: " << ko_max.transpose() << std::endl;
    decimal_t ko_x_min = ko_min(0);
    decimal_t ko_x_max = ko_max(0);
    decimal_t ko_y_min = ko_min(1);
    decimal_t ko_y_max = ko_max(1);
    decimal_t ko_center_x = (ko_x_max + ko_x_min)/2;
    decimal_t ko_center_y = (ko_y_max + ko_y_min)/2;
    std::cout << "ko_center_x: " << ko_center_x << std::endl;
    std::cout << "ko_center_y: " << ko_center_y << std::endl;
    for (size_t ii = 0; ii < N-1; ii++) {
      for (size_t kk = 0; kk < 2; kk++) {  // x and y
        size_t d_slack_var_idx = state_dim*N+control_dim*(N-1)+
        num_force_norm_slack_vars+num_moment_norm_slack_vars+
        state_bd_dim*(N-1)+state_bd_dim*(N-1)+
        (lin_vel_dim+1)*(N-1)+(ang_vel_dim+1)*(N-1)+2*ii+kk;
        // -d_ik <= 0
        linear_con_mat.coeffRef(row_idx, d_slack_var_idx) = -1.0;
        upper_bound(row_idx) = 0;
        row_idx++;
        // Using prev, determine active constraints
        decimal_t prev = Xprev[ii](kk);
        if (prev > (ko_min(kk) + ko_max(kk)) / 2) {  // If x > ko_center_x, then -x + d_i <= -ko_x_max
          linear_con_mat.coeffRef(row_idx, state_dim*(ii+1)+kk) = -1.0;
          linear_con_mat.coeffRef(row_idx, d_slack_var_idx) = 1.0;
          upper_bound(row_idx) = -ko_max(kk);
          row_idx++;
        } else {  // If x < ko_center_x, then x + d_i <= ko_x_min
          linear_con_mat.coeffRef(row_idx, state_dim*(ii+1)+kk) = 1.0;
          linear_con_mat.coeffRef(row_idx, d_slack_var_idx) = 1.0;
          upper_bound(row_idx) = ko_min(kk);
          row_idx++;
        }
      }
    }
  }
}

void TOP::SetSimpleCosts() {
  // Cost function
  for (size_t ii = 0; ii < N; ii++) {
    // Penalize only distance to xg and final linear velocity
    for (size_t jj = 0; jj < state_dim_lin; jj++) {
      // hessian.insert(state_dim*ii+jj, state_dim*ii+jj) = 10.0;
      // gradient(state_dim*ii+jj) = -2*xg(jj);
    }
  }

  for (size_t ii = 0; ii < N-1; ii++) {
    for (size_t jj = 0; jj < control_dim; jj++) {
      // hessian.insert(state_dim*N+control_dim*ii+jj, state_dim*N+control_dim*ii+jj) = 10.0;
    }
  }

  if (enforce_state_LB) {
    // Add penalty for slack variables associated with LB
    size_t slack_var_idx_start = state_dim*N + control_dim*(N-1) + num_force_norm_slack_vars+num_moment_norm_slack_vars;
    // Penalty for state lower bounds
    for (size_t ii = 0; ii < state_bd_dim*(N-1); ii++) {
      size_t slack_var_idx = slack_var_idx_start + ii;
      gradient(slack_var_idx) = omega;
      slack_var_idx++;
    }
  }

  if (enforce_state_UB) {
    // Add penalty for slack variables associated with UB
    size_t slack_var_idx_start = state_dim*N+control_dim*(N-1)+num_force_norm_slack_vars+num_moment_norm_slack_vars+
      state_bd_dim*(N-1);
    // Penalty for state upper bounds
    for (size_t ii = 0; ii < state_bd_dim*(N-1); ii++) {
      size_t slack_var_idx = slack_var_idx_start + ii;
      gradient(slack_var_idx) = omega;
      slack_var_idx++;
    }
  }

  if (enforce_lin_vel_norm) {
    // Add penalty for z slack var associated with lin vel norm
    for (size_t ii = 0; ii < N-1; ii++) {
      size_t z_slack_var_idx = state_dim*N+control_dim*(N-1)+num_force_norm_slack_vars+num_moment_norm_slack_vars+
        state_bd_dim*(N-1)+state_bd_dim*(N-1) + 4*ii + 3;
      gradient(z_slack_var_idx) = omega;
    }
  }

  if (enforce_ang_vel_norm) {
    // Add penalty for z slack var associated with ang vel norm
    for (size_t ii = 0; ii < N-1; ii++) {
      size_t z_slack_var_idx = state_dim*N+control_dim*(N-1)+num_force_norm_slack_vars+num_moment_norm_slack_vars+
        state_bd_dim*(N-1)+state_bd_dim*(N-1)+4*(N-1)+ 4*ii + 3;
      gradient(z_slack_var_idx) = omega;
    }
  }

  if (penalize_total_force) {
    for (size_t ii = 0; ii < N-1; ii++) {
      size_t z_slack_var_idx = state_dim*N+control_dim*(N-1)+ii*num_force_norm_slack_vars_per_iter+4;
      gradient(z_slack_var_idx) = omega;
    }
  }

  if (penalize_total_moment) {
    for (size_t ii = 0; ii < N-1; ii++) {
      size_t z_slack_var_idx = state_dim*N+control_dim*(N-1)+num_force_norm_slack_vars+
        ii*num_moment_norm_slack_vars_per_iter+4;
      gradient(z_slack_var_idx) = omega;
    }
  }
}

void TOP::UpdateSimpleConstraints() {
  Mat7 eye;
  eye.setIdentity();

  size_t row_idx = 0;
  size_t control_dim_lin = control_dim/2;
  size_t control_dim_nlin = control_dim/2;

  for (size_t ii = 0; ii < N-1; ii++) {
    if (enforce_lin_dynamics) {
      // Double integrator dynamics first
      for (size_t jj = 0; jj < state_dim_lin; jj++) {
        lower_bound(row_idx) = 0.0;
        upper_bound(row_idx) = 0.0;

        linear_con_mat.coeffRef(row_idx, state_dim*(ii+1)+jj) = -1.0;
        for (size_t kk = 0; kk < state_dim_lin; kk++) {
          linear_con_mat.coeffRef(row_idx, state_dim*ii+kk) = Ak_di(jj, kk);
        }

        for (size_t kk = 0; kk < control_dim_lin; kk++) {
          linear_con_mat.coeffRef(row_idx, state_dim*N+control_dim*ii+kk) = Bk_di(jj, kk);
        }
        row_idx++;
      }
    }

    if (enforce_rot_dynamics) {
      // Nonlinear attitude dynamics second
      // c = dh*(A*Xprev + B*Uprev - fk) = (eye + dh*As)*x[ii] + (dh*Bs)*u[ii] -x[ii+1]
      // As[ii] is a 7x7 matrix
      // Bs[ii] is a 7x3 matrix
      // fs[ii] is a 7x1 vector
      Vec7 Xprev_k = Xprev[ii].segment(6, 7);
      Vec3 Uprev_k = Uprev[ii].segment(3, 3);
      Vec7 fk = fs[ii];
      Vec7 ck = dh*(As[ii]*Xprev_k + Bs[ii]*Uprev_k - fk);
      for (size_t jj = 0; jj < state_dim_nlin; jj++) {
        lower_bound(row_idx) = ck(jj);
        upper_bound(row_idx) = ck(jj);

        // Simple explicit Euler integration scheme
        linear_con_mat.coeffRef(row_idx, state_dim*(ii+1)+state_dim_lin+jj) = -1.0;
        for (size_t kk = 0; kk < state_dim_nlin; kk++) {
          linear_con_mat.coeffRef(row_idx, state_dim*ii+state_dim_lin+kk) =
            (eye(jj, kk)+dh*As[ii](jj, kk) );
        }
        for (size_t kk = 0; kk < control_dim_nlin; kk++) {
          linear_con_mat.coeffRef(row_idx,
            state_dim*N+control_dim*ii+control_dim_lin+kk) = dh*Bs[ii](jj, kk);
        }
        row_idx++;
      }
    }
  }

  // TODO(somrita): Remove this block
  // // Skip over force and moment constraints
  // row_idx += 2*10*(N-1);

  // // State LB
}

void TOP::UpdateSimpleCosts() {
  // Cost function
  for (size_t ii = 0; ii < N; ii++) {
    // Penalize only distance to xg and final linear velocity
    for (size_t jj = 0; jj < state_dim_lin; jj++) {
      // hessian.coeffRef(state_dim*ii+jj, state_dim*ii+jj) = 10.0;
      // gradient(state_dim*ii+jj) = -2*xg(jj);
    }
  }

  for (size_t ii = 0; ii < N-1; ii++) {
    for (size_t jj = 0; jj < control_dim; jj++) {
      // hessian.coeffRef(state_dim*N+control_dim*ii+jj, state_dim*N+control_dim*ii+jj) = 10.0;
    }
  }

  // TODO(somrita): Put this block back
  // // Add penalty for slack variables associated with max(g(x),0)
  // size_t row_idx = state_dim*N + control_dim*(N-1) + control_dim*(N-1);

  // // Penalty for state upper and lower bounds
  // // TODO(somrita): Make this 2*state_bd_dim*(N-1) when UB is added
  // for (size_t ii = 0; ii < state_bd_dim*(N-1); ii++) {
  //   gradient(row_idx) = omega;
  //   row_idx++;
  // }
}


void TOP::UpdateDoubleIntegrator() {
  Ak_di.setZero();
  Bk_di.setZero();

  for (size_t ii = 0; ii < 3; ii++) {
    Ak_di(ii, ii) = 1.;
    Ak_di(ii, 3+ii) = dh;
    Ak_di(ii+3, ii+3) = 1;

    // Bk_di(ii, ii) = 0.5 * pow(dh, 2) / mass; // Not part of a double integrator
    Bk_di(3+ii, ii) = dh / mass;
  }
}

void TOP::UpdateF(Vec7& f, Vec13& X, Vec6& U) {
  f.setZero();

  decimal_t Jxx = J(0, 0);
  decimal_t Jyy = J(1, 1);
  decimal_t Jzz = J(2, 2);

  decimal_t wx = X(10);
  decimal_t wy = X(11);
  decimal_t wz = X(12);

  Vec3 negJinvOmegaJomega;
  negJinvOmegaJomega << (Jyy-Jzz)*wz*wy/Jxx,
                        (Jzz-Jxx)*wx*wz/Jyy,
                        (Jxx-Jyy)*wy*wx/Jzz;

  f.segment(4, 3) = negJinvOmegaJomega;
}

void TOP::UpdateA(Mat7& A, Vec13& X, Vec6& U) {
  A.setZero();

  decimal_t wx = X(10);
  decimal_t wy = X(11);
  decimal_t wz = X(12);

  Mat4 df_dq;
  df_dq << 0, -wz, wy, wx,
                wz, 0, -wx, wy,
                -wy, wx, 0, wz,
                -wx, -wy, -wz, 0;
  A.block(0, 0, 4, 4) = 0.5*df_dq;
}

void TOP::UpdateB(Mat7x3& B, Vec13& X, Vec6& U) {
  B.setZero();

  B.block(4, 0, 3, 3) = Jinv;
}

void TOP::UpdateRotationalDynamics() {
  // re-normalize quaternions between iterations
  NormalizeQuaternions();

  for (size_t ii = 0; ii < N-1; ii++) {
    UpdateF(fs[ii], Xprev[ii], Uprev[ii]);
    UpdateA(As[ii], Xprev[ii], Uprev[ii]);
    UpdateB(Bs[ii], Xprev[ii], Uprev[ii]);
  }
}

bool TOP::Solve() {
  solved_ = false;
  InitTrajStraightline();
  bool add_custom_keep_out_zone = true;

  std::cout << "SCP::InitTrajStraightline complete" << std::endl;
  std::cout << "SCP:: start of init traj is " << Xprev[0].transpose() << std::endl;
  std::cout << "SCP:: end of init traj is " << Xprev[N-1].transpose() << std::endl;
  // ROS_ERROR_STREAM("SCP::InitTrajStraightline done");

  std::cout << "TOP:: mass: " << mass << std::endl;
  std::cout << "TOP:: inertia: " << J << std::endl;

  std::cout << "TOP:: desired accel: " << desired_accel_ << std::endl;

  std::cout << "TOP:: Keep in zones: " << std::endl;
  for (size_t i = 0; i < keep_in_zones_.size(); ++i) {
    std::cout << "Zone " << i << std::endl;
    std::cout << "min: " << keep_in_zones_[i].min().transpose() << std::endl;
    std::cout << "max: " << keep_in_zones_[i].max().transpose() << std::endl;
  }
  std::cout << "TOP:: Keep out zones: " << std::endl;
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

  // Set warm start
  for (size_t ii = 0; ii < N; ii++) {
    qp_soln.segment(state_dim*ii, state_dim) = Xprev[ii];
  }
  for (size_t ii = 0; ii < N-1; ii++) {
    qp_soln.segment(state_dim*N+control_dim*ii, control_dim) = Uprev[ii];
  }
  if (!solver->setPrimalVariable(qp_soln)) {
    solver_ready_ = false;
  }

  std::cout << "SCP::Warm start complete" << std::endl;

  // TODO(somrita): Reset max_iter
  max_iter = 3;
  for (size_t kk = 0; kk < max_iter; kk++) {
    // Update dynamics matrices
    UpdateDoubleIntegrator();
    UpdateRotationalDynamics();

    if (kk == 0) {
      // Set constraints and costs on first iteration
      SetSimpleConstraints();
      SetSimpleCosts();
    } else {
      // Update constraints and costs
      UpdateSimpleConstraints();
      UpdateSimpleCosts();
    }

    // UpdateTrustRegionCons();
    // UpdateGradient();
    // UpdateObsCons();

    if (!solver->updateLinearConstraintsMatrix(linear_con_mat)) {
      solver_ready_ = false;
    } else if (!solver->updateGradient(gradient)) {
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
  UpdateDoubleIntegrator();
  UpdateRotationalDynamics();

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
    lin_dyn_resid[ii] = Ak_di*x + Bk_di*u - xp;
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

  // TODO(somrita): Add checks for trust region and obstacle avoidance constraints
  std::cout << std::endl;
}

void TOP::NormalizeQuaternions() {
  // re-normalize quaternions
  for (size_t ii = 0; ii < N; ii++) {
    decimal_t q_norm = Xprev[ii].segment(6, 4).norm();
    Xprev[ii].segment(6, 4) /= q_norm;
  }
}

void TOP::PolishSolution() {
  if (!solved_) {
    return;
  }
  NormalizeQuaternions();
}

// NOTE: Functions below this point are not currently being used but may be good for future modularization.

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

}  //  namespace scp
