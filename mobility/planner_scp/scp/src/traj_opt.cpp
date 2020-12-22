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

#include "scp/traj_opt.h"

#include <algorithm>
#include <iostream>
#include <vector>
#include <cmath>

namespace scp {

TOP::TOP(decimal_t Tf_, int N_)
  : N(N_), Tf(Tf_) {
  state_dim = 13;
  state_dim_lin = 6;
  state_dim_nlin = 7;
  control_dim = 6;
  state_bd_dim = 7;   // State LB and UB only enforced for pos. and orientation
  dh = Tf / N;

  free_final_state = false;
  state_con_strict = false;
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

  desired_vel_ = 0.2000;
  desired_accel_ = 0.0175;
  desired_omega_ = 0.1745;
  desired_alpha_ = 0.1745;

  // TODO(acauligi): process keep-in+keep-out data
  keep_in_zones_ = new std::vector<Eigen::AlignedBox3d>(0);
  keep_out_zones_ = new std::vector<Eigen::AlignedBox3d>(0);

  // TODO(acauligi): determine what pos_min_ and pos_max_ should be correctly
  for (size_t ii = 0; ii < 3; ii++) {
    pos_min_(ii) = -OsqpEigen::INFTY;
    pos_max_(ii) = OsqpEigen::INFTY;
  }

  x_max << pos_max_(0), pos_max_(1), pos_max_(2),
    desired_vel_, desired_vel_, desired_vel_,
    1, 1, 1, 1,
    desired_omega_, desired_omega_, desired_omega_;
  x_min = -x_max;
  x_min(0) = pos_min_(0);
  x_min(1) = pos_min_(1);
  x_min(2) = pos_min_(2);

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
  // TODO(acauligi): Only state and control (no slack vars yet)
  return state_dim*N + control_dim*(N-1) + control_dim*(N-1);
}

size_t TOP::GetNumTOPConstraints() {
  // TODO(acauligi): only dynamics and boundary conditions
  return state_dim*(N-1) + 2*state_dim + 2*10*(N-1);
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

  Xprev.resize(N);
  Uprev.resize(N-1);

  size_t n_obs = keep_out_zones_->size();
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
  UpdateDynamics();

  linear_con_mat.resize(num_cons, num_vars);
  lower_bound.resize(num_cons);
  upper_bound.resize(num_cons);

  SetSimpleConstraints();
  SetSimpleCosts();

  // Set up solver
  abs_tol_ = 1e-3;
  verbose_ = false;
  warm_start_ = true;
  solver->settings()->setWarmStart(warm_start_);
  solver->settings()->setAbsoluteTolerance(abs_tol_);
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

  for (size_t ii = 0; ii < N; ii++) {
    Xprev[ii] = x0 + (xg-x0)*ii/(N-1.);
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
  Mat7 eye;
  eye.setIdentity();

  size_t row_idx = 0;
  size_t control_dim_lin = control_dim/2;
  size_t control_dim_nlin = control_dim/2;

  for (size_t ii = 0; ii < N-1; ii++) {
    // Double integrator dynamics first
    for (size_t jj = 0; jj < state_dim_lin; jj++) {
      lower_bound(row_idx+jj) = 0.0;
      upper_bound(row_idx+jj) = 0.0;

      linear_con_mat.insert(row_idx+jj, state_dim*(ii+1)+jj) = -1.0;
      for (size_t kk = 0; kk < state_dim_lin; kk++) {
        linear_con_mat.insert(row_idx+jj, state_dim*ii+kk) = Ak_di(jj, kk);
      }

      for (size_t kk = 0; kk < control_dim_lin; kk++) {
        linear_con_mat.insert(row_idx+jj, state_dim*N+control_dim*ii+kk) = Bk_di(jj, kk);
      }
    }

    // Nonlinear attitude dynamics second
    Vec7 Xprev_k = Xprev[ii].segment(6, 7);
    Vec3 Uprev_k = Uprev[ii].segment(3, 3);
    Vec7 fk = fs[ii];
    Vec7 ck = dh*(As[ii]*Xprev_k + Bs[ii]*Uprev_k - fk);
    for (size_t jj = 0; jj < state_dim_nlin; jj++) {
      lower_bound(row_idx+state_dim_lin+jj) = ck(jj);
      upper_bound(row_idx+state_dim_lin+jj) = ck(jj);

      // Simple explicit Euler integration scheme
      linear_con_mat.insert(row_idx+state_dim_lin+jj, state_dim*(ii+1)+state_dim_lin+jj) = -1.0;
      for (size_t kk = 0; kk < state_dim_nlin; kk++) {
        linear_con_mat.insert(row_idx+state_dim_lin+jj, state_dim*ii+state_dim_lin+kk) = eye(jj, kk)+dh*As[ii](jj, kk);
      }
      for (size_t kk = 0; kk < control_dim_nlin; kk++) {
        linear_con_mat.insert(row_idx+state_dim_lin+jj,
          state_dim*N+control_dim*ii+control_dim_lin+kk) = dh*Bs[ii](jj, kk);
      }
    }
    row_idx += state_dim;
  }

  // Initial state
  for (size_t ii = 0; ii < state_dim; ii++) {
    linear_con_mat.insert(row_idx+ii, ii) = 1.0;
    lower_bound(row_idx+ii) = x0(ii);
    upper_bound(row_idx+ii) = x0(ii);
  }
  row_idx += state_dim;

  // Goal state
  for (size_t ii = 0; ii < state_dim; ii++) {
    linear_con_mat.insert(row_idx+ii, state_dim*(N-1)+ii) = 1.0;
    lower_bound(row_idx+ii) = xg(ii);
    upper_bound(row_idx+ii) = xg(ii);
  }
  row_idx += state_dim;

  // Force constraints
  for (size_t ii = 0; ii < N-1; ii++) {
    // -s_ik - a_ik <= 0.0
    for (size_t jj = 0; jj < control_dim_lin; jj++) {
      linear_con_mat.insert(row_idx+jj, state_dim*N+control_dim*(N-1)+control_dim*ii+jj) = -1.0;
      linear_con_mat.insert(row_idx+jj, state_dim*N+control_dim*ii+jj) = -1.0;
      upper_bound(row_idx+jj) = 0.0;
    }
    row_idx += control_dim_lin;

    // a_ik -s_ik <= 0.0
    for (size_t jj = 0; jj < control_dim_lin; jj++) {
      linear_con_mat.insert(row_idx+jj, state_dim*N+control_dim*(N-1)+control_dim*ii+jj) = -1.0;
      linear_con_mat.insert(row_idx+jj, state_dim*N+control_dim*ii+jj) = 1.0;
      upper_bound(row_idx+jj) = 0.0;
    }
    row_idx += control_dim_lin;

    // -s_ik <= 0.0
    for (size_t jj = 0; jj < control_dim_lin; jj++) {
      linear_con_mat.insert(row_idx+jj, state_dim*N+control_dim*(N-1)+control_dim*ii+jj) = -1.0;
      upper_bound(row_idx+jj) = 0.0;
    }
    row_idx += control_dim_lin;

    // sum(s_ik) <= a_max
    for (size_t jj = 0; jj < control_dim_lin; jj++) {
      linear_con_mat.insert(row_idx, state_dim*N+control_dim*(N-1)+control_dim*ii+jj) = 1.0;
    }
    upper_bound(row_idx) = desired_accel_;

    row_idx++;
  }
}

void TOP::SetSimpleCosts() {
  // Cost function
  for (size_t ii = 0; ii < N; ii++) {
    // Penalize only distance to xg and final linear velocity
    for (size_t jj = 0; jj < state_dim_lin; jj++) {
      hessian.insert(state_dim*ii+jj, state_dim*ii+jj) = 10.0;
      gradient(state_dim*ii+jj) = -2*xg(jj);
    }
  }

  for (size_t ii = 0; ii < N-1; ii++) {
    for (size_t jj = 0; jj < control_dim; jj++) {
      hessian.insert(state_dim*N+control_dim*ii+jj, state_dim*N+control_dim*ii+jj) = 10.0;
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
    // Double integrator dynamics first
    for (size_t jj = 0; jj < state_dim_lin; jj++) {
      lower_bound(row_idx+jj) = 0.0;
      upper_bound(row_idx+jj) = 0.0;

      linear_con_mat.coeffRef(row_idx+jj, state_dim*(ii+1)+jj) = -1.0;
      for (size_t kk = 0; kk < state_dim_lin; kk++) {
        linear_con_mat.coeffRef(row_idx+jj, state_dim*ii+kk) = Ak_di(jj, kk);
      }

      for (size_t kk = 0; kk < control_dim_lin; kk++) {
        linear_con_mat.coeffRef(row_idx+jj, state_dim*N+control_dim*ii+kk) = Bk_di(jj, kk);
      }
    }

    // Nonlinear attitude dynamics second
    Vec7 Xprev_k = Xprev[ii].segment(6, 7);
    Vec3 Uprev_k = Uprev[ii].segment(3, 3);
    Vec7 fk = fs[ii];
    Vec7 ck = dh*(As[ii]*Xprev_k + Bs[ii]*Uprev_k - fk);
    for (size_t jj = 0; jj < state_dim_nlin; jj++) {
      lower_bound(row_idx+state_dim_lin+jj) = ck(jj);
      upper_bound(row_idx+state_dim_lin+jj) = ck(jj);

      // Simple explicit Euler integration scheme
      linear_con_mat.coeffRef(row_idx+state_dim_lin+jj, state_dim*(ii+1)+state_dim_lin+jj) = -1.0;
      for (size_t kk = 0; kk < state_dim_nlin; kk++) {
        linear_con_mat.coeffRef(row_idx+state_dim_lin+jj, state_dim*ii+state_dim_lin+kk) =
          eye(jj, kk)+dh*As[ii](jj, kk);
      }
      for (size_t kk = 0; kk < control_dim_nlin; kk++) {
        linear_con_mat.coeffRef(row_idx+state_dim_lin+jj,
          state_dim*N+control_dim*ii+control_dim_lin+kk) = dh*Bs[ii](jj, kk);
      }
    }
    row_idx += state_dim;
  }

  // Initial state
  for (size_t ii = 0; ii < state_dim; ii++) {
    linear_con_mat.coeffRef(row_idx+ii, ii) = 1.0;
    lower_bound(row_idx+ii) = x0(ii);
    upper_bound(row_idx+ii) = x0(ii);
  }
  row_idx += state_dim;

  // Goal state
  for (size_t ii = 0; ii < state_dim; ii++) {
    linear_con_mat.coeffRef(row_idx+ii, state_dim*(N-1)+ii) = 1.0;
    lower_bound(row_idx+ii) = xg(ii);
    upper_bound(row_idx+ii) = xg(ii);
  }
  row_idx += state_dim;
}

void TOP::UpdateSimpleCosts() {
  // Cost function
  for (size_t ii = 0; ii < N; ii++) {
    // Penalize only distance to xg and final linear velocity
    for (size_t jj = 0; jj < state_dim_lin; jj++) {
      hessian.coeffRef(state_dim*ii+jj, state_dim*ii+jj) = 10.0;
      gradient(state_dim*ii+jj) = -2*xg(jj);
    }
  }

  for (size_t ii = 0; ii < N-1; ii++) {
    for (size_t jj = 0; jj < control_dim; jj++) {
      hessian.coeffRef(state_dim*N+control_dim*ii+jj, state_dim*N+control_dim*ii+jj) = 10.0;
    }
  }
}

void TOP::ComputeSignedDistances() {
  size_t n_obs = keep_out_zones_->size();

  collision_checker::SignedDistanceResult sd_result;
  for (size_t ii = 0; ii < N-1; ii++) {
    for (size_t jj = 0; jj < n_obs; jj++) {
      // signed distance, point on co1, point on co2;
      cc.ComputeDistance(jj, sd_result);
      Vec3 nhat;
      if (sd_result.sd >  0) {
        nhat = (sd_result.co1_pt-sd_result.co2_pt);
      } else {
        nhat = (sd_result.co2_pt-sd_result.co1_pt);
      }
      nhat.normalize();
      obs_ub[n_obs*ii+jj] = sd_result.sd - nhat.dot(Xprev[ii].segment(0, 3)) - obs_clearance;
      support_vectors[n_obs*ii+jj] = -nhat;
    }
  }
}

void TOP::UpdateDoubleIntegrator() {
  Ak_di.setZero();
  Bk_di.setZero();

  for (size_t ii = 0; ii < 3; ii++) {
    Ak_di(ii, ii) = 1.;
    Ak_di(ii, 3+ii) = dh;
    Ak_di(ii+3, ii+3) = 1;

    Bk_di(ii, ii) = 0.5 * pow(dh, 2) / mass;
    Bk_di(3+ii, ii) = dh / mass;
  }
}

void TOP::UpdateF(Vec7& f, Vec13& X, Vec6& U) {
  f.setZero();

  Vec4 q = X.segment(6, 4);
  decimal_t q_norm = q.norm();
  q /= q_norm;

  Vec3 w = X.segment(10, 3);

  decimal_t wx = w(0);
  decimal_t wy = w(1);
  decimal_t wz = w(2);

  Mat4 omega_skew;
  omega_skew << 0, wz, -wy, wz,
            -wz, 0, wx, wy,
            wy, -wx, 0, wz,
            -wx, -wy, -wz, 0;

  // Eq. 305 in A Survey of Attitude Representations; Shuster 1993
  f.segment(0, 4) = 0.5 * omega_skew * q;
  f.segment(4, 3) = Jinv*(U.segment(3, 3) - w.cross(J*w));
}

void TOP::UpdateA(Mat7& A, Vec13& X, Vec6& U) {
  A.setZero();

  decimal_t wx = X(10);
  decimal_t wy = X(11);
  decimal_t wz = X(12);

  Mat4 omega_skew;
  omega_skew << 0, wz, -wy, wz,
            -wz, 0, wx, wy,
            wy, -wx, 0, wz,
            -wx, -wy, -wz, 0;

  A.block(0, 0, 4, 4) = omega_skew;

  decimal_t Jxx = J(0, 0);
  decimal_t Jyy = J(1, 1);
  decimal_t Jzz = J(2, 2);

  Mat3 quat_skew;
  quat_skew << 0, (Jyy-Jzz)*wz/Jxx, (Jyy-Jzz)*wy/Jxx,
              -(Jxx-Jzz)*wz/Jyy, 0, -(Jxx-Jzz)*wx/Jyy,
              (Jxx-Jyy)*wy/Jzz, (Jxx-Jyy)*wx/Jzz, 0;

  A.block(4, 4, 3, 3) = quat_skew;
}

void TOP::UpdateB(Mat7x3& B, Vec13& X, Vec6& U) {
  B.setZero();

  B.block(4, 0, 3, 3) = Jinv;
}

void TOP::UpdateDynamics() {
  // re-normalize quaternions between iterations
  for (size_t ii = 0; ii < N; ii++) {
    decimal_t q_norm = Xprev[ii].segment(6, 4).norm();
    Xprev[ii].segment(6, 4) /= q_norm;
  }

  for (size_t ii = 0; ii < N-1; ii++) {
    UpdateF(fs[ii], Xprev[ii], Uprev[ii]);
    UpdateA(As[ii], Xprev[ii], Uprev[ii]);
    UpdateB(Bs[ii], Xprev[ii], Uprev[ii]);
  }
}

void TOP::SetHessianMatrix() {
  size_t num_vars = GetNumTOPVariables();
  Qf.diagonal() << 1000, 1000, 1000, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
  R.diagonal() << 1, 1, 1, 1, 1, 1;

  int idx = state_dim*N;
  for (size_t ii = 0; ii < N-1; ii++) {
    for (size_t jj = 0; jj < control_dim; jj++) {
      // hessian.insert(idx+jj, idx+jj) = R.diagonal()[jj];
    }
    idx += control_dim;
  }

  for (size_t ii = 0; ii < num_vars; ii++) hessian.insert(ii, ii) = 0.0;
}

void TOP::SetGradient() {
  // Only slack variables associated with max(g(x),0) are penalized

  size_t row_idx = state_dim*N + 2*control_dim*(N-1);

  // Penalty for state upper and lower bounds
  for (size_t ii = 0; ii < 2*state_bd_dim*(N-1); ii++) {
    gradient(row_idx) = omega;
    row_idx++;
  }

  // Penalty for linear velocity violation
  for (size_t ii = 0; ii < N-1; ii++) {
    gradient(row_idx + 3) = omega;
    row_idx += 4;
  }

  // Penalty for angular velocity violation
  for (size_t ii = 0; ii < N-1; ii++) {
    gradient(row_idx + 3) = omega;
    row_idx += 4;
  }

  // Penalty for trust region constraint violation
  for (size_t ii = 0; ii < N-1; ii++) {
    gradient(row_idx + state_dim) = omega;
    row_idx += (state_dim+1);
  }

  // Penalty for collision avoidance
}

void TOP::SetBoundaryCons() {
  lower_bound.segment(0, state_dim) = x0;
  upper_bound.segment(0, state_dim) = x0;
  for (size_t ii = 0; ii < state_dim; ii++) {
    linear_con_mat.insert(ii, ii) = 1;
  }

  lower_bound.segment(state_dim, state_dim) = xg;
  upper_bound.segment(state_dim, state_dim) = xg;
  for (size_t ii = 0; ii < state_dim; ii++) {
    linear_con_mat.insert(state_dim+ii, state_dim*(N-1)+ii) = 1;
  }
}

void TOP::SetControlCons() {
  size_t row_idx = 2*state_dim;

  // col_idx tracks the slack variable under consideration
  size_t col_idx = state_dim*N + control_dim*(N-1);

  for (size_t ii = 0; ii < N-1; ii++) {
    // Linear acceleration
    // -sik-aik <= 0
    upper_bound.segment(row_idx, 3).setZero();
    for (size_t jj = 0; jj < 3; jj++) {
      linear_con_mat.insert(row_idx+jj, state_dim*N + control_dim*ii + jj) = -1;
      linear_con_mat.insert(row_idx+jj, col_idx+jj) = -1;
    }

    row_idx += 3;

    // aik-sik <= 0
    upper_bound.segment(row_idx, 3).setZero();
    for (size_t jj = 0; jj < 3; jj++) {
      linear_con_mat.insert(row_idx+jj, state_dim*N + control_dim*ii + jj) = 1;
      linear_con_mat.insert(row_idx+jj, col_idx+jj) = -1;
    }

    row_idx += 3;

    // sum(sik) <= a_max
    upper_bound(row_idx) = mass*desired_accel_;
    for (size_t jj = 0; jj < 3; jj++) {
      linear_con_mat.insert(row_idx, col_idx+jj) = 1;
    }

    row_idx++;
    col_idx += 3;

    // Angular acceleration
    // -sik-alpha_ik <= 0
    upper_bound.segment(row_idx, 3).setZero();
    for (size_t jj = 0; jj < 3; jj++) {
      linear_con_mat.insert(row_idx+jj, state_dim*N + control_dim*ii + 3 + jj) = -1;
      linear_con_mat.insert(row_idx+jj, col_idx+jj) = -1;
    }

    row_idx += 3;

    // alpha_ik - sik <= 0
    upper_bound.segment(row_idx, 3).setZero();
    for (size_t jj = 0; jj < 3; jj++) {
      linear_con_mat.insert(row_idx+jj, state_dim*N + control_dim*ii + 3 + jj) = 1;
      linear_con_mat.insert(row_idx+jj, col_idx+jj) = -1;
    }

    row_idx += 3;

    // sum(sik) <= alpha_max
    Vec3 alpha_;
    alpha_.setOnes();
    alpha_ *= desired_alpha_;
    Vec3 M_ = J*alpha_;
    upper_bound(row_idx) = M_.minCoeff();
    for (size_t jj = 0; jj < 3; jj++) {
      linear_con_mat.insert(row_idx, col_idx+jj) = 1;
    }

    row_idx++;
    col_idx += 3;
  }

  // Slack variables non-negative constraints
  upper_bound.segment(row_idx, control_dim*(N-1)).setZero();
  col_idx = state_dim*N + control_dim*(N-1);
  for (size_t ii = 0; ii < control_dim*(N-1); ii++) {
    linear_con_mat.insert(row_idx+ii, col_idx+ii) = -1;
  }
}

void TOP::SetStateCons() {
  // State LB
  size_t row_idx = 2*state_dim + 20*(N-1);

  // col_idx tracks the slack variable under consideration
  size_t col_idx = state_dim*N + 2*control_dim*(N-1);
  for (size_t ii = 0; ii < N-1; ii++) {
    // State LB
    for (size_t jj = 0; jj < 3; jj++) {
      // Position limits: -xik-zik <= -x_min_i
      upper_bound(row_idx+jj) = -x_min(jj);

      linear_con_mat.insert(row_idx+jj, state_dim*(ii+1)+jj) = -1;
      linear_con_mat.insert(row_idx+jj, col_idx+jj) = -1;
    }

    row_idx += 3;
    col_idx += 3;

    for (size_t jj = 0; jj < 4; jj++) {
      // Quaternion limits: -xik-zik <= -x_min_i
      upper_bound(row_idx+jj) = -x_min(6+jj);

      linear_con_mat.insert(row_idx+jj, state_dim*(ii+1)+6+jj) = -1;
      linear_con_mat.insert(row_idx+jj, col_idx+jj) = -1;
    }

    row_idx += 4;
    col_idx += 4;
  }

  // State UB
  for (size_t ii = 0; ii < N-1; ii++) {
    for (size_t jj = 0; jj < 3; jj++) {
      // Position limits: xik-zik <= x_max_i
      upper_bound(row_idx+jj) = x_max(jj);

      linear_con_mat.insert(row_idx+jj, state_dim*(ii+1)+jj) = 1;
      linear_con_mat.insert(row_idx+jj, col_idx+jj) = -1;
    }

    row_idx += 3;
    col_idx += 3;

    for (size_t jj = 0; jj < 4; jj++) {
      // Quaternion limits: xik-zik <= x_max_i
      upper_bound(row_idx+jj) = x_max(6+jj);

      linear_con_mat.insert(row_idx+jj, state_dim*(ii+1)+6+jj) = 1;
      linear_con_mat.insert(row_idx+jj, col_idx+jj) = -1;
    }

    row_idx += 4;
    col_idx += 4;
  }

  // Slack variables non-negative constraints
  upper_bound.segment(row_idx, 2*state_bd_dim*(N-1)).setZero();
  if (state_con_strict) {
    lower_bound.segment(row_idx, 2*state_bd_dim*(N-1)).setZero();
  }

  col_idx = state_dim*N + 2*control_dim*(N-1);
  for (size_t ii = 0; ii < 2*state_bd_dim*(N-1); ii++) {
    linear_con_mat.insert(row_idx+ii, col_idx+ii) = -1;
  }
}

void TOP::SetVelCons() {
  size_t row_idx = 2*state_dim + 20*(N-1) + 4*state_bd_dim*(N-1);

  // col_idx tracks the slack variable under consideration
  size_t col_idx = state_dim*N + 2*control_dim*(N-1) + 2*state_bd_dim*(N-1);

  // Linear velocity constraints
  for (size_t ii = 0; ii < N-1; ii++) {
    // -sk-vk <= 0.
    upper_bound.segment(row_idx, 3).setZero();
    for (size_t jj = 0; jj < 3; jj++) {
      linear_con_mat.insert(row_idx+jj, state_dim*(ii+1)+3+jj) = -1;
      linear_con_mat.insert(row_idx+jj, col_idx+jj) = -1;
    }

    row_idx += 3;

    // vk-sk <= 0.
    upper_bound.segment(row_idx, 3).setZero();
    for (size_t jj = 0; jj < 3; jj++) {
      linear_con_mat.insert(row_idx+jj, state_dim*(ii+1)+3+jj) = 1;
      linear_con_mat.insert(row_idx+jj, col_idx+jj) = -1;
    }

    row_idx += 3;

    // sum(sik) - zk <= v_max
    upper_bound(row_idx) = desired_vel_;
    for (size_t jj = 0; jj <3; jj++) {
      linear_con_mat.insert(row_idx, col_idx+jj) = 1;
    }
    linear_con_mat.insert(row_idx, col_idx+3) = -1;

    row_idx++;
    col_idx += 4;
  }

  // Slack variables non-negative constraints
  col_idx = state_dim*N + 2*control_dim*(N-1) + 2*state_bd_dim*(N-1);
  upper_bound.segment(row_idx, 4*(N-1)).setZero();
  if (lin_vel_strict) {
    // slack vars are set to 0 if constraint is to be strictly enforced
    lower_bound.segment(row_idx, 4*(N-1)).setZero();
  }
  for (size_t ii = 0; ii < 4*(N-1); ii++) {
    linear_con_mat.insert(row_idx+ii, col_idx+ii) = -1;
  }
}

void TOP::SetAngVelCons() {
  size_t row_idx = 2*state_dim + 20*(N-1) + 4*state_bd_dim*(N-1) + 11*(N-1);

  // col_idx tracks the slack variable under consideration
  size_t col_idx = state_dim*N + 2*control_dim*(N-1) + 2*state_bd_dim*(N-1) + 4*(N-1);

  for (size_t ii = 0; ii < N-1; ii++) {
    // -sk-wk <= 0.
    upper_bound.segment(row_idx, 3).setZero();
    for (size_t jj = 0; jj < 3; jj++) {
      linear_con_mat.insert(row_idx+jj, state_dim*(ii+1)+10+jj) = -1;
      linear_con_mat.insert(row_idx+jj, col_idx+jj) = -1;
    }

    row_idx += 3;

    upper_bound.segment(row_idx, 3).setZero();
    // wk-sk <= 0.
    for (size_t jj = 0; jj < 3; jj++) {
      linear_con_mat.insert(row_idx+jj, state_dim*(ii+1)+10+jj) = 1;
      linear_con_mat.insert(row_idx+jj, col_idx+jj) = -1;
    }

    row_idx += 3;

    // sum(sik) - zk <= v_max
    upper_bound(row_idx) = desired_omega_;
    for (size_t jj = 0; jj < 3; jj++) {
      linear_con_mat.insert(row_idx, col_idx+jj) = 1;
    }
    linear_con_mat.insert(row_idx, col_idx+3) = -1;

    row_idx++;
    col_idx += 4;
  }

  // Slack variables non-negative constraints
  col_idx = state_dim*N + 2*control_dim*(N-1) + 2*state_bd_dim*(N-1) + 4*(N-1);
  upper_bound.segment(row_idx, 4*(N-1)).setZero();
  if (ang_vel_strict) {
    // slack vars are set to 0 if constraint is to be strictly enforced
    lower_bound.segment(row_idx, 4*(N-1)).setZero();
  }
  for (size_t ii = 0; ii < 4*(N-1); ii++) {
    linear_con_mat.insert(row_idx+ii, col_idx+ii) = -1;
  }
}

void TOP::SetLinearDynamicsCons() {
  size_t row_idx = 2*state_dim + 20*(N-1) + 4*state_bd_dim*(N-1) + 2*11*(N-1);
  row_idx = 2*state_dim + 20*(N-1) + 4*state_bd_dim*(N-1);
  size_t control_dim_lin = static_cast<size_t>(control_dim/2);

  // Ak*xk+Bk*uk - x_{k+1} = 0
  for (size_t ii = 0; ii < N-1; ii++) {
    for (size_t jj = 0; jj < state_dim_lin; jj++) {
      linear_con_mat.coeffRef(row_idx+jj, state_dim*(ii+1)+jj) = -1.0;

      for (size_t kk = 0; kk < state_dim_lin; kk++) {
        if (Ak_di(jj, kk) == 0) {
          continue;
        }

        linear_con_mat.coeffRef(row_idx+jj, state_dim*ii+kk)     = Ak_di(jj, kk);
      }

      for (size_t kk = 0; kk < control_dim_lin; kk++) {
        if (Bk_di(jj, kk) == 0) {
          continue;
        }

        linear_con_mat.coeffRef(row_idx+jj, state_dim*N+control_dim*ii+kk) = Bk_di(jj, kk);
      }
    }

    lower_bound.segment(row_idx, state_dim_lin).setZero();
    upper_bound.segment(row_idx, state_dim_lin).setZero();
    row_idx += state_dim_lin;
  }
}

void TOP::SetDynamicsCons() {
  Mat7 eye;
  eye.setIdentity();

  Mat7 Ak;
  Mat7 Akp1;
  Mat7x3 Bk;
  Mat7x3 Bkp1;
  Vec7 ck;
  Vec7 Xprev_k, Xprev_kp1;
  Vec7 fk, fkp1;
  Vec3 Uprev_k, Uprev_kp1;

  size_t row_idx = 2*state_dim + 20*(N-1) + 4*state_bd_dim*(N-1) + 2*11*(N-1) + state_dim_lin*(N-1);
  row_idx = 2*state_dim + 20*(N-1) + 4*state_bd_dim*(N-1) + state_dim_lin*(N-1);
  size_t control_dim_nlin = static_cast<size_t>(control_dim/2);

  // Trapezoidal integration for ii = 0,..,N-3
  for (size_t ii = 0; ii < N-2; ii++) {
    // Assign Ak matrices
    Ak    = 0.5*dh*As[ii]+eye;
    Akp1  = 0.5*dh*As[ii+1]-eye;

    // Assign Bk matrices
    Bk    = 0.5*dh*Bs[ii];
    Bkp1  = 0.5*dh*Bs[ii+1];

    for (size_t jj = 0; jj < state_dim_nlin; jj++) {
      for (size_t kk = 0; kk < state_dim_nlin; kk++) {
          linear_con_mat.coeffRef(row_idx+jj, state_dim*ii+kk)     = Ak(jj, kk);
          linear_con_mat.coeffRef(row_idx+jj, state_dim*(ii+1)+kk) = Akp1(jj, kk);
      }

      for (size_t kk = 0; kk < control_dim_nlin; kk++) {
        // add +3 to column index to grab (Mx,My,Mz) component
        linear_con_mat.coeffRef(row_idx+jj, state_dim*N+control_dim*ii+3+kk)      = Bk(jj, kk);
        linear_con_mat.coeffRef(row_idx+jj, state_dim*N+control_dim*(ii+1)+3+kk)  = Bkp1(jj, kk);
      }
    }

    // Assign ck vectors
    Xprev_k = Xprev[ii].segment(6, 7);
    Xprev_kp1 = Xprev[ii+1].segment(6, 7);
    Uprev_k = Uprev[ii].segment(3, 3);
    Uprev_kp1 = Uprev[ii+1].segment(3, 3);
    fk = fs[ii];
    fkp1 = fs[ii+1];

    ck = 0.5*dh*(
      As[ii]*Xprev_k + Bs[ii]*Uprev_k
      + As[ii+1]*Xprev_kp1 + Bs[ii+1]*Uprev_kp1
      - fk - fkp1);
    lower_bound.segment(row_idx, state_dim_nlin) = ck;
    upper_bound.segment(row_idx, state_dim_nlin) = ck;

    row_idx += state_dim_nlin;
  }

  // Euler integration for last step
  Ak = dh*As[N-2] + eye;
  Akp1 = -eye;
  Bk = dh*Bs[N-2];
  fk = fs[N-2];
  Xprev_k = Xprev[N-2].segment(6, 7);
  Uprev_k = Uprev[N-2].segment(3, 3);

  ck = dh*(As[N-2]*Xprev_k + Bs[N-2]*Uprev_k - fk);
  for (size_t jj = 0; jj < state_dim_nlin; jj++) {
    for (size_t kk = 0; kk < state_dim_nlin; kk++) {
      linear_con_mat.coeffRef(row_idx+jj, state_dim*(N-2)+kk)      = Ak(jj, kk);
      linear_con_mat.coeffRef(row_idx+jj, state_dim*(N-1)+kk)      = Akp1(jj, kk);
    }
    for (size_t kk = 0; kk < control_dim_nlin; kk++) {
      linear_con_mat.coeffRef(row_idx+jj, state_dim*N+control_dim*(N-2)+kk)   = Bk(jj, kk);
    }
  }
  lower_bound.segment(row_idx, state_dim_nlin) = ck;
  upper_bound.segment(row_idx, state_dim_nlin) = ck;
}

void TOP::SetTrustRegionCons() {
  size_t row_idx = 2*state_dim + 20*(N-1) + 4*state_bd_dim*(N-1) + 2*11*(N-1) +
    state_dim*(N-1);
  size_t col_idx = state_dim*N + 2*control_dim*(N-1) + 2*state_bd_dim*(N-1) + 2*4*(N-1);

  // Ordering of slack variables here is (s1,z1,...,s_{N-1},z_{N-1})
  // where s1 \in R^{n_x} and z1 \in R

  for (size_t ii = 0; ii < N-1; ii++) {
    // -sk-Xk <= -Xkp
    upper_bound.segment(row_idx, state_dim) = -Xprev[ii+1];
    for (size_t jj = 0; jj < state_dim; jj++) {
      linear_con_mat.insert(row_idx+jj, state_dim*(ii+1)+jj) = -1;
      linear_con_mat.insert(row_idx+jj, col_idx+jj) = -1;
    }

    row_idx += (state_dim);

    // Xk-sk <= Xkp
    upper_bound.segment(row_idx, state_dim) = Xprev[ii+1];
    for (size_t jj = 0; jj < state_dim; jj++) {
      linear_con_mat.insert(row_idx+jj, state_dim*(ii+1)+jj) = 1;
      linear_con_mat.insert(row_idx+jj, col_idx+jj) = -1;
    }

    row_idx += (state_dim);

    // \sum(sk)-zk \leq Delta
    upper_bound(row_idx) = Delta;
    for (size_t jj = 0; jj < state_dim; jj++) {
      linear_con_mat.insert(row_idx, col_idx+jj) = 1;
    }
    linear_con_mat.insert(row_idx, col_idx+state_dim) = -1;

    row_idx++;
    col_idx += (state_dim+1);
  }

  // Slack variables non-negative constraints
  col_idx = state_dim*N + 2*control_dim*(N-1) + 2*state_bd_dim*(N-1) + 2*4*(N-1);
  upper_bound.segment(row_idx, (state_dim+1)*(N-1)).setZero();
  for (size_t ii = 0; ii < (N-1)*(state_dim+1); ii++) {
    linear_con_mat.insert(row_idx, col_idx+ii) = -1;
    row_idx++;
  }
}

void TOP::SetObsCons() {
  size_t row_idx = 2*state_dim + 20*(N-1) + 4*7*(N-1) + 2*11*(N-1) +
    state_dim*(N-1) + (3*state_dim+2)*(N-1);
  size_t col_idx = state_dim*N + 2*control_dim*(N-1) +
    2*7*(N-1) + 2*4*(N-1)  + (state_dim+1)*(N-1);
  size_t n_obs = keep_out_zones_->size();

  Vec3 support_vec;

  for (size_t ii = 0; ii < N-1; ii++) {
    for (size_t jj = 0; jj < n_obs; jj++) {
      support_vec = support_vectors[n_obs*ii+jj];

      upper_bound(row_idx) = obs_ub[n_obs*ii+jj];
      linear_con_mat.insert(row_idx, col_idx) = -1;    // -z_{k,m}
      for (size_t kk = 0; kk < 3; kk++) {
        linear_con_mat.coeffRef(row_idx, state_dim*(ii+1)+kk) = support_vec(kk);
      }

      row_idx++;
      col_idx++;
    }
  }

  // Slack variables non-negative constraints
  col_idx = state_dim*N + control_dim*(N-1) +
    2*7*(N-1) + 2*4*(N-1)  + (state_dim+1)*(N-1);
  upper_bound.segment(row_idx, n_obs*(N-1)).setZero();
  for (size_t ii = 0; ii < n_obs*(N-1); ii++) {
    linear_con_mat.insert(row_idx, col_idx) = -1;
    row_idx++;
    col_idx++;
  }
}

void TOP::UpdateBoundaryCons() {
  lower_bound.segment(0, state_dim) = x0;
  upper_bound.segment(0, state_dim) = x0;

  lower_bound.segment(state_dim, state_dim) = xg;
  upper_bound.segment(state_dim, state_dim) = xg;
}

void TOP::UpdateControlCons() {
  size_t row_idx = 2*state_dim;

  for (size_t ii = 0; ii < N-1; ii++) {
    // Linear accelereation
    row_idx += 6;
    upper_bound(row_idx) = mass*desired_accel_;

    row_idx++;

    // Angular acceleration
    row_idx += 6;
    Vec3 alpha_;
    alpha_.setOnes();
    alpha_ *= desired_alpha_;
    Vec3 M_ = J*alpha_;
    upper_bound(row_idx) = M_.minCoeff();

    row_idx++;
  }
}

void TOP::UpdateStateCons() {
  // Update any changes to params
  x_max << pos_max_(0), pos_max_(1), pos_max_(2),
    desired_vel_/std::sqrt(3), desired_vel_/std::sqrt(3), desired_vel_/std::sqrt(3),
    1, 1, 1, 1,
    desired_omega_/std::sqrt(3), desired_omega_/std::sqrt(3), desired_omega_/std::sqrt(3);
  x_min = -x_max;
  x_min(0) = pos_min_(0);
  x_min(1) = pos_min_(1);
  x_min(2) = pos_min_(2);

  size_t row_idx = 2*state_dim + 20*(N-1);

  // State LB
  for (size_t ii = 0; ii < N-1; ii++) {
    // Position limits
    for (size_t jj = 0; jj < 3; jj++) {
      upper_bound(row_idx+jj) = -x_min(jj);
    }

    row_idx += 3;

    // Quaternion limits
    for (size_t jj = 0; jj < 4; jj++) {
      upper_bound(row_idx+jj) = -x_min(6+jj);
    }

    row_idx += 4;
  }

  // State UB
  for (size_t ii = 0; ii < N-1; ii++) {
    for (size_t jj = 0; jj < 3; jj++) {
      upper_bound(row_idx+jj) = x_max(jj);
    }

    row_idx += 3;

    for (size_t jj = 0; jj < 4; jj++) {
      upper_bound(row_idx+jj) = x_max(6+jj);
    }

    row_idx += 4;
  }

  // Slack variables non-negative constraints
  upper_bound.segment(row_idx, 2*state_bd_dim*(N-1)).setZero();
  if (state_con_strict) {
    lower_bound.segment(row_idx, 2*state_bd_dim*(N-1)).setZero();
  }
}

void TOP::UpdateVelCons() {
  size_t row_idx = 2*state_dim + 20*(N-1) + 4*state_bd_dim*(N-1);

  // Linear velocity constraints
  for (size_t ii = 0; ii < N-1; ii++) {
    row_idx += 6;

    upper_bound(row_idx) = desired_vel_;

    row_idx++;
  }

  // Slack variables non-negative constraints
  upper_bound.segment(row_idx, 4*(N-1)).setZero();
  if (lin_vel_strict) {
    // slack vars are set to 0 if constraint is to be strictly enforced
    lower_bound.segment(row_idx, 4*(N-1)).setZero();
  }
}

void TOP::UpdateAngVelCons() {
  size_t row_idx = 2*state_dim + 20*(N-1) + 4*state_bd_dim*(N-1) + 11*(N-1);

  for (size_t ii = 0; ii < N-1; ii++) {
    row_idx += 6;

    upper_bound(row_idx) = desired_omega_;

    row_idx++;
  }

  upper_bound.segment(row_idx, 4*(N-1)).setZero();
  if (ang_vel_strict) {
    // slack vars are set to 0 if constraint is to be strictly enforced
    lower_bound.segment(row_idx, 4*(N-1)).setZero();
  }
}

void TOP::UpdateTrustRegionCons() {
  size_t row_idx = 2*state_dim + 20*(N-1) + 4*state_bd_dim*(N-1) + 2*11*(N-1) +
    state_dim*(N-1);

  for (size_t ii = 0; ii < N-1; ii++) {
    // -sk-Xk <= -Xkp
    upper_bound.segment(row_idx, state_dim) = -Xprev[ii+1];
    row_idx += (state_dim);

    // Xk-sk <= Xkp
    upper_bound.segment(row_idx, state_dim) = Xprev[ii+1];
    row_idx += (state_dim);

    // \sum(sk)-zk \leq Delta
    upper_bound(row_idx) = Delta;
    row_idx++;
  }
}

void TOP::UpdateGradient() {
  // TODO(acauligi)
}

void TOP::UpdateObsCons() {
  size_t row_idx = 2*state_dim + 20*(N-1) + 4*7*(N-1) + 2*11*(N-1) +
    state_dim*(N-1) + (3*state_dim+2)*(N-1);
  size_t n_obs = keep_out_zones_->size();

  Vec3 support_vec;

  for (size_t ii = 0; ii < N-1; ii++) {
    for (size_t jj = 0; jj < n_obs; jj++) {
      support_vec = support_vectors[n_obs*ii+jj];

      upper_bound(row_idx) = obs_ub[n_obs*ii+jj];
      for (size_t kk = 0; kk < 3; kk++) {
        linear_con_mat.coeffRef(row_idx, state_dim*(ii+1)+kk) = support_vec(kk);
      }

      row_idx++;
    }
  }
}

decimal_t TOP::ConvergenceMetric() {
  decimal_t max_num = -OsqpEigen::INFTY;
  decimal_t max_den = -OsqpEigen::INFTY;

  for (size_t ii = 0; ii < N; ii++) {
    decimal_t val = (qp_soln.block(state_dim*ii, 0, state_dim, 1) - Xprev[ii]).norm();
    max_num = (val > max_num) ? val : max_num;

    val = Xprev[ii].norm();
    max_den = (val > max_den) ? val : max_den;
  }
  return max_num*100.0/max_den;
}

decimal_t TOP::AccuracyRatio() {
  decimal_t num = 0;
  decimal_t den = 0;

  Vec13 X_k;
  Vec6 U_k;
  Vec7 f_k;
  Vec7 linearized;

  // dynamics
  for (size_t ii = 0; ii < N-1; ii++) {
    X_k = qp_soln.segment(state_dim*ii, state_dim);
    decimal_t q_norm = X_k.segment(6, 4).norm();
    X_k.segment(6, 4) /= q_norm;

    U_k = qp_soln.segment(state_dim*N+control_dim*ii, control_dim);

    UpdateF(f_k, X_k, U_k);

    // TODO(acauligi): determine whether A_kp,B_kp,f_kp need to be recomputed

  Vec7 ck;
  Vec7 X_k, Xprev_k;
  Vec7 f_k;
  Vec3 U_k, Uprev_k;

    linearized = fs[ii] +
      As[ii]*(X_k.segment(7, 6)-Xprev[ii].segment(7, 6)) +
      Bs[ii]*(U_k.segment(3, 3)-Uprev[ii].segment(3, 3));
    num += (f_k - linearized).norm();
    den += linearized.norm();
  }

  // TODO(acauligi)
  // obstacles

  return num*100.0/den;
}

bool TOP::TrustRegionSatisfied() {
  Vec13 diff;
  for (size_t ii = 0; ii < N; ii++) {
    diff = qp_soln.segment(state_dim*ii, state_dim)-Xprev[ii];
    // TODO(acauligi): use lp-norm here
    if (diff.norm() > Delta) {
      return false;
    }
  }
  return true;
}

bool TOP::SatisfiesStateInequalityConstraints() {
  // State box constraints
  for (size_t ii = 0; ii < N; ii++) {
    for (size_t jj = 0; jj < state_dim; jj++) {
      if (qp_soln(state_dim*ii+jj) > x_max(jj)) {
        return false;
      } else if (qp_soln(state_dim*ii+jj) < x_min(jj)) {
        return false;
      }
    }
  }

  // Linear and angular velocity norm constraints
  for (size_t ii = 0; ii < N; ii++) {
    if (qp_soln.segment(state_dim*ii, 3).lpNorm<1>() > desired_vel_) {
      return false;
    } else if (qp_soln.segment(state_dim*ii+10, 3).lpNorm<1>() > desired_omega_) {
      return false;
    }
  }

  // Trust region constraints: already checked in TrustRegionSatisfied()

  // Obstacle avoidance constraints
  size_t n_obs = keep_out_zones_->size();
  for (size_t ii = 0; ii < N; ii++) {
    for (size_t jj = 0 ; jj < n_obs; jj++) {
      if (support_vectors[n_obs*ii+jj].dot(qp_soln.segment(state_dim*ii, 3)) >= obs_ub[n_obs*ii+jj]) {
        return false;
      }
    }
  }

  return true;
}

bool TOP::Solve() {
  solved_ = false;
  InitTrajStraightline();
  // UpdateBoundaryCons();

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

  for (size_t kk = 0; kk < max_iter; kk++) {
    // update constraint matrix
    UpdateDynamics();
    UpdateSimpleConstraints();
    UpdateSimpleCosts();
    // SetDynamicsCons();
    // UpdateTrustRegionCons();
    // UpdateGradient();
    // UpdateObsCons();

    // solver.data()->clearLinearConstraintsMatrix();
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
    for (size_t jj = 0; jj < N; jj++) {
      Xprev[jj] = qp_soln.block(state_dim*jj, 0, state_dim, 1);
    }
    for (size_t jj = 0; jj < N-1; jj++) {
      Uprev[jj] = qp_soln.block(state_dim*N + control_dim*jj, 0, control_dim, 1);
    }
    // if (solved_ && state_ineq_con_satisfied) {
    //   return true;
    // }
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

void TOP::PolishSolution() {
  if (!solved_) {
    return;
  }

  // re-normalize quaternions between iterations
  for (size_t ii = 0; ii < N; ii++) {
    decimal_t q_norm = Xprev[ii].segment(6, 4).norm();
    Xprev[ii].segment(6, 4) /= q_norm;
  }
}

}  //  namespace scp
