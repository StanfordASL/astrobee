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

#ifndef SCP_TRAJ_OPT_H_
#define SCP_TRAJ_OPT_H_

#include <scp/types.h>
#include <scp/bullet_collision_checker.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>
#include <string>

#include "OsqpEigen/OsqpEigen.h"

namespace scp {

class TOP {
 public:
  size_t state_dim;
  size_t control_dim;
  size_t state_bd_dim;
  size_t N;
  decimal_t dh;
  decimal_t Tf;
  bool free_final_state;

  collision_checker::BulletCollisionChecker cc;

  OsqpEigen::Solver* solver;
  decimal_t abs_tol_;
  bool verbose_;
  bool warm_start_;
  bool solver_ready_;
  bool solved_;

  Vec13 x0;
  Vec13 xg;
  Vec13Vec Xprev;
  Vec6Vec Uprev;

  // allocate QP problem matrices and vectores
  SparseMatD hessian;
  VecD gradient;
  SparseMatD linear_con_mat;
  VecD lower_bound;
  VecD upper_bound;

  DiagMat6 R;
  DiagMat13 Q;
  DiagMat13 Qf;
  Vec6 u_min;
  Vec6 u_max;
  Vec13 x_min;
  Vec13 x_max;

  decimal_t radius_;
  decimal_t mass;
  Mat3 J;
  Mat3 Jinv;

  // SCP parameters
  size_t max_iter;
  decimal_t Delta_0;
  decimal_t Delta;
  decimal_t omega_0;
  decimal_t omega;
  decimal_t omega_max;
  decimal_t rho_0;
  decimal_t rho_1;
  decimal_t beta_fail;
  decimal_t beta_succ;
  decimal_t gamma_fail;
  decimal_t convergence_threshold;
  VecD qp_soln;

  Vec13Vec fs;
  Mat13Vec As;
  Mat13x6Vec Bs;

  std::vector<Eigen::AlignedBox3d>* keep_in_zones_;
  std::vector<Eigen::AlignedBox3d>* keep_out_zones_;
  Vec3Vec support_vectors;
  std::vector<decimal_t> obs_ub;
  decimal_t obs_clearance;

  Vec3 pos_min_;              // Bounds of keep-in region
  Vec3 pos_max_;              // Bounds of keep-in region

  // Solver params
  bool faceforward_;          // Face-forward trajectory?
  bool check_obstacles_;      // Perform obstacle checking
  decimal_t desired_vel_;     // Soft limit on velocity
  decimal_t desired_accel_;   // Soft limit on accel
  decimal_t desired_omega_;   // Soft limit on omega
  decimal_t desired_alpha_;   // Soft limit on alpha
  decimal_t max_time_;        // Max Tf
  decimal_t control_rate_;        // Control frequency

  TOP(decimal_t Tf, int N);

  // ~TOP();

  size_t GetNumTOPVariables();
  size_t GetNumTOPConstraints();

  void ResetSCPParams();
  void UpdateProblemDimension(size_t N_);

  void ComputeSignedDistances();
  void InitTrajStraightline();
  void UpdateF(Vec13& f, Vec13& X, Vec6& U);
  void UpdateA(Mat13& A, Vec13& X, Vec6& U);
  void UpdateB(Mat13x6& B, Vec13& X, Vec6& U);
  void UpdateDynamics();

  void SetHessianMatrix();
  void SetGradient();

  void SetBoundaryCons();
  void SetControlCons();
  void SetStateCons();
  void SetVelCons();
  void SetAngVelCons();

  void SetDynamicsMatrices();
  void SetTrustRegionCons();
  void SetObsCons();

  void UpdateBoundaryCons();
  void UpdateControlCons();
  void UpdateStateCons();
  void UpdateVelCons();
  void UpdateAngVelCons();
  void UpdateTrustRegionCons();
  void UpdateGradient();
  void UpdateObsCons();

  decimal_t ConvergenceMetric();
  decimal_t AccuracyRatio();
  bool TrustRegionSatisfied();
  bool SatisfiesStateInequalityConstraints();

  bool Solve();
  void PolishSolution();
};

}  //  namespace scp

#endif  // SCP_TRAJ_OPT_H_
