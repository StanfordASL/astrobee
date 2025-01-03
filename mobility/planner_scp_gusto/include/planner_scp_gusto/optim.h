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

#ifndef PLANNER_SCP_GUSTO_OPTIM_H_
#define PLANNER_SCP_GUSTO_OPTIM_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>
#include <string>
#include <cmath>

#include "planner_scp_gusto/types.h"
#include "planner_scp_gusto/bullet_collision_checker.h"

#include "OsqpEigen/OsqpEigen.h"

#ifdef PROFILING
#undef PROFILING
#endif
#include <torch/torch.h>

namespace scp {

// Define the NeuralNet struct for warm start
struct Net : torch::nn::Module {
  Net() {
    fc1 = register_module("fc1", torch::nn::Linear(26, 128));
    fc2 = register_module("fc2", torch::nn::Linear(128, 64));
    fc3 = register_module("fc3", torch::nn::Linear(64, 12));
  }

  // // Load weights from a file
  // void loadWeights(const std::string& filename) {
  //   // torch::load(*this, filename);
  //   auto m = torch::jit::load(filename);
  //   // Copy the weights to the model
  //   for (torch::jit::script::Named<at::Tensor> p :
  //       m.named_parameters(/*recurse=*/true)) {
  //     for (torch::jit::script::Named<at::Tensor> p2 :
  //         this->named_parameters(/*recurse=*/true)) {
  //       if (p.name == p2.name) {
  //         p2.value = p.value;
  //       }
  //     }
  //   }
  //   for (torch::jit::script::Named<at::Tensor> p :
  //       m.named_parameters(/*recurse=*/true)) {
  //     std::cout << p.name << ": " << p.value << "\n";
  //   }
  // }

  // // Method to initialize all weights to zero
  // void initializeWeightsToZero() {
  //   for (auto& param : this->parameters()) {
  //       param.data().zero_();
  //   }
  // }

  torch::Tensor forward(torch::Tensor x) {
    x = torch::relu(fc1->forward(x));
    x = torch::relu(fc2->forward(x));
    x = fc3->forward(x);
    return x;
  }

  // Use one of many "standard library" modules.
  torch::nn::Linear fc1{nullptr}, fc2{nullptr}, fc3{nullptr};
};

class TOP {
 public:
  size_t state_dim;
  size_t state_dim_lin;
  size_t state_dim_nlin;
  size_t control_dim;
  size_t control_dim_lin;
  size_t control_dim_nlin;
  size_t state_bd_dim;
  size_t pos_dim;
  size_t quat_dim;
  size_t lin_vel_dim;
  size_t ang_vel_dim;
  size_t N;
  decimal_t dh;
  decimal_t Tf;

  bool is_granite;
  bool enforce_init_cond;
  bool enforce_final_cond;
  bool enforce_lin_dynamics;
  bool enforce_rot_dynamics;
  bool enforce_force_norm;
  bool enforce_moment_norm;
  bool enforce_state_LB;
  bool enforce_state_UB;
  bool enforce_lin_vel_norm;
  bool enforce_ang_vel_norm;
  bool enforce_trust_region_const;
  bool enforce_obs_avoidance_const;
  bool enforce_state_bounds;

  bool penalize_total_force;
  bool penalize_total_moment;
  size_t num_force_norm_slack_vars;
  size_t num_force_norm_slack_vars_per_iter;
  size_t num_moment_norm_slack_vars;
  size_t num_moment_norm_slack_vars_per_iter;

  bool free_final_state;
  bool state_con_strict;
  bool lin_vel_strict;
  bool ang_vel_strict;

  // scp::BulletCollisionChecker cc;

  OsqpEigen::Solver* solver;
  decimal_t abs_tol_;
  decimal_t rel_tol_;
  decimal_t primal_tol_;
  decimal_t dual_tol_;
  decimal_t rho_;
  decimal_t sigma_;
  size_t max_iter_solver_;
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
  size_t n_iter;
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

  Vec7Vec fs;
  Mat7Vec As;
  Mat7x3Vec Bs;
  Mat6 Ak_di;     // Ak for double integrator
  Mat6x3 Bk_di;   // Bk for double integrator

  std::vector<Eigen::AlignedBox3d> keep_in_zones_;
  std::vector<Eigen::AlignedBox3d> keep_out_zones_;
  Vec3Vec support_vectors;
  std::vector<decimal_t> obs_ub;
  decimal_t obs_clearance;

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

  ~TOP();

  size_t GetNumTOPVariables();
  size_t GetNumTOPConstraints();

  Vec3 MinPos();
  Vec3 MaxPos();

  void ResetSCPParams();
  void UpdateProblemDimension(size_t N_);

  void SetSimpleConstraints();
  void SetSimpleCosts();

  void PrettyPrintConstraints();
  std::string ConvertiiToString(size_t ii);


  void UpdateSimpleConstraints();
  void UpdateSimpleCosts();

  Vec3 ComputeSignedDistanceGradient(const Vec3& point);
  decimal_t ComputeSignedDistance(const Vec3& point);

  void ComputeSignedDistances();
  void InitTrajStraightline();
  void UpdateDoubleIntegrator();
  void UpdateF(Vec7& f, Vec13& X, Vec6& U);
  void UpdateA(Mat7& A, Vec13& X, Vec6& U);
  void UpdateB(Mat7x3& B, Vec13& X, Vec6& U);
  void UpdateRotationalDynamics();
  Mat4x3 CalculateQMat(const Vec4& quaternion);

  void SetHessianMatrix();
  void SetGradient();

  void SetBoundaryCons();
  void SetControlCons();
  void SetStateCons();
  void SetVelCons();
  void SetAngVelCons();

  void SetLinearDynamicsCons();
  void SetDynamicsCons();
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
  void ValidationChecks();
  void NormalizeQuaternions();
  void WriteTrajectoryToFile(const Vec13Vec& states, const Vec6Vec& controls, const std::string& filename);
  void WriteTrajectoryToFileForNN(const Vec13& x0, const Vec13& xg, int N, const Vec13Vec& Xsoln, const Vec6Vec& Usoln,
                                  const std::string& fname);

  // Neural network for warm start
  std::shared_ptr<Net> net;
  torch::optim::Adam optimizer;

  // Function to read a single dataset file
  std::tuple<torch::Tensor, torch::Tensor> ReadData(const std::string& filename);

  // Function to train the model
  void TrainModel(const std::vector<std::string>& files, int epochs);

  // Function to save the model to disk
  void SaveModel(const std::string& model_path);

  // Function to load the model from disk
  void LoadModel(const std::string& model_path);

  Vec13 ForwardDynamics(Vec13 x, Vec6 u);
  std::tuple<Vec6, Vec6> InferenceNN(Vec13 x0, Vec13 xg);
  std::tuple<Vec13Vec, Vec6Vec> WarmStartFromNN(Vec13 x0, Vec13 xg);
};

}  //  namespace scp

#endif  // PLANNER_SCP_GUSTO_OPTIM_H_
