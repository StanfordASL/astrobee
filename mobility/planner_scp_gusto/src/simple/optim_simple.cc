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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>

namespace scp {

class TOPSimp {
 public:
  using Vec13 = Eigen::Matrix<double, 13, 1>;
  using Vec6 = Eigen::Matrix<double, 6, 1>;
  using Vec13Vec = std::vector<Vec13>;
  using Vec6Vec = std::vector<Vec6>;

  size_t N;               // Number of time steps
  double Tf;              // Final time
  double dt;              // Time step duration
  Vec13 x0, xg;           // Initial and goal states
  Vec13Vec Xprev;         // State trajectory
  Vec6Vec Uprev;          // Control trajectory
  double obs_clearance;   // Clearance distance
  std::vector<Eigen::AlignedBox3d> keep_out_zones_;  // Obstacles
  OsqpEigen::Solver solver;                          // OSQP solver instance

  Eigen::SparseMatrix<double> A;     // Constraint matrix
  Eigen::VectorXd lower_bound;       // Lower bounds for constraints
  Eigen::VectorXd upper_bound;       // Upper bounds for constraints
  int row_offset;                    // Offset for constraint rows

  // Constructor
  TOPSimp(double Tf_, size_t N_) : Tf(Tf_), N(N_) {
    dt = Tf / (N - 1);
    obs_clearance = 0.1;  // Example clearance
    x0.setZero();
    xg.setZero();
    Xprev.resize(N, Vec13::Zero());
    Uprev.resize(N - 1, Vec6::Zero());  // One less control variable than states
  }

  void SetBoundaryCons();
  void SetObsCons();
  int AddLinearDynamics(size_t offset);
  int AddControlLimits(size_t offset);
  bool Solve();
  void InitTrajStraightline();
  void WriteTrajectoryToFile(const Vec13Vec& states, const Vec6Vec& controls, const std::string& filename);

 private:
  Eigen::SparseMatrix<double> H;     // Hessian for the objective function
  Eigen::VectorXd gradient;          // Gradient vector
  // Function to check if the point is inside any obstacle
  bool IsInsideObstacle(const Vec3& point);
};

void TOPSimp::SetBoundaryCons() {
  std::cout << "Setting boundary constraints..." << std::endl;
  for (size_t i = 0; i < 13; ++i) {
    Xprev[0][i] = x0[i];  // Set initial state
    Xprev[N - 1][i] = xg[i];  // Set goal state
  }
}

// Returns the number of constraints added
int TOPSimp::AddLinearDynamics(size_t offset) {
  std::cout << "Adding double integrator dynamics with controls..." << std::endl;
  std::vector<Eigen::Triplet<double>> dynamics_triplets;

  size_t row_offset = offset;  // Start from the end of previous constraints

  for (size_t i = 0; i < N - 1; ++i) {
    // Double integrator dynamics for x, y, z
    for (size_t j = 0; j < 3; ++j) {  // j = 0 (x), 1 (y), 2 (z)
      // Position update: x_{i+1} = x_i + v_i * dt
      dynamics_triplets.emplace_back(row_offset, i * 13 + j, -1.0);       // -x_i
      dynamics_triplets.emplace_back(row_offset, i * 13 + 3 + j, -dt);   // -v_i * dt
      dynamics_triplets.emplace_back(row_offset, (i + 1) * 13 + j, 1.0);  // x_{i+1}
      ++row_offset;

      // Velocity update: v_{i+1} = v_i + u_j * dt
      dynamics_triplets.emplace_back(row_offset, i * 13 + 3 + j, -1.0);       // -v_i
      dynamics_triplets.emplace_back(row_offset, N * 13 + i * 6 + j, -dt);    // -u_j * dt
      dynamics_triplets.emplace_back(row_offset, (i + 1) * 13 + 3 + j, 1.0);  // v_{i+1}
      ++row_offset;
    }
  }

  // Add dynamics constraints to A matrix
  for (const auto& triplet : dynamics_triplets) {
    A.insert(triplet.row(), triplet.col()) = triplet.value();
  }

  // Set bounds for equality (zero for dynamics constraints)
  for (size_t i = offset; i < row_offset; ++i) {
    lower_bound[i] = 0.0;
    upper_bound[i] = 0.0;
  }

  int num_constraints = row_offset - offset;
  if (num_constraints != 2 * 3 * (N - 1)) {
    std::cerr << "Error: Expected " << 2 * 3 * (N - 1) << " constraints, but added " << num_constraints
              << " constraints." << std::endl;
  }
  return num_constraints;
}

// Return the number of constraints added
int TOPSimp::AddControlLimits(size_t offset) {
  size_t row_offset = offset;

  for (size_t i = 0; i < N - 1; ++i) {
    // Get the controls
    for (size_t j = 0; j < 3; ++j) {
      A.insert(row_offset, N * 13 + i * 6 + j) = 1.0;
      lower_bound[row_offset] = -1.0;
      upper_bound[row_offset] = 1.0;
      ++row_offset;
    }
  }
  int num_constraints = row_offset - offset;
  if (num_constraints != 3 * (N - 1)) {
    std::cerr << "Error: Expected " << 3 * (N - 1) << " constraints, but added " << num_constraints << " constraints."
              << std::endl;
  }
  return num_constraints;
}



// void TOPSimp::AddLinearDynamicsWithControls() {
//   std::cout << "Adding linear dynamics with controls..." << std::endl;
//   std::vector<Eigen::Triplet<double>> dynamics_triplets;

//   size_t row_offset = 2 * 13;  // After boundary constraints
//   for (size_t i = 0; i < N - 1; ++i) {
//     for (size_t j = 0; j < 13; ++j) {
//       // Dynamics: x_{i+1} = A * x_i + B * u_i
//       dynamics_triplets.emplace_back(row_offset + j, i * 13 + j, -1.0);  // -A * x_i
//       dynamics_triplets.emplace_back(row_offset + j, (i + 1) * 13 + j, 1.0);  // x_{i+1}
//     }
//     for (size_t j = 0; j < 6; ++j) {
//       dynamics_triplets.emplace_back(row_offset + j, N * 13 + i * 6 + j, -1.0);  // -B * u_i
//     }
//     row_offset += 13;
//   }

//   // Add dynamics constraints to A matrix
//   for (const auto& triplet : dynamics_triplets) {
//     A.insert(triplet.row(), triplet.col()) = triplet.value();
//   }

//   // Set bounds for equality (zero for dynamics constraints)
//   for (size_t i = 2 * 13; i < row_offset; ++i) {
//     lower_bound[i] = 0.0;
//     upper_bound[i] = 0.0;
//   }

//   // Add control bounds (-1 <= u_i <= 1)
//   for (size_t i = 0; i < (N - 1) * 6; ++i) {
//     lower_bound[2 * 13 + (N - 1) * 13 + i] = -1.0;
//     upper_bound[2 * 13 + (N - 1) * 13 + i] = 1.0;
//   }
// }

void TOPSimp::SetObsCons() {
  // Set obstacle constraints (just a box in the center)
  std::cout << "Setting obstacle constraints..." << std::endl;
  Eigen::AlignedBox3d obstacle(Eigen::Vector3d(-1.0, 0.5, 0.0),
                               Eigen::Vector3d(1.0, 1.5, 0.0));
  keep_out_zones_.push_back(obstacle);
}

bool TOPSimp::IsInsideObstacle(const Vec3& point) {
  for (const auto& box : keep_out_zones_) {
    if (box.contains(point)) {
      return true;
    }
  }
  return false;
}

void TOPSimp::InitTrajStraightline() {
  for (size_t i = 0; i < N; ++i) {
    Xprev[i] = x0 + (xg - x0) * (static_cast<double>(i) / (N - 1));
  }
}

void TOPSimp::WriteTrajectoryToFile(const Vec13Vec& states, const Vec6Vec& controls, const std::string& filename) {
  std::ofstream traj_file(filename);
  if (!traj_file.is_open()) {
    std::cerr << "Failed to open trajectory file for writing." << std::endl;
    return;
  }
  for (size_t i = 0; i < states.size(); ++i) {
    traj_file << states[i].transpose();
    if (i < controls.size()) traj_file << " " << controls[i].transpose();
    traj_file << std::endl;
  }
  traj_file.close();
}

bool TOPSimp::Solve() {
  bool enforce_init_final_cond = true;
  bool enforce_lin_dynamics = true;
  bool enforce_control_bounds = true;
  bool enforce_obs_const = false;
  SetBoundaryCons();
  SetObsCons();
  InitTrajStraightline();
  WriteTrajectoryToFile(Xprev, Uprev, "init_traj.txt");

  int num_states = 13;
  int num_controls = 6;

  H = Eigen::SparseMatrix<double>(N * num_states + (N - 1) * num_controls, N * num_states + (N - 1) * num_controls);
  // Weights for control effort minimization
  double control_weight = 1.0;
  std::vector<Eigen::Triplet<double>> hessian_triplets;
  // Penalize control inputs (u1, u2, u3)
  for (size_t i = 0; i < (N - 1); ++i) {
    for (size_t j = 0; j < 3; ++j) {
      size_t idx = N * num_states + i * num_controls + j;
      hessian_triplets.emplace_back(idx, idx, control_weight);
    }
  }
  H.setFromTriplets(hessian_triplets.begin(), hessian_triplets.end());


  gradient = Eigen::VectorXd::Zero(N * num_states + (N - 1) * num_controls);

  int init_final_boundary_constraints = 2 * num_states;
  int lin_dynamics_constraints = 2 * 3 * (N - 1);  // (x,y,z) and (vx,vy,vz) for each time step
  int bounded_control_constraints = 3 * (N - 1);   // (u1, u2, u3) for each time step
  int obstacle_constraints = (N - 1) * 3;          // Max 3 (XYZ) constraints per time step

  // Print constraint  counts
  std::cout << "init_final_boundary_constraints: " << init_final_boundary_constraints << std::endl;
  std::cout << "lin_dynamics_constraints: " << lin_dynamics_constraints << std::endl;
  std::cout << "bounded_control_constraints: " << bounded_control_constraints << std::endl;
  std::cout << "obstacle_constraints: " << obstacle_constraints << std::endl;

  // calculate total_constraints depending on which constraints are enabled
  int total_constraints = (enforce_init_final_cond ? init_final_boundary_constraints : 0) +
                          (enforce_lin_dynamics ? lin_dynamics_constraints : 0) +
                          (enforce_control_bounds ? bounded_control_constraints : 0) +
                          (enforce_obs_const ? obstacle_constraints : 0);

  // Print which constraints are enabled and corresponding number of constraints
  std::cout << "enforce_init_final_cond: " << enforce_init_final_cond << " (" << init_final_boundary_constraints
            << " constraints)" << std::endl;
  std::cout << "enforce_lin_dynamics: " << enforce_lin_dynamics << " (" << lin_dynamics_constraints << " constraints)"
            << std::endl;
  std::cout << "enforce_control_bounds: " << enforce_control_bounds << " (" << bounded_control_constraints
            << " constraints)" << std::endl;
  std::cout << "enforce_obs_const: " << enforce_obs_const << " (" << obstacle_constraints << " constraints)"
            << std::endl;
  std::cout << "Total constraints: " << total_constraints << std::endl;


  A = Eigen::SparseMatrix<double>(total_constraints,
                                   N * num_states + (N - 1) * num_controls);
  lower_bound = Eigen::VectorXd::Zero(total_constraints);
  upper_bound = Eigen::VectorXd::Zero(total_constraints);

  int running_total = 0;

  if (enforce_init_final_cond) {
    // Set boundary constraints
    for (int i = 0; i < 13; ++i) {
      // Initial state constraint
      A.insert(i, i) = 1.0;  // Constrain x[0][i] (initial state component)
      lower_bound[i] = x0[i];  // Set lower bound for x[0][i]
      upper_bound[i] = x0[i];  // Set upper bound for x[0][i]

      // Final state constraint
      A.insert(13 + i, (N - 1) * 13 + i) = 1.0;  // Constrain x[N-1][i] (final state component)
      lower_bound[13 + i] = xg[i];  // Set lower bound for x[N-1][i]
      upper_bound[13 + i] = xg[i];  // Set upper bound for x[N-1][i]
    }
    running_total += init_final_boundary_constraints;  // Update running total of constraints
  }

  if (enforce_lin_dynamics) {
    int new_total = AddLinearDynamics(running_total);
    if (new_total != lin_dynamics_constraints) {
      std::cerr << "Error: Expected " << lin_dynamics_constraints << " constraints, but added " << new_total
                << " constraints." << std::endl;
    }
    running_total += lin_dynamics_constraints;  // Update running total of constraints
  }

  if (enforce_control_bounds) {
    int new_total = AddControlLimits(running_total);
    if (new_total != bounded_control_constraints) {
      std::cerr << "Error: Expected " << bounded_control_constraints << " constraints, but added " << new_total
                << " constraints." << std::endl;
    }
    running_total += bounded_control_constraints;  // Update running total of constraints
  }

  if (enforce_obs_const) {
    if (keep_out_zones_.size() != 1) {
      std::cerr << "Error: Only one obstacle is supported for now! Found " << keep_out_zones_.size() << " obstacles."
                << std::endl;
      return false;
    }
    Eigen::AlignedBox3d box = keep_out_zones_[0];
    Eigen::Vector3d ko_min_original = box.min();
    Eigen::Vector3d ko_max_original = box.max();
    Eigen::Vector3d ko_min = ko_min_original;
    Eigen::Vector3d ko_max = ko_max_original;
    Eigen::Vector3d ko_center = (ko_min + ko_max)/2;
    Eigen::Vector3d pos_min_ = x0.head<3>();
    Eigen::Vector3d pos_max_ = xg.head<3>();
    std::cout << "original ko_min: " << ko_min_original.transpose() << std::endl;
    std::cout << "original ko_max: " << ko_max_original.transpose() << std::endl;
    std::cout << "pos_min_: " << pos_min_.transpose() << std::endl;
    std::cout << "pos_max_: " << pos_max_.transpose() << std::endl;
    std::cout << "ko_center: " << ko_center.transpose() << std::endl;
    for (size_t ii = 0; ii < N-1; ii++) {
      for (size_t jj = 0; jj < 3; jj++) {
        decimal_t lb = pos_min_[jj];
        decimal_t ub = pos_max_[jj];
        // lb < x < ub
        // Either ko_max < x < ub or lb < x < ko_min
        bool active_proj = true;
        for (size_t kk = 0; kk < 3; kk++) {
          if (kk == jj) {
            continue;
          }
          // std::cout << "Checking active proj" << std::endl;
          // std::cout << "Checking x y z " << Xprev[ii](0) << ", " << Xprev[ii](1) << ", " << Xprev[ii](2) <<
          // std::endl;
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
        A.insert(running_total, 13*ii + jj) = 1.0;
        lower_bound(running_total) = lb;
        upper_bound(running_total) = ub;
        running_total++;
      }
    }
  }

  // Check that running total matches total number of constraints
  if (running_total != total_constraints) {
    std::cerr << "Error: Running total of constraints does not match total number of constraints!" << std::endl;
    return false;
  }
  // Check that A matrix has the correct number of rows
  if (A.rows() != total_constraints) {
    std::cerr << "Error: A matrix does not have the correct number of rows!" << std::endl;
    return false;
  }
  // Check that lower bound and upper bound vectors have the correct size
  if (lower_bound.size() != total_constraints || upper_bound.size() != total_constraints) {
    std::cerr << "Error: Lower bound and upper bound vectors do not have the correct size!" << std::endl;
    return false;
  }
  // // Check that lower bounds are less than or equal to upper bounds
  // if ((lower_bound.array() > upper_bound.array()).any()) {
  //   std::cerr << "Error: Lower bounds are not less than or equal to upper bounds!" << std::endl;
  //   // print which lower bound is greater than upper bound
  //   // find which lower bound is greater than upper bound
  //   for (size_t i = 0; i < total_constraints; ++i) {
  //     if (lower_bound[i] > upper_bound[i]) {
  //       std::cerr << " num of init final " << init_final_boundary_constraints << std::endl;

  //       std::cerr << "Lower bound: " << lower_bound[i] << " Upper bound: " << upper_bound[i] << std::endl;
  //       size_t temp_i = i;
  //       if ((0 <= temp_i) && (temp_i < init_final_boundary_constraints)) {
  //         std::cerr << "Init final constraint" << i << std::endl;
  //       } else{
  //         temp_i -= init_final_boundary_constraints;
  //         if ((0 <= temp_i) && (temp_i < lin_dynamics_constraints)) {
  //           std::cerr << "Lin dynamics constraint" << i << std::endl;
  //         } else {
  //           temp_i -= lin_dynamics_constraints;
  //           if ((0 <= temp_i) && (temp_i < bounded_control_constraints)) {
  //             std::cerr << "Bounded control constraint" << i << std::endl;
  //           } else {
  //             temp_i -= bounded_control_constraints;
  //             if ((0 <= temp_i) && (temp_i < obstacle_constraints)) {
  //               std::cerr << "Obstacle constraint" << i << std::endl;
  //             }
  //           }
  //         }
  //       }

  //     }
  //   }
  //   return false;
  // }

  // // Add obstacle avoidance constraints for each time step
  // int obstacle_row = init_final_boundary_constraints + lin_dynamics_constraints + bounded_control_constraints;
  // for (size_t i = 0; i < N; ++i) {
  //   Vec3 point = Xprev[i].head<3>();  // Extract the position (first 3 elements)
  //   if (IsInsideObstacle(point)) {
  //     A.insert(obstacle_row, i * 13) = -1.0; // Penalty for being inside the obstacle
  //     lower_bound[obstacle_row] = obs_clearance;  // Minimum clearance required
  //     upper_bound[obstacle_row] = std::numeric_limits<double>::infinity();
  //     ++obstacle_row;
  //   }
  // }

  solver.data()->setNumberOfVariables(N * 13 + (N - 1) * 6);
  solver.data()->setNumberOfConstraints(A.rows());

  solver.data()->setHessianMatrix(H);
  solver.data()->setGradient(gradient);
  solver.data()->setLinearConstraintsMatrix(A);
  solver.data()->setLowerBound(lower_bound);
  solver.data()->setUpperBound(upper_bound);

  solver.settings()->setVerbosity(false);
  solver.initSolver();

  if (solver.solve()) {
    Eigen::VectorXd solution = solver.getSolution();
    for (size_t i = 0; i < N; ++i) {
      Xprev[i] = solution.segment(i * 13, 13);
      if (i < N - 1) {
        Uprev[i] = solution.segment(N * 13 + i * 6, 6);
      }
    }
    WriteTrajectoryToFile(Xprev, Uprev, "trajectory.txt");
    std::cout << "Optimization successful!" << std::endl;
    return true;
  } else {
    std::cerr << "Optimization failed!" << std::endl;
    return false;
  }
}

}  // namespace scp

int main() {
  scp::TOPSimp top(10.0, 50);
  top.x0 << -1, -2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  top.xg << 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  if (!top.Solve()) return -1;
  return 0;
}
