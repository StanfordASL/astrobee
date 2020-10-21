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

#include <sampling_planner/mpproblem.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <fcntl.h>
#include <math.h>
#include <planner_fmt_protobuf.pb.h>
#include <boost/math/constants/constants.hpp>
#include <boost/math/special_functions/factorials.hpp>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/message_lite.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
// #include "gurobi_c++.h"

#include <Eigen/SparseCore>
#include <Eigen/Dense>
#include <iostream>
#include <algorithm>
#include <set>
#include <utility>
#include <limits>
#include <string>
#include <iterator>
#include <cassert>
#include <fstream>
#include <sstream>
#include <vector>

namespace motionplanner {

motionplanner::MPSolution::MPSolution() {
}


motionplanner::MPSolution::~MPSolution() {
}


motionplanner::MPProblem::MPProblem() {
  mpsolution = new MPSolution();
  mpsolution->sbmp_solved =  false;
  mpsolution->opt_solved =  false;
  mpsolution->numSamples = 0;
  mpsolution->totalCost = std::numeric_limits<decimal_t>::infinity();

  n = 4;
  m = 2;

  dt = 1.0;
  Tf = 30.0;
  N = static_cast<int>(ceil(Tf/dt))+1;
  goalThreshold = 0.05;
}


motionplanner::MPProblem::~MPProblem() {
  if (mpsolution) delete mpsolution;
}


bool motionplanner::MPProblem::ReadSampleData(const std::string protobuf_file) {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  planner_fmt_protobuf::Solution_Data solutions;

  std::fstream textInput(protobuf_file, std::ios::in | std::ios::binary);
  if (!solutions.ParseFromIstream(&textInput)) {
    // ROS_ERROR_STREAM("Failed to parse from istream!");
    return false;
  }

  numSamples = solutions.numsamples();
  xDim = solutions.xdim();
  samples = MatD(xDim, numSamples);

  const planner_fmt_protobuf::Samples solutions_samples = solutions.samples();
  for (int i = 0; i < numSamples; i++) {
    for (int j = 0; j < xDim; j++) {
      samples(j, i) = solutions_samples.data(xDim*i + j);
    }
  }

  adjCosts.resize(numSamples * numSamples);
  for (int i = 0; i < numSamples*numSamples; i++) {
    adjCosts[i] = solutions.adjcosts(i);
  }

  nnGoEdges.resize(numSamples * numSamples);
  nnComeEdges.resize(numSamples * numSamples);
  for (int i = 0; i < numSamples*numSamples; i++) {
    nnGoEdges[i] = solutions.nngoedges(i);
    nnComeEdges[i] = solutions.nncomeedges(i);
  }

  google::protobuf::ShutdownProtobufLibrary();
  return true;
}


bool motionplanner::MPProblem::InGoalRegion(int node, int goal_node) {
  VecD cur = VecD(xDim/2);
  VecD goal = VecD(xDim/2);
  for (int i = 0; i < xDim/2; i++) {
    cur(i) = samples(i, node);
    goal(i) = samples(i, goal_node);
  }

  VecD diff = goal - cur;
  return diff.norm() < goalThreshold ? true : false;
}

void motionplanner::MPProblem::FMT(int initial_idx, int final_idx) {
  std::vector<bool> closedNodes(numSamples, false);  // visited nodes
  std::vector<bool> openNodes(numSamples, false);    // open nodes
  std::vector<bool> unvisitedNodes(numSamples, true);        // unconnected nodes
  std::vector<int> A(numSamples, -1);           // array of parent nodes
  std::vector<decimal_t> costs(numSamples, std::numeric_limits<decimal_t>::max());
  std::set<std::pair<decimal_t, int>> openSet;  // cost and idx
  std::set<std::pair<decimal_t, int>>::iterator it;

  costs[initial_idx] = 0;
  openNodes[initial_idx] = true;
  unvisitedNodes[initial_idx] = false;

  std::pair<decimal_t, int> nodePair(0, initial_idx);
  openSet.insert(nodePair);

  it = openSet.begin();   // retrieves iterator object with lowest cost
  int z = it->second;   // index of lowest cost node
  openSet.erase(it);

  while (!InGoalRegion(z, final_idx)) {
  // for (int fmt_idx = 0; fmt_idx < 10000; fmt_idx++) {
    if (z < 0) break;   // none left to closed, i.e. failure
    if (z == final_idx) break;  // soln found

    // iterate through forward reachable set from z
    for (int j = 0; j < numSamples; ++j) {  // select node to expand to
      int nextIdx = nnGoEdges[z*numSamples + j];

      if (nextIdx < 0 || nextIdx > numSamples) break;  // out of neighbors or overreach bug
      if (closedNodes[nextIdx]) continue;       // this node has been closed
      if (!unvisitedNodes[nextIdx]) continue;   // this node has been set

      // identify backwards reachable set
      int oneStepIdx = -1;
      decimal_t backwardCost = std::numeric_limits<decimal_t>::max();

      // iterate through backward reachable set from x
      for (int k = 0; k < numSamples; ++k) {
        int prevIdx = nnComeEdges[nextIdx*numSamples + k];
        if (prevIdx < 0 || prevIdx > numSamples) break;  // out of neighbors or overreach bug

        if (!openNodes[prevIdx]) continue;  // node to connect to has been closed or unvisited

        // identify closest backwards reachable node
        decimal_t pathCost = costs[prevIdx] + adjCosts[prevIdx*numSamples + nextIdx];
        if (pathCost < backwardCost) {
          backwardCost = pathCost;
          oneStepIdx = prevIdx;
        }
      }

      if (oneStepIdx == -1) continue;  // nothing to connect to

      // check edge connection for collision

      if (IsFreeEdge(oneStepIdx, nextIdx)) {
        // Re-wire tree
        openNodes[nextIdx] = true;
        unvisitedNodes[nextIdx] = false;
        std::pair<decimal_t, int> nodePairNew(backwardCost, nextIdx);
        openSet.insert(nodePairNew);

        // costs[nextIdx] = pathCost;
        costs[nextIdx] = backwardCost;
        A[nextIdx] = oneStepIdx;
      }
    }
    closedNodes[z] = true;  // place in closed set
    openNodes[z] = false;   // remove from open set

    if (openSet.empty()) {
      // open set is empty, i.e. failure
      break;
    } else  {
      // find min node to expand from
      it = openSet.begin();   // retrieves iterator object with lowest cost
      z = it->second;         // index of lowest cost node
      if (z < 0) break;       // none left to closed, i.e. failure
      openSet.erase(it);
      // decimal_t curCost = it->first;  // cost of lowest cost node
    }
  }

  // if (InGoalRegion(z, final_idx)) {
  if (!unvisitedNodes[final_idx]) {
    mpsolution->sbmp_solved = true;

    int cur_idx = final_idx;
    std::vector<decimal_t> soln_costs;
    while (cur_idx != -1) {
      mpsolution->solution.push_back(cur_idx);
      soln_costs.push_back(costs[A[cur_idx]*numSamples + cur_idx]);
      cur_idx = A[cur_idx];
    }
    std::reverse(std::begin(mpsolution->solution), std::end(mpsolution->solution));
  }
}


void motionplanner::MPProblem::solve(Vec4 start, Vec4 finish,
  std::vector<Eigen::AlignedBox3d>* free_zones_, std::vector<Eigen::AlignedBox3d>* obstacle_zones_) {
  initial = start;
  goal = finish;
  keep_in_zones_ = free_zones_;
  keep_out_zones_ = obstacle_zones_;
  mpsolution->initial = initial;
  mpsolution->goal = goal;
  mpsolution->numSamples = numSamples;

  // Determine initial and final nodes
  int init_idx = -1, goal_idx = -1;
  decimal_t min_init_distance = std::numeric_limits<decimal_t>::max();
  decimal_t min_goal_distance = std::numeric_limits<decimal_t>::max();

  for (int k = 0; k < numSamples; k++) {
    Vec2 diff = samples.col(k) - start.block(0, 0, n/2, 1);
    if (diff.norm() < min_init_distance) {
      min_init_distance = diff.norm();
      init_idx = k;
    }

    diff = samples.col(k) - finish.block(0, 0, n/2, 1);
    if (diff.norm() < min_goal_distance) {
      min_goal_distance = diff.norm();
      goal_idx = k;
    }
  }

  FMT(init_idx, goal_idx);
  if (!mpsolution->sbmp_solved) {
    // ROS_ERROR_STREAM("FMT failed to find a solution!");
    return;
  }

  int nWPs = mpsolution->solution.size();
  int nEdges = nWPs-1;

  // Dynamics update matrices
  MatD Ak(n, n);
  Ak.topLeftCorner(n/2, n/2) = MatD::Identity(n/2, n/2);
  Ak.topRightCorner(n/2, n/2) = dt*MatD::Identity(n/2, n/2);
  Ak.bottomLeftCorner(n/2, n/2) = MatD::Zero(n/2, n/2);
  Ak.bottomRightCorner(n/2, n/2) = MatD::Identity(n/2, n/2);

  MatD Bk(n, m);
  Bk.block(0, 0, n/2, n/2) = MatD::Zero(n/2, n/2);
  Bk.block(0, 0, n/2, m) = 0.5*dt*dt*MatD::Identity(m, m);
  Bk.block(n/2, 0, n/2, m) = dt*MatD::Identity(m, m);

  // Storage for solutions
  MatD Xtpbvp(n, N), Utpbvp(m, N-1);
  GRBEnv* env = 0;
  env = new GRBEnv();

  // Reach first node in graph
  Xsoln.resize(n, N);
  Usoln.resize(m, N-1);
  VecD Xi(n), Xf(n);
  Xi << start;
  Xf << samples.col(mpsolution->solution[0]);

  bool success = tpbvp(env, Ak, Bk, Xi, Xf, Xsoln, Usoln);
  if (!success) {
    // ROS_ERROR_STREAM("Could not connect to first node in FMT path!");
    return;
  }

  bool tpbvp_solved = success;
  for (int i = 0; i < nEdges; i++) {
    // BCs
    Xi << samples.col(mpsolution->solution[i]);
    Xf << samples.col(mpsolution->solution[i+1]);

    try {
      success = tpbvp(env, Ak, Bk, Xi, Xf, Xtpbvp, Utpbvp);

      // if a single tpbvp fails, optimization fails
      if (tpbvp_solved) {
        tpbvp_solved = success;
      }

      if (!success) {
        continue;
      }

      int oldWidth = Xsoln.cols();

      // TODO(acauligi): implement efficiently
      MatD newMatrix(n, oldWidth+N-1);
      newMatrix << Xsoln, Xtpbvp.block(0, 1, xDim, N-1);
      Xsoln.resize(n, oldWidth+N-1);
      Xsoln = newMatrix;
    } catch(GRBException e) {
      // ROS_ERROR_STREAM("Error code = " + std::to_string(e.getErrorCode()));
      // ROS_ERROR_STREAM("Error message " + e.getMessage());
    } catch(...) {
      // ROS_ERROR_STREAM("Exception during optimization!");
    }
  }

  Xi << samples.col(mpsolution->solution[nWPs-1]);
  Xf << finish;
  success = tpbvp(env, Ak, Bk, Xi, Xf, Xsoln, Usoln);
  if (success) {
    int oldWidth = Xsoln.cols();

    // TODO(acauligi): implement efficiently
    MatD newMatrix(n, oldWidth+N-1);
    newMatrix << Xsoln, Xtpbvp.block(0, 1, xDim, N-1);
    Xsoln.resize(n, oldWidth+N-1);
    Xsoln = newMatrix;
  } else {
    // ROS_ERROR_STREAM("Could not connect goal from last node in FMT path!");
  }

  delete env;
  mpsolution->opt_solved = tpbvp_solved;
  return;
}

bool motionplanner::MPProblem::IsFreeEdge(int node1, int node2) {
  // TODO(acauligi): collision checker --> default return true for now
  return true;
}

bool motionplanner::MPProblem::tpbvp(GRBEnv* env,
  MatD& Ak, MatD& Bk,
  VecD& Xi, VecD& Xf,
  MatD& Xsoln, MatD& Usoln) {
  bool success = false;
  GRBModel model = GRBModel(*env);

  // /* Add variables to the model */
  GRBVar **X = NULL;
  X = new GRBVar*[N];
  for (int i = 0; i < N; i++) {
    X[i] = new GRBVar[n];
    for (int j = 0; j < n; j++) {
      X[i][j] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0,
        GRB_CONTINUOUS, "x_"+std::to_string(i)+"_"+std::to_string(j));
    }
  }

  GRBVar **U = NULL;
  U = new GRBVar*[N-1];
  for (int i = 0; i < N-1; i++) {
    U[i] = new GRBVar[m];
    for (int j = 0; j < m; j++) {
      U[i][j] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0,
        GRB_CONTINUOUS, "u_"+std::to_string(i)+"_"+std::to_string(j));
    }
  }
  model.update();

  /* Dynamics constraints */
  for (int i = 0; i < N-1; i++) {
    for (int j = 0; j < n; j++) {
      GRBLinExpr lhs = 0;
      for (int k = 0; k < n; k++) {
        lhs += Ak(j, k)*X[i][k];
      }
      for (int k = 0; k < m; k++) {
        lhs += Bk(j, k)*U[i][k];
      }
      lhs -= X[i+1][j];
      model.addConstr(lhs, '=', 0.0);
    }
  }

  /* Boundary conditions */
  for (int i = 0; i < n; i++) {
    GRBLinExpr leftBC = Xi(i) - X[0][i];
    model.addConstr(leftBC, '=', 0.0);
    GRBLinExpr rightBC = Xf(i) - X[N-1][i];
    model.addConstr(rightBC, '=', 0.0);
  }

  /* Speed constraints */
  for (int i = 0; i < N; i++) {
    GRBQuadExpr vel_norm = 0;
    for (int j = n/2; j < n; j++) {
      vel_norm += X[i][j]*X[i][j];
    }
    model.addQConstr(vel_norm, '<', pow(desired_vel_, 2), "vel_norm_"+std::to_string(i));
  }

  /* Actuator constraints */
  for (int i = 0; i < N-1; i++) {
    GRBQuadExpr accel_norm = 0;
    for (int j = 0; j < m; j++) {
      accel_norm += U[i][j]*U[i][j];
    }
    model.addQConstr(accel_norm, '<', pow(desired_accel_, 2), "accel_norm_"+std::to_string(i));
  }

  /* Minimum energy cost */
  GRBQuadExpr J = 0;
  for (int i = 0; i < N-1; i++) {
    for (int j = 0; j < m; j++) {
      J += U[i][j]*U[i][j];
    }
  }

  model.setObjective(J);

  model.optimize();

  model.write("opt.lp");

  if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
    success = true;

    // Unpack solutions into matrices
    for (int i = 0; i < N; i++) {
      for (int j = 0; j < n; j++) {
        Xsoln(j, i) = X[i][j].get(GRB_DoubleAttr_X);
      }
    }
    for (int i = 0; i < N-1; i++) {
      for (int j = 0; j < m; j++) {
        Usoln(j, i) = U[i][j].get(GRB_DoubleAttr_X);
      }
    }
  }

  for (int i = 0; i < N; i++) delete X[i];
  delete X;
  for (int i = 0; i < N-1; i++) delete U[i];
  delete U;

  return success;
}

}  // namespace motionplanner
