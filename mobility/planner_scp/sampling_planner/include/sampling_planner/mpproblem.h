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
#ifndef SAMPLING_PLANNER_MPPROBLEM_H_
#define SAMPLING_PLANNER_MPPROBLEM_H_

#include <sampling_planner/types.h>

#include <boost/math/constants/constants.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gurobi_c++.h>

#include <vector>
#include <string>

namespace motionplanner {

class MPSolution {
 public:
    Vec4 initial;
    Vec4 goal;
    int numSamples;
    std::vector<decimal_t> costs;
    std::vector<int> solution;
    double totalCost;
    bool sbmp_solved;
    bool opt_solved;

    MPSolution();

    ~MPSolution();
};

class MPProblem {
 public:
    Vec4 initial;
    Vec4 goal;
    std::vector<Eigen::AlignedBox3d>* keep_in_zones_;
    std::vector<Eigen::AlignedBox3d>* keep_out_zones_;

    int numSamples;
    int xDim;
    MatD samples;
    std::vector<decimal_t> costs;
    std::vector<decimal_t> adjCosts;
    std::vector<int> nnComeEdges;
    std::vector<int> nnGoEdges;
    decimal_t goalThreshold;
    unsigned int max_objects = 50;  // bullet configuration
    MatD Xsoln;
    MatD Usoln;

    bool faceforward_;          // Face-forward trajectory?
    bool check_obstacles_;      // Perform obstacle checking
    decimal_t desired_vel_;     // Soft limit on velocity
    decimal_t desired_accel_;   // Soft limit on accel
    decimal_t desired_omega_;   // Soft limit on omega
    decimal_t desired_alpha_;   // Soft limit on alpha

    int n;
    int m;
    decimal_t dt;
    decimal_t Tf;
    int N;

    MPSolution* mpsolution;

    MPProblem();

    ~MPProblem();

    bool ReadSampleData(const std::string fn);

    bool IsFreeEdge(int node1, int node2);

    bool InGoalRegion(int node, int goal_node);

    void FMT(int initial_idx, int final_idx);

    bool tpbvp(GRBEnv* env, MatD& Ak, MatD& Bk,
        VecD& Xi, VecD& Xf, MatD& Xsoln, MatD& Usoln);

    void solve(Vec4 start, Vec4 goal,
      std::vector<Eigen::AlignedBox3d>* keep_in_zones, std::vector<Eigen::AlignedBox3d>* keep_out_zones);
};
}  // namespace motionplanner
#endif  // SAMPLING_PLANNER_MPPROBLEM_H_
