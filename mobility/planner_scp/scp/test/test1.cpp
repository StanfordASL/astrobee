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

#include <Eigen/Dense>

#include <iostream>
#include <chrono>

#include "OsqpEigen/OsqpEigen.h"

#include "scp/traj_opt.h"
#include "scp/bullet_collision_checker.h"

int main(void) {
  scp::TOP top = scp::TOP(20., 201);
  top.x0 << 0.35, -0.5, 0., 0, 0, 0, 0.5, 0.5, 0.5, 0.5, 0, 0, 0;
  top.xg << 0.35, -0.5, 0., 0, 0, 0, 0.5, 0.5, 0.5, 0.5, 0, 0, 0;
  // top.xg(0) = 0.55;
  // top.xg(1) = -0.35;

  std::cout << "The num vars is " << top.GetNumTOPVariables() << "\n";
  std::cout << "The num constraints is " << top.GetNumTOPConstraints() << "\n";

  // for (size_t ii = 0; ii < top.N; ii++) {
  //   for (size_t jj = 0; jj < top.state_dim; jj++) {
  //     std::cout << top.Xprev[ii](jj) << " ";
  //   }
  //   std::cout << "\n";
  // }
  // std::cout << "\n";

  for (size_t ii = 0; ii < top.state_dim; ii++) {
    // std::cout << top.fs[0](ii) << " ";
    // std::cout << "\n";
  }
  // std::cout << "\n";

  // for (size_t ii = 0; ii < top.state_dim; ii++) {
  //   for (size_t jj = 0; jj < top.state_dim; jj++) {
  //     // std::cout << top.As[0](ii,jj) << " ";
  //   }
  //   // std::cout << "\n";
  // }
  // std::cout << "\n";

  for (size_t ii = 0; ii < top.state_dim; ii++) {
    for (size_t jj = 0; jj < top.control_dim; jj++) {
      // std::cout << top.Bs[0](ii,jj) << " ";
    }
    // std::cout << "\n";
  }

  // for (size_t ii = 0; ii < 2*top.state_dim; ii++) {
  //   std::cout << top.upper_bound(ii) << std::endl;
  // }

  // std::cout << "Control max: \n";
  // for (size_t ii = 0; ii < top.control_dim; ii++) {
  //   std::cout << top.u_min(ii) << " ";
  // }
  // std::cout << "\n";

  // int row_idx = 2*top.state_dim;
  // for (size_t ii = 0; ii < top.control_dim*(top.N-1); ii++) {
  //   std::cout << top.lower_bound(row_idx+ii) << std::endl;
  // }

  // int row_idx = 2*top.state_dim + top.control_dim*(top.N-1) + 2*top.state_dim*(top.N-1);
  // for (size_t ii = 0; ii < row_idx; ii++) {
  //   std::cout << top.upper_bound(ii) << std::endl;
  // }

  /*
  int row_idx = 0;
  for (size_t ii = 0; ii < 2; ii++) {
    for (size_t jj = 0; jj < top.state_dim; jj++) {
      std::cout << top.upper_bound(row_idx+jj) << " ";
    }
    std::cout << std::endl;
    row_idx += top.state_dim;
  }
  std::cout << std::endl;

  for (size_t ii = 0; ii < top.N-1; ii++) {
    for (size_t jj = 0; jj < top.control_dim; jj++) {
      std::cout << top.upper_bound(row_idx+jj) << " ";
    }
    std::cout << std::endl;
    row_idx += top.control_dim;
  }
  std::cout << std::endl;

  for (size_t ii = 0; ii < top.N-1; ii++) {
    for (size_t jj = 0; jj < top.state_dim; jj++) {
      std::cout << top.upper_bound(row_idx+jj) << " ";
    }
    std::cout << std::endl;
    row_idx += top.state_dim;
    for (size_t jj = 0; jj < top.state_dim; jj++) {
      std::cout << top.upper_bound(row_idx+jj) << " ";
    }
    std::cout << std::endl;
    row_idx += top.state_dim;
  }
  std::cout << std::endl;

  for (size_t ii = 0; ii < top.N-1; ii++) {
    for (size_t jj = 0; jj < top.state_dim; jj++) {
      std::cout << top.upper_bound(row_idx+jj) << " ";
    }
    std::cout << std::endl;
    row_idx += top.state_dim;
    for (size_t jj = 0; jj < top.state_dim; jj++) {
      std::cout << top.upper_bound(row_idx+jj) << " ";
    }
    std::cout << std::endl;
    row_idx += top.state_dim;
  }
  std::cout << std::endl;

  int row_idx = 0;
  MatD dense_mat;
  dense_mat = MatD(top.linear_con_mat);

  for (size_t ii = 0; ii < top.state_dim; ii++) {
    for (size_t jj = 0; jj < top.state_dim; jj++) {
      std::cout << dense_mat(row_idx,jj) << " ";
    }
    std::cout << std::endl;
    row_idx++;
  }
  std::cout << std::endl;

  for (size_t ii = 0; ii < top.state_dim; ii++) {
    for (size_t jj = 0; jj < top.state_dim; jj++) {
      std::cout << dense_mat(row_idx,top.state_dim*(top.N-1)+jj) << " ";
    }
    std::cout << std::endl;
    row_idx++;
  }
  */

  auto start = std::chrono::high_resolution_clock::now();
  if (!top.Solve()) {
    std::cout << "Solver failed!" << std::endl;
  } else {
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);
    std::cout << "Solver worked in " << duration.count()/1e6 << "s!" << std::endl;

    /*
    for (size_t jj = 0; jj < top.N; jj++) {
      for (size_t ii = 0; ii < top.state_dim; ii++) {
        std::cout << top.Xprev[jj](ii) << " ";
      }
      std::cout << std::endl;
    }

    for (size_t jj = 0; jj < top.N-1; jj++) {
      for (size_t ii = 0; ii < top.control_dim; ii++) {
        std::cout << top.Uprev[jj](ii) << " ";
      }
      std::cout << std::endl;
    }
    */
  }

  /*
  for (size_t ii = 0; ii < top.N; ii++) {
    for (size_t jj = 0; jj < top.state_dim; jj++) {
      std::cout << top.Xprev[ii](jj) << " ";
    } 
    std::cout << std::endl;
  }
  std::cout << std::endl;

  Mat13 eye;
  eye.setIdentity();
  for (size_t ii = 0; ii < top.N-1; ii++) {
    Mat13 Ak;
    Mat13x6 Bk;
    Ak = top.dh*top.As[ii] + eye;
    Bk = top.dh*top.Bs[ii];
    for (size_t jj = 0; jj < top.state_dim; jj++) {
      std::cout << top.fs[ii](jj) << " ";
      std::cout << "\t\t\t\t\t";
      for (size_t kk = 0; kk < top.state_dim; kk++) {
        std::cout << top.As[ii](jj,kk) << " ";
      }
      std::cout << "\t\t\t\t\t";
      for (size_t kk = 0; kk < top.control_dim; kk++) {
        std::cout << top.Bs[ii](jj,kk) << " ";
      }

      std::cout << std::endl;
    }
    std::cout << std::endl;
  }
  */

/*
  start = std::chrono::high_resolution_clock::now();
  if (true) {
    top.UpdateProblemDimension(500);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);
    std::cout << "Updated in " << duration.count()/1e6 << "s!" << std::endl;
  }
  
  start = std::chrono::high_resolution_clock::now();
  if (!top.Solve()) {
    std::cout << "Updated and failed to solve!\n";
  } else {
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);
    std::cout << "Updated and solved in " << duration.count()/1e6 << "s!" << std::endl;
  }
  */

  return 0;
}
