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

#ifndef SCP_TYPES_H_
#define SCP_TYPES_H_

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <vector>

// This file is to define typdefs for all the eigen types as either float or
// double
// Add flag "-DTRAJ_OPT_USE_SINGLE_PRECISION" to compile the entire package with
// float,
// otherwise, we will default to use double

namespace scp {

#ifndef SCP_USE_SINGLE_PRECISION
typedef double decimal_t;
#else
typedef double decimal_t;
#endif

typedef Eigen::Matrix<decimal_t, 2, 1> Vec2;
typedef Eigen::Matrix<decimal_t, 3, 1> Vec3;
typedef Eigen::Matrix<decimal_t, 4, 1> Vec4;
typedef Eigen::Matrix<decimal_t, 6, 1> Vec6;
typedef Eigen::Matrix<decimal_t, 7, 1> Vec7;
typedef Eigen::Matrix<decimal_t, 13, 1> Vec13;

typedef Eigen::Matrix<decimal_t, 3, 3> Mat3;
typedef Eigen::Matrix<decimal_t, 4, 4> Mat4;
typedef Eigen::Matrix<decimal_t, 6, 6> Mat6;
typedef Eigen::Matrix<decimal_t, 7, 7> Mat7;
typedef Eigen::Matrix<decimal_t, 13, 13> Mat13;
typedef Eigen::Matrix<decimal_t, 4, 3> Mat4x3;
typedef Eigen::Matrix<decimal_t, 6, 3> Mat6x3;
typedef Eigen::Matrix<decimal_t, 7, 3> Mat7x3;
typedef Eigen::Matrix<decimal_t, 13, 6> Mat13x6;

typedef Eigen::DiagonalMatrix<decimal_t, 6> DiagMat6;
typedef Eigen::DiagonalMatrix<decimal_t, 13> DiagMat13;

typedef std::vector<Vec3, Eigen::aligned_allocator<Vec3>> Vec3Vec;
typedef std::vector<Vec6, Eigen::aligned_allocator<Vec6>> Vec6Vec;
typedef std::vector<Vec7, Eigen::aligned_allocator<Vec7>> Vec7Vec;
typedef std::vector<Vec13, Eigen::aligned_allocator<Vec13>> Vec13Vec;
typedef std::vector<Mat7, Eigen::aligned_allocator<Mat7>> Mat7Vec;
typedef std::vector<Mat13, Eigen::aligned_allocator<Mat13>> Mat13Vec;
typedef std::vector<Mat7x3, Eigen::aligned_allocator<Mat7x3>> Mat7x3Vec;
typedef std::vector<Mat13x6, Eigen::aligned_allocator<Mat13x6>> Mat13x6Vec;

typedef Eigen::Matrix<decimal_t, Eigen::Dynamic, 1> VecD;
typedef Eigen::Matrix<decimal_t, Eigen::Dynamic, 3> MatD3;
typedef Eigen::Matrix<decimal_t, Eigen::Dynamic, Eigen::Dynamic> MatD;
typedef Eigen::SparseMatrix<decimal_t> SparseMatD;
// typedef Eigen::SparseMatrix<decimal_t, Eigen::Dynamic, Eigen::Dynamic> SparseMatD;

typedef Eigen::Quaternion<decimal_t> Quat;

typedef std::vector<VecD> VecDVec;
typedef std::vector<MatD> MatDVec;

}  //  namespace scp
#endif  // SCP_TYPES_H_
