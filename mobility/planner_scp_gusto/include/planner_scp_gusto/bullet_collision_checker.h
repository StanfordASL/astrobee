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

#ifndef PLANNER_SCP_GUSTO_BULLET_COLLISION_CHECKER_H_
#define PLANNER_SCP_GUSTO_BULLET_COLLISION_CHECKER_H_

#include <boost/math/constants/constants.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>
#include <string>

#include "planner_scp_gusto/types.h"

#include "bullet/btBulletDynamicsCommon.h"
#include "bullet/btBulletCollisionCommon.h"
#include "bullet/BulletCollision/CollisionShapes/btConvex2dShape.h"
#include "bullet/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "bullet/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include "bullet/BulletCollision/NarrowPhaseCollision/btPointCollector.h"
#include "bullet/BulletDynamics/Featherstone/btMultiBody.h"
#include "bullet/LinearMath/btConvexHull.h"

namespace scp {

struct SignedDistanceResult {
  scp::decimal_t sd;
  scp::Vec3 co1_pt;
  scp::Vec3 co2_pt;
};

class BulletCollisionChecker {
  // btCollisionWorld* cw;
  // btBroadphaseInterface* broadphase;
  // btCollisionDispatcher* dispatcher;
  // btCollisionConfiguration* collisionConfiguration;

  btCollisionWorld* m_world;
  btBroadphaseInterface* m_broadphase;
  btCollisionDispatcher* m_dispatcher;
  btCollisionConfiguration* m_coll_config;
  btCollisionObject* robot;

  std::vector<btCollisionObject*> convex_robot_components;
  std::vector<btCollisionObject*> convex_env_components;

  double m_contactDistance;

 public:
    BulletCollisionChecker();
    ~BulletCollisionChecker();

    unsigned int max_objects = 100;

    void SetContactDistance(float distance);
    double GetContactDistance() {return m_contactDistance;}

    void ComputeDistance(size_t env_idx, SignedDistanceResult& out);

    void SetTransformation(btCollisionObject* o, scp::Vec3 v, scp::Quat q);
    void SetTransformation(btCollisionObject* o, scp::Vec3 v);

    std::vector<btCollisionObject*> GetConvexComponents(btCollisionShape* cs);
    std::vector<btCollisionObject*> GetConvexComponents(btCollisionObject* co);
    std::vector<btCollisionObject*> GetConvexComponents(btCollisionShape* cs, btTransform tr);

    void AddRobot(scp::Vec3 a = scp::Vec3{0.3, 0.3, 0.3},
                  scp::Vec3 r = scp::Vec3{0.0, 0.0, 0.0},
                  scp::Quat q = scp::Quat{0.0, 0.0, 0.0, 1.0});  // add a robot obstacle (ellipsoid)
    void AddObstacle(scp::Vec3 a, scp::Vec3 r, scp::Quat q);  // add arbitrary obstacle (ellipsoid)

    bool IsFreeState(scp::Vec3 v, scp::Quat q);
    bool IsFreeMotion(scp::Vec3 v, scp::Vec3 w);
    bool IsFreeMotion(scp::Vec3 v, scp::Quat vrot, scp::Vec3 w, scp::Quat wrot);

    void SetMargin(btCollisionObject* obj, scp::decimal_t dx);
    void SetMargin(btCollisionShape* cs, scp::decimal_t dx);

 private:
    void ComputeDistance(const btCollisionShape* cs1, const btTransform& tr1,
        const btCollisionShape* cs2, const btTransform& tr2,
        btScalar* result, btScalar max_d2);
};
}  // namespace scp
#endif  // PLANNER_SCP_GUSTO_BULLET_COLLISION_CHECKER_H_
