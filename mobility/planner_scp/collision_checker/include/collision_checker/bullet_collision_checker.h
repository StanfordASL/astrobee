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
#ifndef COLLISION_CHECKER_BULLET_COLLISION_CHECKER_H_
#define COLLISION_CHECKER_BULLET_COLLISION_CHECKER_H_

#include <btBulletDynamicsCommon.h>
#include <boost/math/constants/constants.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <collision_checker/types.h>

#include <vector>
#include <string>

#include "btBulletCollisionCommon.h"
#include "BulletCollision/CollisionShapes/btConvex2dShape.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h"
#include "BulletCollision/NarrowPhaseCollision/btPointCollector.h"
#include "BulletDynamics/Featherstone/btMultiBody.h"
#include "LinearMath/btConvexHull.h"



namespace collision_checker {

class BulletCollisionChecker {
  // btCollisionWorld* cw;
  // btBroadphaseInterface* broadphase;
  // btCollisionDispatcher* dispatcher;
  // btCollisionConfiguration* collisionConfiguration;

  btCollisionWorld* m_world;
  btBroadphaseInterface* m_broadphase;
  btCollisionDispatcher* m_dispatcher;
  btCollisionConfiguration* m_coll_config;

  std::vector<btCollisionObject*> convex_robot_components;
  std::vector<btCollisionObject*> convex_env_components;

  double m_contactDistance;

 public:
    BulletCollisionChecker();
    ~BulletCollisionChecker();

    unsigned int max_objects = 100;

    void SetContactDistance(float distance);
    double GetContactDistance() {return m_contactDistance;}

    void set_transformation(btCollisionObject* o, Vec3 v, Quat q);
    void set_transformation(btCollisionObject* o, Vec3 v);

    std::vector<btCollisionObject*> get_convex_components(btCollisionShape* cs);
    std::vector<btCollisionObject*> get_convex_components(btCollisionObject* co);
    std::vector<btCollisionObject*> get_convex_components(btCollisionShape* cs, btTransform tr);

    bool is_free_state(Vec3 v, Quat q);
    bool is_free_motion(Vec3 v, Vec3 w);
    bool is_free_motion(Vec3 v, Quat vrot, Vec3 w, Quat wrot);

    void set_margin(btCollisionObject* obj, decimal_t dx);
    void set_margin(btCollisionShape* cs, decimal_t dx);
};
}  // namespace collision_checker
#endif  // COLLISION_CHECKER_BULLET_COLLISION_CHECKER_H_
