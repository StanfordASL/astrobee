/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BULLET_DYNAMICS_COMMON_H
#define BULLET_DYNAMICS_COMMON_H

///Common headerfile includes for Bullet Dynamics, including Collision Detection
#include "bullet3/src/btBulletCollisionCommon.h"

#include "bullet3/src/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"

#include "bullet3/src/BulletDynamics/Dynamics/btSimpleDynamicsWorld.h"
#include "bullet3/src/BulletDynamics/Dynamics/btRigidBody.h"

#include "bullet3/src/BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"
#include "bullet3/src/BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "bullet3/src/BulletDynamics/ConstraintSolver/btConeTwistConstraint.h"
#include "bullet3/src/BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h"
#include "bullet3/src/BulletDynamics/ConstraintSolver/btSliderConstraint.h"
#include "bullet3/src/BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.h"
#include "bullet3/src/BulletDynamics/ConstraintSolver/btUniversalConstraint.h"
#include "bullet3/src/BulletDynamics/ConstraintSolver/btHinge2Constraint.h"
#include "bullet3/src/BulletDynamics/ConstraintSolver/btGearConstraint.h"
#include "bullet3/src/BulletDynamics/ConstraintSolver/btFixedConstraint.h"

#include "bullet3/src/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"

///Vehicle simulation, with wheel contact simulated by raycasts
#include "bullet3/src/BulletDynamics/Vehicle/btRaycastVehicle.h"

#endif  //BULLET_DYNAMICS_COMMON_H
