# scp

SCP-based traj opt includes a collision checking wrapper around Bullet3. The interface for this wrapper for ellipsoidal collision-checking
can be found in `test/test-bullet.cpp` (`./test-bullet`).

## Usage (collision-checking)

1. Create collision-checking module:

`collision_checker::BulletCollisionChecker bullet_ = collision_checker::BulletCollisionChecker();  // collision-checking module`

2. Add desired (ellipsoidal) obstacles:

`bullet_.AddObstacle(eig_obs.head(3), eig_obs.tail(3), Eigen::Quaternion<double>{0.0, 0.0, 0.0, 1.0});`

3. Add desired robot geometry:

(Created by default using `AddRobot()` method)

4. Test for collision at various robot poses:

`bullet_.IsFreeState(scp::Vec3{pt(0), pt(1), pt(2)}, Eigen::Quaternion<double>{0.0, 0.0, 0.0, 1.0})`

