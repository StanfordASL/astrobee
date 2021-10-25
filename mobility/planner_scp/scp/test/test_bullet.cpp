#include <eigen3/Eigen/Dense>
#include <scp/bullet_collision_checker.h>
#include <scp/types.h>
#include <memory>
#include <vector>
#include <iostream>

int main() {
  // (1) Bullet collision checking unit test
  collision_checker::BulletCollisionChecker bullet_ = collision_checker::BulletCollisionChecker();  // collision-checking module

  Eigen::Matrix<double, 6, 1> eig_obs;
  eig_obs << 1.0, 1.0, 1.0, 10.0, 10.0, 10.0;

  bullet_.AddObstacle(eig_obs.head(3), eig_obs.tail(3), Eigen::Quaternion<double>{0.0, 0.0, 0.0, 1.0});
  // TODO: good to test with multiple obstacles, don't have this set up for this eigen obstacle version
  // obstacles.push_back(Obstacle(0.05, 0.8, 0.1, 0.3, -0.35, 0.0));

  // for (auto obs : obstacles) {
  //   bullet_.AddObstacle(obs.head(3), obs.tail(3), Eigen::Quaternion<double>{0.0, 0.0, 0.0, 1.0});
  //   std::cout << "Obstacle: " << obs << std::endl;
  // }

  // Check the robot at various points
  scp::Vec3 pt{10.0, 10.0, 10.0};
  std::cout << "setup: " << pt(0) << " " << pt(1) << " " << pt(2) << std::endl;
  // Obstacle checking (3D) ellipsoid-on-ellipsoid
  if (!bullet_.IsFreeState(scp::Vec3{pt(0), pt(1), pt(2)}, Eigen::Quaternion<double>{0.0, 0.0, 0.0, 1.0}) ) {
    std::cout << "collision!" << std::endl;
  }

  // z
  pt = scp::Vec3{9.5, 9.5, 9.5};
  std::cout << "setup: " << pt(0) << " " << pt(1) << " " << pt(2) << std::endl;
  // Obstacle checking (3D) ellipsoid-on-ellipsoid
  if (!bullet_.IsFreeState(scp::Vec3{pt(0), pt(1), pt(2)}, Eigen::Quaternion<double>{0.0, 0.0, 0.0, 1.0}) ) {
    std::cout << "collision!" << std::endl;
  }

  pt = scp::Vec3{0.0, 0.0, 0.14};
  std::cout << "setup: " << pt(0) << " " << pt(1) << " " << pt(2) << std::endl;
  // Obstacle checking (3D) ellipsoid-on-ellipsoid
  if (!bullet_.IsFreeState(scp::Vec3{pt(0), pt(1), pt(2)}, Eigen::Quaternion<double>{0.0, 0.0, 0.0, 1.0}) ) {
    std::cout << "collision!" << std::endl;
  }

  pt = scp::Vec3{0.0, 0.0, 1.16};
  std::cout << "setup: " << pt(0) << " " << pt(1) << " " << pt(2) << std::endl;
  // Obstacle checking (3D) ellipsoid-on-ellipsoid
  if (!bullet_.IsFreeState(scp::Vec3{pt(0), pt(1), pt(2)}, Eigen::Quaternion<double>{0.0, 0.0, 0.0, 1.0}) ) {
    std::cout << "collision!" << std::endl;
  }

  pt = scp::Vec3{0.0, 0.0, 1.14};
  std::cout << "setup: " << pt(0) << " " << pt(1) << " " << pt(2) << std::endl;
  // Obstacle checking (3D) ellipsoid-on-ellipsoid
  if (!bullet_.IsFreeState(scp::Vec3{pt(0), pt(1), pt(2)}, Eigen::Quaternion<double>{0.0, 0.0, 0.0, 1.0}) ) {
    std::cout << "collision!" << std::endl;
  }

  // y
  pt = scp::Vec3{0.0, 0.66, 0.0};
  std::cout << "setup: " << pt(0) << " " << pt(1) << " " << pt(2) << std::endl;
  // Obstacle checking (3D) ellipsoid-on-ellipsoid
  if (!bullet_.IsFreeState(scp::Vec3{pt(0), pt(1), pt(2)}, Eigen::Quaternion<double>{0.0, 0.0, 0.0, 1.0}) ) {
    std::cout << "collision!" << std::endl;
  }

  pt = scp::Vec3{0.0, 0.64, 0.0};
  std::cout << "setup: " << pt(0) << " " << pt(1) << " " << pt(2) << std::endl;
  // Obstacle checking (3D) ellipsoid-on-ellipsoid
  if (!bullet_.IsFreeState(scp::Vec3{pt(0), pt(1), pt(2)}, Eigen::Quaternion<double>{0.0, 0.0, 0.0, 1.0}) ) {
    std::cout << "collision!" << std::endl;
  }

  return 0;
}