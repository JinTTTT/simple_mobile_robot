#include "slam_fastslam/motion_model.hpp"

#include <cmath>
#include <random>

namespace slam_fastslam
{

MotionModel::MotionModel(const MotionModelOptions & options)
{
  configure(options);
}

void MotionModel::configure(const MotionModelOptions & options)
{
  options_ = options;
}

void MotionModel::propagate(
  std::vector<FastSlamParticle> & particles,
  const Pose2D & old_pose,
  const Pose2D & new_pose,
  std::default_random_engine & rng) const
{
  double delta_rot1 = 0.0;
  const double delta_x = new_pose.x - old_pose.x;
  const double delta_y = new_pose.y - old_pose.y;
  const double delta_trans = std::hypot(delta_x, delta_y);
  if (delta_trans > 1e-6) {
    delta_rot1 = normalizeAngle(std::atan2(delta_y, delta_x) - old_pose.theta);
  }
  const double delta_rot2 = normalizeAngle(new_pose.theta - old_pose.theta - delta_rot1);

  const double total_rotation = std::abs(delta_rot1) + std::abs(delta_rot2);
  const double trans_noise_std =
    options_.translation_noise_from_translation * delta_trans +
    options_.translation_noise_from_rotation * total_rotation +
    options_.translation_noise_base;
  const double rot1_noise_std =
    options_.rotation_noise_from_rotation * std::abs(delta_rot1) +
    options_.rotation_noise_from_translation * delta_trans +
    options_.rotation_noise_base;
  const double rot2_noise_std =
    options_.rotation_noise_from_rotation * std::abs(delta_rot2) +
    options_.rotation_noise_from_translation * delta_trans +
    options_.rotation_noise_base;

  std::normal_distribution<double> trans_noise(0.0, trans_noise_std);
  std::normal_distribution<double> rot1_noise(0.0, rot1_noise_std);
  std::normal_distribution<double> rot2_noise(0.0, rot2_noise_std);

  for (auto & particle : particles) {
    const double noisy_rot1 = delta_rot1 + rot1_noise(rng);
    const double noisy_trans = delta_trans + trans_noise(rng);
    const double noisy_rot2 = delta_rot2 + rot2_noise(rng);

    particle.pose.x += noisy_trans * std::cos(particle.pose.theta + noisy_rot1);
    particle.pose.y += noisy_trans * std::sin(particle.pose.theta + noisy_rot1);
    particle.pose.theta = normalizeAngle(particle.pose.theta + noisy_rot1 + noisy_rot2);
  }
}

}  // namespace slam_fastslam
