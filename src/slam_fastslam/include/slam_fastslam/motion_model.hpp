#pragma once

#include <random>
#include <vector>

#include "slam_fastslam/fastslam_particle.hpp"
#include "slam_fastslam/fastslam_types.hpp"

namespace slam_fastslam
{

struct MotionModelOptions
{
  double translation_noise_from_translation{0.05};
  double translation_noise_from_rotation{0.01};
  double translation_noise_base{0.005};
  double rotation_noise_from_rotation{0.05};
  double rotation_noise_from_translation{0.02};
  double rotation_noise_base{0.005};
};

class MotionModel
{
public:
  explicit MotionModel(const MotionModelOptions & options = MotionModelOptions());

  void configure(const MotionModelOptions & options);

  void propagate(
    std::vector<FastSlamParticle> & particles,
    const Pose2D & old_pose,
    const Pose2D & new_pose,
    std::default_random_engine & rng) const;

private:
  MotionModelOptions options_{};
};

}  // namespace slam_fastslam
