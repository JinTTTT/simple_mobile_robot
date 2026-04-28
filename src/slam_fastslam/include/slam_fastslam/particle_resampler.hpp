#pragma once

#include <cstddef>
#include <random>
#include <vector>

#include "slam_fastslam/fastslam_particle.hpp"

namespace slam_fastslam
{

class ParticleResampler
{
public:
  bool shouldResample(
    double effective_particle_count,
    std::size_t particle_count,
    double resample_min_eff_ratio) const;

  void resample(
    std::vector<FastSlamParticle> & particles,
    std::size_t selected_particle_id,
    std::size_t & next_particle_id,
    std::default_random_engine & rng) const;
};

}  // namespace slam_fastslam
