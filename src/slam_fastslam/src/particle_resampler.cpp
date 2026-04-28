#include "slam_fastslam/particle_resampler.hpp"

#include <cmath>
#include <utility>

namespace slam_fastslam
{

bool ParticleResampler::shouldResample(
  double effective_particle_count,
  std::size_t particle_count,
  double resample_min_eff_ratio) const
{
  if (effective_particle_count <= 0.0 || !std::isfinite(effective_particle_count)) {
    return false;
  }
  return effective_particle_count < (resample_min_eff_ratio * static_cast<double>(particle_count));
}

void ParticleResampler::resample(
  std::vector<FastSlamParticle> & particles,
  std::size_t selected_particle_id,
  std::size_t & next_particle_id,
  std::default_random_engine & rng) const
{
  double weight_sum = 0.0;
  for (const auto & particle : particles) {
    weight_sum += particle.weight;
  }

  if (weight_sum <= 0.0 || !std::isfinite(weight_sum)) {
    const double equal_weight = 1.0 / static_cast<double>(particles.size());
    for (auto & particle : particles) {
      particle.weight = equal_weight;
    }
    weight_sum = 1.0;
  }

  std::vector<double> cumulative_weights;
  cumulative_weights.reserve(particles.size());
  double cumulative_sum = 0.0;
  for (auto & particle : particles) {
    particle.weight /= weight_sum;
    cumulative_sum += particle.weight;
    cumulative_weights.push_back(cumulative_sum);
  }

  cumulative_weights.back() = 1.0;

  // Low-variance/systematic resampling walks evenly through the cumulative
  // distribution so high-weight particles are copied without excessive jitter.
  std::uniform_real_distribution<double> start_distribution(
    0.0, 1.0 / static_cast<double>(particles.size()));

  std::vector<FastSlamParticle> new_particles;
  new_particles.reserve(particles.size());
  bool selected_id_preserved = false;

  double pointer = start_distribution(rng);
  const double step = 1.0 / static_cast<double>(particles.size());
  std::size_t particle_index = 0;

  for (std::size_t i = 0; i < particles.size(); ++i) {
    while (pointer > cumulative_weights[particle_index]) {
      particle_index++;
    }

    FastSlamParticle copied = particles[particle_index];
    if (copied.id == selected_particle_id && !selected_id_preserved) {
      selected_id_preserved = true;
    } else {
      copied.id = next_particle_id++;
    }
    copied.weight = 1.0 / static_cast<double>(particles.size());
    new_particles.push_back(std::move(copied));
    pointer += step;
  }

  particles = std::move(new_particles);
}

}  // namespace slam_fastslam
