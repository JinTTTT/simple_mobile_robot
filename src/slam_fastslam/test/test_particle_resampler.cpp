#include <cmath>
#include <cstddef>
#include <limits>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include "slam_fastslam/fastslam_particle.hpp"
#include "slam_fastslam/particle_resampler.hpp"

namespace
{

slam_fastslam::FastSlamParticle makeParticle(std::size_t id, double weight)
{
  slam_fastslam::FastSlamParticle particle;
  particle.id = id;
  particle.weight = weight;
  particle.pose.x = static_cast<double>(id);
  return particle;
}

std::vector<slam_fastslam::FastSlamParticle> makeParticles(
  const std::vector<double> & weights)
{
  std::vector<slam_fastslam::FastSlamParticle> particles;
  particles.reserve(weights.size());
  for (std::size_t i = 0; i < weights.size(); ++i) {
    particles.push_back(makeParticle(i, weights[i]));
  }
  return particles;
}

}  // namespace

TEST(ParticleResamplerTest, ShouldNotResampleForInvalidOrZeroEffectiveParticleCount)
{
  const slam_fastslam::ParticleResampler resampler;

  EXPECT_FALSE(resampler.shouldResample(0.0, 10U, 0.5));
  EXPECT_FALSE(resampler.shouldResample(-1.0, 10U, 0.5));
  EXPECT_FALSE(
    resampler.shouldResample(
      std::numeric_limits<double>::infinity(), 10U, 0.5));
  EXPECT_FALSE(
    resampler.shouldResample(
      std::numeric_limits<double>::quiet_NaN(), 10U, 0.5));
}

TEST(ParticleResamplerTest, ShouldResampleOnlyBelowThreshold)
{
  const slam_fastslam::ParticleResampler resampler;

  EXPECT_TRUE(resampler.shouldResample(2.9, 10U, 0.3));
  EXPECT_FALSE(resampler.shouldResample(3.0, 10U, 0.3));
  EXPECT_FALSE(resampler.shouldResample(3.1, 10U, 0.3));
}

TEST(ParticleResamplerTest, EqualWeightsKeepParticleCountAndOutputUniformWeights)
{
  slam_fastslam::ParticleResampler resampler;
  auto particles = makeParticles({0.25, 0.25, 0.25, 0.25});
  std::default_random_engine rng(7U);
  std::size_t next_particle_id = particles.size();

  resampler.resample(particles, particles.front().id, next_particle_id, rng);

  ASSERT_EQ(particles.size(), 4U);
  for (const auto & particle : particles) {
    EXPECT_NEAR(particle.weight, 0.25, 1e-12);
  }
}

TEST(ParticleResamplerTest, DegenerateWeightsResetToUniformWithoutCrashing)
{
  slam_fastslam::ParticleResampler resampler;
  auto particles = makeParticles(
  {
    0.0,
    std::numeric_limits<double>::quiet_NaN(),
    0.0,
    0.0});
  std::default_random_engine rng(11U);
  std::size_t next_particle_id = particles.size();

  resampler.resample(particles, particles.front().id, next_particle_id, rng);

  ASSERT_EQ(particles.size(), 4U);
  for (const auto & particle : particles) {
    EXPECT_TRUE(std::isfinite(particle.weight));
    EXPECT_NEAR(particle.weight, 0.25, 1e-12);
  }
}

TEST(ParticleResamplerTest, DominantParticleIsCopiedMoreOften)
{
  slam_fastslam::ParticleResampler resampler;
  auto particles = makeParticles({0.9, 0.05, 0.05});
  std::default_random_engine rng(23U);
  std::size_t next_particle_id = particles.size();

  resampler.resample(particles, particles.front().id, next_particle_id, rng);

  ASSERT_EQ(particles.size(), 3U);

  std::size_t dominant_copies = 0U;
  for (const auto & particle : particles) {
    if (particle.pose.x == 0.0) {
      ++dominant_copies;
    }
    EXPECT_NEAR(particle.weight, 1.0 / 3.0, 1e-12);
  }

  EXPECT_GE(dominant_copies, 2U);
}
