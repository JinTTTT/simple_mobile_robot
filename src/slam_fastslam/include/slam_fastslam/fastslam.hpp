#pragma once

#include <cstddef>
#include <limits>
#include <random>
#include <vector>

#include "mapping/occupancy_mapper.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "slam_fastslam/fastslam_particle.hpp"
#include "slam_fastslam/motion_model.hpp"
#include "slam_fastslam/particle_resampler.hpp"
#include "slam_fastslam/scan_scorer.hpp"

namespace slam_fastslam
{

struct FastSlamParameters
{
  int num_particles{20};
  std::size_t scan_beam_step{10};
  double likelihood_max_distance{0.5};
  double likelihood_sigma{0.15};
  int ray_occupied_threshold{65};
  double ray_occupied_crossing_penalty{2.0};
  double ray_penalty_max_per_beam{6.0};
  std::size_t ray_endpoint_margin_cells{2};
  double translation_noise_from_translation{0.05};
  double translation_noise_from_rotation{0.01};
  double translation_noise_base{0.005};
  double rotation_noise_from_rotation{0.05};
  double rotation_noise_from_translation{0.02};
  double rotation_noise_base{0.005};
  double min_translation_for_update{0.10};
  double min_rotation_for_update{0.08};
  double resample_min_eff_ratio{0.3};
  double free_space_reward_per_cell{0.01};
  std::size_t traj_max_poses{500};
  int consecutive_wins_to_switch{3};
  double switch_weight_ratio{2.0};
  std::size_t freed_cells_rebuild_threshold{5};
};

struct FastSlamParticleStats
{
  double best_score{0.0};
  double average_score{0.0};
  double min_score{0.0};
  double effective_particle_count{0.0};
  std::vector<double> raw_scores{};
  std::vector<double> normalized_scores{};
};

struct FastSlamUpdateResult
{
  bool updated{false};
  bool resampled{false};
  bool particle_switched{false};
  std::size_t published_index{0U};
  double best_weight{0.0};
  double best_log_likelihood{-std::numeric_limits<double>::infinity()};
  FastSlamParticleStats stats{};
};

class FastSlam
{
public:
  FastSlam();
  FastSlam(const FastSlamParameters & parameters, const OccupancyMapper::Config & map_config);

  void configure(const FastSlamParameters & parameters, const OccupancyMapper::Config & map_config);
  void setLaserOffset(const Pose2D & laser_offset);

  bool shouldAcceptUpdate(
    const Pose2D & previous_odom_pose,
    const Pose2D & current_odom_pose) const;

  FastSlamUpdateResult update(
    const sensor_msgs::msg::LaserScan & scan,
    const Pose2D & previous_odom_pose,
    const Pose2D & current_odom_pose);

  const FastSlamParameters & parameters() const;
  const std::vector<FastSlamParticle> & particles() const;

private:
  void initializeParticles();
  FastSlamParticleStats scoreParticles(const std::vector<CachedScanBeam> & scoring_beams);
  std::size_t bestParticleIndex() const;
  std::size_t findParticleById(std::size_t id) const;
  std::size_t selectPublishedParticleIndex(std::size_t best_index);

  FastSlamParameters parameters_{};
  std::default_random_engine rng_{std::random_device{}()};
  static constexpr std::size_t kInvalidParticleId = std::numeric_limits<std::size_t>::max();

  OccupancyMapper::Config map_config_{};
  Pose2D laser_offset_{};
  MotionModel motion_model_{};
  ParticleResampler particle_resampler_{};
  ScanScorer scan_scorer_{};
  std::vector<FastSlamParticle> particles_{};
  std::size_t next_particle_id_{0U};
  std::size_t published_particle_id_{kInvalidParticleId};
  std::size_t leading_particle_id_{kInvalidParticleId};
  int leading_wins_{0};
  bool all_particles_have_map_{false};
};

}  // namespace slam_fastslam
