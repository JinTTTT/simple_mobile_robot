#pragma once

#include <cstddef>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <random>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "mapping/occupancy_mapper.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace slam_fastslam
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

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

struct TrajectoryPose
{
  Pose2D pose{};
  builtin_interfaces::msg::Time stamp{};
};

class LikelihoodField
{
public:
  void build(const OccupancyMapper & mapper, double max_distance_m, double sigma_m)
  {
    const auto & cfg = mapper.getConfig();
    const int width = cfg.width;
    const int height = cfg.height;
    const int total_cells = width * height;
    if (total_cells <= 0 || max_distance_m <= 0.0) {
      field_map_.data.clear();
      distance_cells_.clear();
      return;
    }

    const double resolution = cfg.resolution;
    const int max_distance_cells = static_cast<int>(std::ceil(max_distance_m / resolution));

    // Populate field_map_ metadata so valueAtWorld() can use it.
    field_map_.info.resolution = cfg.resolution;
    field_map_.info.width = static_cast<std::uint32_t>(width);
    field_map_.info.height = static_cast<std::uint32_t>(height);
    field_map_.info.origin.position.x = cfg.origin_x;
    field_map_.info.origin.position.y = cfg.origin_y;
    field_map_.info.origin.orientation.w = 1.0;

    std::vector<int> distance_to_wall(static_cast<std::size_t>(total_cells), max_distance_cells);
    std::vector<int> queue(static_cast<std::size_t>(total_cells), 0);
    int queue_head = 0;
    int queue_tail = 0;

    const auto & log_odds = mapper.getLogOdds();
    for (int i = 0; i < total_cells; ++i) {
      if (log_odds[static_cast<std::size_t>(i)] > 0.0) {
        distance_to_wall[static_cast<std::size_t>(i)] = 0;
        queue[static_cast<std::size_t>(queue_tail++)] = i;
      }
    }

    const int offsets[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

    while (queue_head < queue_tail) {
      const int index = queue[static_cast<std::size_t>(queue_head++)];
      const int row = index / width;
      const int col = index % width;
      const int next_distance = distance_to_wall[static_cast<std::size_t>(index)] + 1;

      if (next_distance > max_distance_cells) {
        continue;
      }

      for (const auto & offset : offsets) {
        const int next_col = col + offset[0];
        const int next_row = row + offset[1];

        if (next_col < 0 || next_row < 0 || next_col >= width || next_row >= height) {
          continue;
        }

        const int next_index = next_row * width + next_col;
        if (next_distance >= distance_to_wall[static_cast<std::size_t>(next_index)]) {
          continue;
        }

        distance_to_wall[static_cast<std::size_t>(next_index)] = next_distance;
        queue[static_cast<std::size_t>(queue_tail++)] = next_index;
      }
    }

    const double two_sigma_sq = 2.0 * sigma_m * sigma_m;
    field_map_.data.assign(static_cast<std::size_t>(total_cells), 0);
    for (int i = 0; i < total_cells; ++i) {
      const double distance_m = distance_to_wall[static_cast<std::size_t>(i)] * resolution;
      const double likelihood = std::exp(-(distance_m * distance_m) / two_sigma_sq);
      field_map_.data[static_cast<std::size_t>(i)] =
        static_cast<int8_t>(std::round(likelihood * 100.0));
    }

    // Persist the distance field so incremental updates can reuse it.
    width_ = width;
    height_ = height;
    max_distance_cells_ = max_distance_cells;
    resolution_ = resolution;
    two_sigma_sq_ = two_sigma_sq;
    distance_cells_ = std::move(distance_to_wall);
  }

  // Update only the cells reachable from newly occupied cells via BFS.
  // Falls back to a full build on first call, dimension change, or when the
  // number of freed cells exceeds the noise threshold (genuine structural change).
  // A small number of freed cells is treated as sensor noise and ignored;
  // the field stays slightly stale for one scan, which is acceptable.
  void incrementalUpdate(
    const OccupancyMapper & mapper,
    double max_distance_m,
    double sigma_m,
    const std::vector<int> & newly_occupied,
    const std::vector<int> & newly_freed,
    std::size_t freed_cells_rebuild_threshold)
  {
    const auto & cfg = mapper.getConfig();
    const int new_width = cfg.width;
    const int new_height = cfg.height;
    const int new_max_dist =
      static_cast<int>(std::ceil(max_distance_m / cfg.resolution));

    const bool dimensions_changed =
      (new_width != width_ || new_height != height_ || new_max_dist != max_distance_cells_);

    const bool freed_cells_significant =
      newly_freed.size() > freed_cells_rebuild_threshold;

    if (distance_cells_.empty() || dimensions_changed || freed_cells_significant) {
      build(mapper, max_distance_m, sigma_m);
      return;
    }

    if (newly_occupied.empty()) {
      return;
    }

    // BFS from every newly occupied cell, relaxing distances outward.
    std::vector<int> queue;
    queue.reserve(newly_occupied.size() * 64);

    for (const int idx : newly_occupied) {
      if (idx < 0 || idx >= static_cast<int>(distance_cells_.size())) {
        continue;
      }
      if (distance_cells_[static_cast<std::size_t>(idx)] > 0) {
        distance_cells_[static_cast<std::size_t>(idx)] = 0;
        field_map_.data[static_cast<std::size_t>(idx)] = 100;
        queue.push_back(idx);
      }
    }

    const int offsets[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

    for (std::size_t head = 0; head < queue.size(); ++head) {
      const int current = queue[head];
      const int current_dist = distance_cells_[static_cast<std::size_t>(current)];
      const int next_dist = current_dist + 1;

      if (next_dist > max_distance_cells_) {
        continue;
      }

      const int row = current / width_;
      const int col = current % width_;

      for (const auto & offset : offsets) {
        const int next_col = col + offset[0];
        const int next_row = row + offset[1];

        if (next_col < 0 || next_row < 0 || next_col >= width_ || next_row >= height_) {
          continue;
        }

        const int next_idx = next_row * width_ + next_col;
        if (next_dist >= distance_cells_[static_cast<std::size_t>(next_idx)]) {
          continue;
        }

        distance_cells_[static_cast<std::size_t>(next_idx)] = next_dist;
        const double distance_m = next_dist * resolution_;
        const double likelihood = std::exp(-(distance_m * distance_m) / two_sigma_sq_);
        field_map_.data[static_cast<std::size_t>(next_idx)] =
          static_cast<int8_t>(std::round(likelihood * 100.0));
        queue.push_back(next_idx);
      }
    }
  }

  bool hasMap() const
  {
    return !field_map_.data.empty();
  }

  double valueAtWorld(double x, double y) const
  {
    if (!hasMap()) {
      return 0.0;
    }

    const double resolution = field_map_.info.resolution;
    const double origin_x = field_map_.info.origin.position.x;
    const double origin_y = field_map_.info.origin.position.y;
    const int col = static_cast<int>(std::floor((x - origin_x) / resolution));
    const int row = static_cast<int>(std::floor((y - origin_y) / resolution));

    if (col < 0 || row < 0 ||
      col >= static_cast<int>(field_map_.info.width) ||
      row >= static_cast<int>(field_map_.info.height))
    {
      return 0.0;
    }

    const int index = row * static_cast<int>(field_map_.info.width) + col;
    return field_map_.data[static_cast<std::size_t>(index)] / 100.0;
  }

private:
  nav_msgs::msg::OccupancyGrid field_map_{};
  std::vector<int> distance_cells_{};
  int width_{0};
  int height_{0};
  int max_distance_cells_{0};
  double resolution_{0.0};
  double two_sigma_sq_{0.0};
};


struct FastSlamParticle
{
  std::size_t id{0};
  Pose2D pose{};
  double weight{0.0};
  double log_likelihood{-std::numeric_limits<double>::infinity()};
  std::deque<TrajectoryPose> trajectory{};
  OccupancyMapper mapper{};
  LikelihoodField likelihood_field{};
  bool has_map{false};
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
  struct CachedScanBeam
  {
    double range{0.0};
    double cos_angle{1.0};
    double sin_angle{0.0};
    bool endpoint_is_hit{false};
  };

  void initializeParticles();
  void propagateParticles(const Pose2D & old_pose, const Pose2D & new_pose);
  std::vector<CachedScanBeam> buildScoringBeams(const sensor_msgs::msg::LaserScan & scan) const;
  FastSlamParticleStats scoreParticles(const std::vector<CachedScanBeam> & scoring_beams);
  double rayCrossingPenalty(
    const OccupancyMapper & mapper,
    double start_x,
    double start_y,
    double end_x,
    double end_y,
    bool endpoint_is_hit) const;
  double freeSpaceBeamReward(
    const OccupancyMapper & mapper,
    double start_x,
    double start_y,
    double beam_world_cos,
    double beam_world_sin) const;
  std::size_t bestParticleIndex() const;
  std::size_t findParticleById(std::size_t id) const;
  std::size_t selectPublishedParticleIndex(std::size_t best_index);
  bool shouldResample(double effective_particle_count) const;
  void resampleParticles(std::size_t preserved_particle_id);

  FastSlamParameters parameters_{};
  std::default_random_engine rng_{std::random_device{}()};
  static constexpr std::size_t kInvalidParticleId = std::numeric_limits<std::size_t>::max();

  OccupancyMapper::Config map_config_{};
  Pose2D laser_offset_{};
  std::vector<FastSlamParticle> particles_{};
  std::size_t next_particle_id_{0U};
  std::size_t published_particle_id_{kInvalidParticleId};
  std::size_t leading_particle_id_{kInvalidParticleId};
  int leading_wins_{0};
  bool all_particles_have_map_{false};
};

double normalizeAngle(double angle);

}  // namespace slam_fastslam
