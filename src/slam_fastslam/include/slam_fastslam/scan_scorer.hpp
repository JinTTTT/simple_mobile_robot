#pragma once

#include <cstddef>
#include <vector>

#include "mapping/occupancy_mapper.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "slam_fastslam/likelihood_field.hpp"
#include "slam_fastslam/utils.hpp"

namespace slam_fastslam
{

struct ScanScorerOptions
{
  std::size_t scan_beam_step{10};
  int ray_occupied_threshold{65};
  double ray_occupied_crossing_penalty{2.0};
  double ray_penalty_max_per_beam{6.0};
  std::size_t ray_endpoint_margin_cells{2};
  double free_space_reward_per_cell{0.01};
};

struct CachedScanBeam
{
  double range{0.0};
  double cos_angle{1.0};
  double sin_angle{0.0};
  bool endpoint_is_hit{false};
};

class ScanScorer
{
public:
  explicit ScanScorer(const ScanScorerOptions & options = ScanScorerOptions());

  void configure(const ScanScorerOptions & options);

  std::vector<CachedScanBeam> buildScoringBeams(
    const sensor_msgs::msg::LaserScan & scan) const;

  double scoreParticle(
    const std::vector<CachedScanBeam> & scoring_beams,
    const Pose2D & particle_pose,
    const Pose2D & laser_offset,
    const OccupancyMapper & mapper,
    const LikelihoodField & likelihood_field) const;

private:
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
    double end_x,
    double end_y) const;

  ScanScorerOptions options_{};
};

}  // namespace slam_fastslam
