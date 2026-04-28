#include "slam_fastslam/scan_scorer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace slam_fastslam
{
namespace
{

struct GridCell
{
  int x{0};
  int y{0};
};

std::vector<GridCell> bresenhamLine(int x0, int y0, int x1, int y1)
{
  std::vector<GridCell> cells;

  const int dx = std::abs(x1 - x0);
  const int dy = std::abs(y1 - y0);
  const int sx = (x0 < x1) ? 1 : -1;
  const int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  int x = x0;
  int y = y0;

  while (true) {
    cells.push_back(GridCell{x, y});

    if (x == x1 && y == y1) {
      break;
    }

    const int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x += sx;
    }
    if (e2 < dx) {
      err += dx;
      y += sy;
    }
  }

  return cells;
}

}  // namespace

ScanScorer::ScanScorer(const ScanScorerOptions & options)
{
  configure(options);
}

void ScanScorer::configure(const ScanScorerOptions & options)
{
  options_ = options;
}

std::vector<CachedScanBeam> ScanScorer::buildScoringBeams(
  const sensor_msgs::msg::LaserScan & scan) const
{
  std::vector<CachedScanBeam> beams;
  beams.reserve((scan.ranges.size() + options_.scan_beam_step - 1U) / options_.scan_beam_step);

  for (std::size_t i = 0; i < scan.ranges.size(); i += options_.scan_beam_step) {
    const float range = scan.ranges[i];
    if (std::isnan(range) || range < scan.range_min) {
      continue;
    }

    bool endpoint_is_hit = false;
    double scoring_range = scan.range_max;
    if (std::isfinite(range)) {
      scoring_range = std::min(static_cast<double>(range), static_cast<double>(scan.range_max));
      endpoint_is_hit = range < scan.range_max;
    } else {
      continue;
    }

    if (scoring_range <= scan.range_min) {
      continue;
    }

    const double beam_angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    beams.push_back(
      CachedScanBeam{
        scoring_range,
        std::cos(beam_angle),
        std::sin(beam_angle),
        endpoint_is_hit});
  }

  return beams;
}

double ScanScorer::scoreParticle(
  const std::vector<CachedScanBeam> & scoring_beams,
  const Pose2D & particle_pose,
  const Pose2D & laser_offset,
  const OccupancyMapper & mapper,
  const LikelihoodField & likelihood_field) const
{
  if (scoring_beams.empty()) {
    return -std::numeric_limits<double>::infinity();
  }

  constexpr double kMinBeamLikelihood = 1e-9;
  double log_likelihood = 0.0;
  const double cos_theta = std::cos(particle_pose.theta);
  const double sin_theta = std::sin(particle_pose.theta);

  const double laser_x =
    particle_pose.x + laser_offset.x * cos_theta - laser_offset.y * sin_theta;
  const double laser_y =
    particle_pose.y + laser_offset.x * sin_theta + laser_offset.y * cos_theta;
  const double laser_theta = particle_pose.theta + laser_offset.theta;
  const double cos_laser = std::cos(laser_theta);
  const double sin_laser = std::sin(laser_theta);

  for (const auto & beam : scoring_beams) {
    const double beam_world_cos = cos_laser * beam.cos_angle - sin_laser * beam.sin_angle;
    const double beam_world_sin = sin_laser * beam.cos_angle + cos_laser * beam.sin_angle;
    const double hit_x = laser_x + beam.range * beam_world_cos;
    const double hit_y = laser_y + beam.range * beam_world_sin;
    if (beam.endpoint_is_hit) {
      const double beam_score = likelihood_field.valueAtWorld(hit_x, hit_y);
      log_likelihood += std::log(std::max(beam_score, kMinBeamLikelihood));
    } else {
      log_likelihood += freeSpaceBeamReward(
        mapper, laser_x, laser_y, beam_world_cos, beam_world_sin);
    }
    log_likelihood -= rayCrossingPenalty(
      mapper,
      laser_x,
      laser_y,
      hit_x,
      hit_y,
      beam.endpoint_is_hit);
  }

  return log_likelihood;
}

double ScanScorer::rayCrossingPenalty(
  const OccupancyMapper & mapper,
  double start_x,
  double start_y,
  double end_x,
  double end_y,
  bool endpoint_is_hit) const
{
  const auto & log_odds = mapper.getLogOdds();
  if (log_odds.empty()) {
    return 0.0;
  }

  int start_cell_x = 0;
  int start_cell_y = 0;
  if (!mapper.worldToGrid(start_x, start_y, start_cell_x, start_cell_y)) {
    return 0.0;
  }

  int end_cell_x = 0;
  int end_cell_y = 0;
  if (!mapper.worldToGrid(end_x, end_y, end_cell_x, end_cell_y)) {
    return 0.0;
  }

  const auto cells = bresenhamLine(start_cell_x, start_cell_y, end_cell_x, end_cell_y);
  if (cells.size() <= 1U) {
    return 0.0;
  }

  std::size_t end_exclusive = cells.size();
  if (endpoint_is_hit) {
    end_exclusive = options_.ray_endpoint_margin_cells >= end_exclusive ?
      1U :
      end_exclusive - options_.ray_endpoint_margin_cells;
  }

  const int width = mapper.getConfig().width;
  const double p = options_.ray_occupied_threshold / 100.0;
  const double lo_threshold = std::log(p / (1.0 - p));

  double penalty = 0.0;
  bool inside_occupied_run = false;
  for (std::size_t i = 1; i < end_exclusive; ++i) {
    if (!mapper.isValidCell(cells[i].x, cells[i].y)) {
      continue;
    }
    const int index = cells[i].y * width + cells[i].x;
    const bool occupied = log_odds[static_cast<std::size_t>(index)] >= lo_threshold;

    if (occupied && !inside_occupied_run) {
      penalty += options_.ray_occupied_crossing_penalty;
      inside_occupied_run = true;
      if (penalty >= options_.ray_penalty_max_per_beam) {
        return options_.ray_penalty_max_per_beam;
      }
    } else if (!occupied) {
      inside_occupied_run = false;
    }
  }

  return penalty;
}

double ScanScorer::freeSpaceBeamReward(
  const OccupancyMapper & mapper,
  double start_x,
  double start_y,
  double beam_world_cos,
  double beam_world_sin) const
{
  if (options_.free_space_reward_per_cell <= 0.0) {
    return 0.0;
  }

  const auto & log_odds = mapper.getLogOdds();
  if (log_odds.empty()) {
    return 0.0;
  }

  int cx = 0;
  int cy = 0;
  if (!mapper.worldToGrid(start_x, start_y, cx, cy)) {
    return 0.0;
  }

  const auto & cfg = mapper.getConfig();
  const double res = cfg.resolution;
  const int max_steps = cfg.width + cfg.height;
  double reward = 0.0;

  for (int step = 1; step <= max_steps; ++step) {
    const double wx = start_x + step * res * beam_world_cos;
    const double wy = start_y + step * res * beam_world_sin;
    int cell_x = 0;
    int cell_y = 0;
    if (!mapper.worldToGrid(wx, wy, cell_x, cell_y)) {
      break;
    }
    const int index = cell_y * cfg.width + cell_x;
    const double lo = log_odds[static_cast<std::size_t>(index)];
    if (lo < 0.0) {
      reward += options_.free_space_reward_per_cell;
    } else {
      break;
    }
  }

  return reward;
}

}  // namespace slam_fastslam
