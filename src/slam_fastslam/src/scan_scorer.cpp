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

// Clips the beam endpoint to the map boundary along the start→end ray.
// Writes the nearest in-map grid cell into end_cx/end_cy.
// Returns false if the direction vector is degenerate (start ≈ end).
bool clipBeamEndToMap(
  const OccupancyMapper::Config & cfg,
  double start_x, double start_y,
  double end_x, double end_y,
  int & end_cx, int & end_cy)
{
  const double dx = end_x - start_x;
  const double dy = end_y - start_y;
  if (std::hypot(dx, dy) < 1e-9) {
    return false;
  }

  double t = 1.0;
  const double map_max_x = cfg.origin_x + cfg.width * cfg.resolution;
  const double map_max_y = cfg.origin_y + cfg.height * cfg.resolution;

  if (dx > 1e-9) {
    t = std::min(t, (map_max_x - start_x) / dx);
  } else if (dx < -1e-9) {
    t = std::min(t, (cfg.origin_x - start_x) / dx);
  }
  if (dy > 1e-9) {
    t = std::min(t, (map_max_y - start_y) / dy);
  } else if (dy < -1e-9) {
    t = std::min(t, (cfg.origin_y - start_y) / dy);
  }

  end_cx = std::clamp(
    static_cast<int>(std::floor((start_x + t * dx - cfg.origin_x) / cfg.resolution)),
    0, cfg.width - 1);
  end_cy = std::clamp(
    static_cast<int>(std::floor((start_y + t * dy - cfg.origin_y) / cfg.resolution)),
    0, cfg.height - 1);
  return true;
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
    } else if (std::isinf(range) && range > 0.0f) {
      // Positive infinity means the beam reached max range without hitting anything.
      // Treat it as a max-range free-space beam (no obstacle at the endpoint).
      scoring_range = static_cast<double>(scan.range_max);
      endpoint_is_hit = false;
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
      const double beam_score = likelihood_field.likelihoodAtWorld(hit_x, hit_y);
      log_likelihood += std::log(std::max(beam_score, kMinBeamLikelihood));
    } else {
      log_likelihood += freeSpaceBeamReward(mapper, laser_x, laser_y, hit_x, hit_y);
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
  bool endpoint_clipped = false;
  if (!mapper.worldToGrid(end_x, end_y, end_cell_x, end_cell_y)) {
    // Beam exits the map; clip to the boundary so occupied crossings along the
    // in-map portion of the ray are still penalized.
    if (!clipBeamEndToMap(
        mapper.getConfig(), start_x, start_y, end_x, end_y, end_cell_x, end_cell_y))
    {
      return 0.0;
    }
    endpoint_clipped = true;
  }

  const auto cells = bresenhamLine(start_cell_x, start_cell_y, end_cell_x, end_cell_y);
  if (cells.size() <= 1U) {
    return 0.0;
  }

  // When the endpoint was clipped to the map boundary there is no real obstacle
  // there, so skip the endpoint margin (treat as if endpoint_is_hit = false).
  std::size_t end_exclusive = cells.size();
  if (endpoint_is_hit && !endpoint_clipped) {
    end_exclusive = options_.ray_endpoint_margin_cells >= end_exclusive ?
      1U :
      end_exclusive - options_.ray_endpoint_margin_cells;
  }

  const int width = mapper.getConfig().width;
  const double p = options_.ray_occupied_threshold / 100.0;
  const double lo_threshold = std::log(p / (1.0 - p));

  // Count occupied runs along the ray instead of every occupied cell, so a
  // thick wall crossing is penalized once.
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
  double end_x,
  double end_y) const
{
  if (options_.free_space_reward_per_cell <= 0.0) {
    return 0.0;
  }

  const auto & log_odds = mapper.getLogOdds();
  if (log_odds.empty()) {
    return 0.0;
  }

  int start_cx = 0;
  int start_cy = 0;
  if (!mapper.worldToGrid(start_x, start_y, start_cx, start_cy)) {
    return 0.0;
  }

  // Clip the endpoint to the map boundary (preserving direction) so Bresenham
  // walks correctly even when the beam exits the map.
  int end_cx = 0;
  int end_cy = 0;
  if (!mapper.worldToGrid(end_x, end_y, end_cx, end_cy)) {
    if (!clipBeamEndToMap(
        mapper.getConfig(), start_x, start_y, end_x, end_y, end_cx, end_cy))
    {
      return 0.0;
    }
  }

  const auto cells = bresenhamLine(start_cx, start_cy, end_cx, end_cy);

  const int width = mapper.getConfig().width;
  double reward = 0.0;

  // Start from index 1 to skip the laser-origin cell itself.
  for (std::size_t i = 1; i < cells.size(); ++i) {
    if (!mapper.isValidCell(cells[i].x, cells[i].y)) {
      break;
    }
    const int index = cells[i].y * width + cells[i].x;
    const double lo = log_odds[static_cast<std::size_t>(index)];
    if (lo < 0.0) {
      reward += options_.free_space_reward_per_cell;
    } else {
      break;  // stop at occupied or unknown cells
    }
  }

  return reward;
}

}  // namespace slam_fastslam
