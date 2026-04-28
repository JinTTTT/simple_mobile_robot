#pragma once

#include <cstddef>
#include <deque>
#include <limits>

#include "builtin_interfaces/msg/time.hpp"
#include "mapping/occupancy_mapper.hpp"
#include "slam_fastslam/fastslam_types.hpp"
#include "slam_fastslam/likelihood_field.hpp"

namespace slam_fastslam
{

struct TrajectoryPose
{
  Pose2D pose{};
  builtin_interfaces::msg::Time stamp{};
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

}  // namespace slam_fastslam
