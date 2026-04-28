#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "mapping/occupancy_mapper.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "slam_fastslam/likelihood_field.hpp"
#include "slam_fastslam/scan_scorer.hpp"
#include "slam_fastslam/utils.hpp"

namespace
{

constexpr double kMaxDistance = 4.0;
constexpr double kSigma = 0.5;

OccupancyMapper makeMapper()
{
  OccupancyMapper::Config config;
  config.resolution = 1.0;
  config.width = 12;
  config.height = 5;
  config.origin_x = 0.0;
  config.origin_y = 0.0;

  OccupancyMapper mapper;
  mapper.configure(config);
  return mapper;
}

OccupancyMapper::ScanData makeSingleHitScan(double range)
{
  OccupancyMapper::ScanData scan;
  scan.ranges = {static_cast<float>(range)};
  scan.angle_min = 0.0F;
  scan.angle_increment = 0.0F;
  scan.range_min = 0.1F;
  scan.range_max = 10.0F;
  return scan;
}

sensor_msgs::msg::LaserScan makeLaserScan(const std::vector<float> & ranges)
{
  sensor_msgs::msg::LaserScan scan;
  scan.ranges = ranges;
  scan.angle_min = 0.0F;
  scan.angle_increment = 0.25F;
  scan.range_min = 0.1F;
  scan.range_max = 5.0F;
  return scan;
}

slam_fastslam::LikelihoodField buildField(const OccupancyMapper & mapper)
{
  slam_fastslam::LikelihoodField field;
  field.build(mapper, kMaxDistance, kSigma);
  return field;
}

void addObstacle(OccupancyMapper & mapper, double robot_x, double robot_y, double range)
{
  mapper.updateWithScanData(makeSingleHitScan(range), robot_x, robot_y, 0.0);
}

}  // namespace

TEST(ScanScorerTest, BuildScoringBeamsFiltersInvalidRanges)
{
  // Beam preprocessing should drop bad ranges and keep only usable scoring beams.
  slam_fastslam::ScanScorerOptions options;
  options.scan_beam_step = 1U;
  const slam_fastslam::ScanScorer scorer(options);

  const auto scan = makeLaserScan(
  {
    std::numeric_limits<float>::quiet_NaN(),
    0.05F,
    2.0F,
    6.0F,
    std::numeric_limits<float>::infinity()});

  const auto beams = scorer.buildScoringBeams(scan);

  ASSERT_EQ(beams.size(), 2U);
  EXPECT_NEAR(beams[0].range, 2.0, 1e-12);
  EXPECT_TRUE(beams[0].endpoint_is_hit);
  EXPECT_NEAR(beams[1].range, 5.0, 1e-12);
  EXPECT_FALSE(beams[1].endpoint_is_hit);
}

TEST(ScanScorerTest, EmptyScoringBeamsReturnNegativeInfinity)
{
  // Without any usable beams, scoring should return the lowest possible value.
  const slam_fastslam::ScanScorer scorer;
  const OccupancyMapper mapper = makeMapper();
  const slam_fastslam::LikelihoodField field;

  const double score = scorer.scoreParticle(
    {},
    slam_fastslam::Pose2D{},
    slam_fastslam::Pose2D{},
    mapper,
    field);

  EXPECT_EQ(score, -std::numeric_limits<double>::infinity());
}

TEST(ScanScorerTest, HitEndpointScoresHigherWhenAlignedWithObstacle)
{
  // A scan endpoint that lands on an obstacle should score better than a shifted pose.
  OccupancyMapper mapper = makeMapper();
  addObstacle(mapper, 3.5, 2.5, 1.0);
  const slam_fastslam::LikelihoodField field = buildField(mapper);

  slam_fastslam::ScanScorerOptions options;
  options.scan_beam_step = 1U;
  const slam_fastslam::ScanScorer scorer(options);
  const auto beams = scorer.buildScoringBeams(makeLaserScan({1.0F}));

  const double aligned_score = scorer.scoreParticle(
    beams,
    slam_fastslam::Pose2D{3.5, 2.5, 0.0},
    slam_fastslam::Pose2D{},
    mapper,
    field);
  const double shifted_score = scorer.scoreParticle(
    beams,
    slam_fastslam::Pose2D{7.5, 2.5, 0.0},
    slam_fastslam::Pose2D{},
    mapper,
    field);

  EXPECT_GT(aligned_score, shifted_score);
}

TEST(ScanScorerTest, RayCrossingPenaltyLowersScore)
{
  // Rays that cross occupied cells before the endpoint should be penalized.
  OccupancyMapper endpoint_only_mapper = makeMapper();
  addObstacle(endpoint_only_mapper, 6.5, 2.5, 1.0);
  const slam_fastslam::LikelihoodField endpoint_only_field = buildField(endpoint_only_mapper);

  OccupancyMapper crossing_mapper = makeMapper();
  addObstacle(crossing_mapper, 6.5, 2.5, 1.0);
  addObstacle(crossing_mapper, 3.5, 2.5, 1.0);
  const slam_fastslam::LikelihoodField crossing_field = buildField(crossing_mapper);

  slam_fastslam::ScanScorerOptions options;
  options.scan_beam_step = 1U;
  options.ray_endpoint_margin_cells = 1U;
  options.ray_occupied_crossing_penalty = 2.0;
  options.free_space_reward_per_cell = 0.0;
  const slam_fastslam::ScanScorer scorer(options);
  const auto beams = scorer.buildScoringBeams(makeLaserScan({5.0F}));

  const double no_crossing_score = scorer.scoreParticle(
    beams,
    slam_fastslam::Pose2D{2.5, 2.5, 0.0},
    slam_fastslam::Pose2D{},
    endpoint_only_mapper,
    endpoint_only_field);
  const double crossing_score = scorer.scoreParticle(
    beams,
    slam_fastslam::Pose2D{2.5, 2.5, 0.0},
    slam_fastslam::Pose2D{},
    crossing_mapper,
    crossing_field);

  EXPECT_LT(crossing_score, no_crossing_score);
}
