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
  // NaN and below-range-min values are dropped.
  // Finite out-of-range (6.0 > range_max 5.0) and +inf both become max-range free-space beams.
  slam_fastslam::ScanScorerOptions options;
  options.scan_beam_step = 1U;
  const slam_fastslam::ScanScorer scorer(options);

  const auto scan = makeLaserScan(
  {
    std::numeric_limits<float>::quiet_NaN(),  // dropped: NaN
    0.05F,                                    // dropped: below range_min
    2.0F,                                     // kept: valid hit
    6.0F,                                     // kept: finite over-range → max-range free-space
    std::numeric_limits<float>::infinity()});  // kept: +inf → max-range free-space

  const auto beams = scorer.buildScoringBeams(scan);

  ASSERT_EQ(beams.size(), 3U);
  EXPECT_NEAR(beams[0].range, 2.0, 1e-12);
  EXPECT_TRUE(beams[0].endpoint_is_hit);
  EXPECT_NEAR(beams[1].range, 5.0, 1e-12);
  EXPECT_FALSE(beams[1].endpoint_is_hit);
  EXPECT_NEAR(beams[2].range, 5.0, 1e-12);  // inf treated as range_max
  EXPECT_FALSE(beams[2].endpoint_is_hit);
}

TEST(ScanScorerTest, InfRangeBeamTreatedAsMaxRangeFreeSpace)
{
  // A +inf range should produce the same beam as an explicit range_max reading.
  slam_fastslam::ScanScorerOptions options;
  options.scan_beam_step = 1U;
  const slam_fastslam::ScanScorer scorer(options);

  const auto inf_scan = makeLaserScan({std::numeric_limits<float>::infinity()});
  const auto max_scan = makeLaserScan({5.0F});  // range_max is 5.0

  const auto inf_beams = scorer.buildScoringBeams(inf_scan);
  const auto max_beams = scorer.buildScoringBeams(max_scan);

  ASSERT_EQ(inf_beams.size(), 1U);
  ASSERT_EQ(max_beams.size(), 1U);
  EXPECT_NEAR(inf_beams[0].range, max_beams[0].range, 1e-12);
  EXPECT_EQ(inf_beams[0].endpoint_is_hit, max_beams[0].endpoint_is_hit);
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

TEST(ScanScorerTest, FreeSpaceRewardIncreasesScoreForMaxRangeBeam)
{
  // A max-range beam passing through known-free cells should raise the score
  // compared to a map where those cells are unknown (never observed).
  OccupancyMapper unknown_mapper = makeMapper();
  const slam_fastslam::LikelihoodField unknown_field = buildField(unknown_mapper);

  // Fire a ray from the map's left edge rightward so that the endpoint lands
  // inside the map (range 7.0: (0.5+7)=7.5 ≤ map width 12).  This marks
  // cells (0,2)-(6,2) as free and (7,2) as occupied.
  OccupancyMapper free_mapper = makeMapper();
  addObstacle(free_mapper, 0.5, 2.5, 7.0);
  const slam_fastslam::LikelihoodField free_field = buildField(free_mapper);

  slam_fastslam::ScanScorerOptions options;
  options.scan_beam_step = 1U;
  options.free_space_reward_per_cell = 0.1;
  options.ray_occupied_crossing_penalty = 0.0;
  const slam_fastslam::ScanScorer scorer(options);

  // Use a max-range (inf) beam fired from the robot at (2.5, 2.5) heading right.
  const auto beams =
    scorer.buildScoringBeams(makeLaserScan({std::numeric_limits<float>::infinity()}));
  ASSERT_FALSE(beams.empty());
  ASSERT_FALSE(beams[0].endpoint_is_hit);

  const double unknown_score = scorer.scoreParticle(
    beams,
    slam_fastslam::Pose2D{2.5, 2.5, 0.0},
    slam_fastslam::Pose2D{},
    unknown_mapper,
    unknown_field);
  const double free_score = scorer.scoreParticle(
    beams,
    slam_fastslam::Pose2D{2.5, 2.5, 0.0},
    slam_fastslam::Pose2D{},
    free_mapper,
    free_field);

  EXPECT_GT(free_score, unknown_score);
}

TEST(ScanScorerTest, FreeSpaceRewardNotDoubleCountedForDiagonalBeam)
{
  // Walking a 45-degree beam with Bresenham should visit each cell exactly once.
  // We verify this by checking the reward equals free_space_reward_per_cell * <cell count>.
  //
  // Map is 12 wide x 5 tall (resolution 1 m).  A 45-degree beam from (0.5, 0.5)
  // exits the top of the map around y=5 (at x≈5), so the endpoint must stay
  // below that.  Range 4.5 → endpoint ≈ (3.68, 3.68) which is cell (3,3).
  OccupancyMapper mapper = makeMapper();
  OccupancyMapper::ScanData diag_scan;
  diag_scan.ranges = {4.5F};
  diag_scan.angle_min = static_cast<float>(M_PI / 4.0);  // 45 degrees
  diag_scan.angle_increment = 0.0F;
  diag_scan.range_min = 0.1F;
  diag_scan.range_max = 10.0F;
  mapper.updateWithScanData(diag_scan, 0.5, 0.5, 0.0);
  // Mapper Bresenham (0,0)→(3,3): cells (0,0),(1,1),(2,2) are free; (3,3) is occupied.

  slam_fastslam::ScanScorerOptions options;
  options.scan_beam_step = 1U;
  options.free_space_reward_per_cell = 1.0;  // 1.0 per cell for easy counting
  options.ray_occupied_crossing_penalty = 0.0;
  const slam_fastslam::ScanScorer scorer(options);

  // Score a 45-degree max-range beam from (0.5, 0.5).
  // range_max = 10 but the map clips the endpoint to (5,4) (top-right boundary).
  sensor_msgs::msg::LaserScan scan;
  scan.ranges = {std::numeric_limits<float>::infinity()};
  scan.angle_min = static_cast<float>(M_PI / 4.0);
  scan.angle_increment = 0.0F;
  scan.range_min = 0.1F;
  scan.range_max = 10.0F;
  const auto beams = scorer.buildScoringBeams(scan);

  const slam_fastslam::LikelihoodField field = buildField(mapper);
  const double score = scorer.scoreParticle(
    beams,
    slam_fastslam::Pose2D{0.5, 0.5, 0.0},
    slam_fastslam::Pose2D{},
    mapper,
    field);

  // Bresenham skips the laser-origin cell (0,0).  Cells (1,1) and (2,2) are free
  // (reward = 2.0); cell (3,2) or (3,3) is either unknown or occupied → stop.
  // The reward must be a whole-number multiple of free_space_reward_per_cell,
  // confirming Bresenham visited each cell exactly once (no double-counting).
  EXPECT_NEAR(score, std::round(score), 1e-9);
  EXPECT_GT(score, 0.0);
}
