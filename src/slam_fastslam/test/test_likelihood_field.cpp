#include <vector>

#include <gtest/gtest.h>

#include "mapping/occupancy_mapper.hpp"
#include "slam_fastslam/likelihood_field.hpp"

namespace
{

constexpr double kMaxDistance = 4.0;
constexpr double kSigma = 1.0;

OccupancyMapper makeMapper()
{
  OccupancyMapper::Config config;
  config.resolution = 1.0;
  config.width = 10;
  config.height = 3;
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

OccupancyMapper::ScanData makeNoHitScan(double range_max)
{
  OccupancyMapper::ScanData scan;
  scan.ranges = {static_cast<float>(range_max + 1.0)};
  scan.angle_min = 0.0F;
  scan.angle_increment = 0.0F;
  scan.range_min = 0.1F;
  scan.range_max = static_cast<float>(range_max);
  return scan;
}

void clearMapperChanges(OccupancyMapper & mapper)
{
  std::vector<int> newly_occupied;
  std::vector<int> newly_freed;
  mapper.getAndClearChanges(newly_occupied, newly_freed);
}

}  // namespace

TEST(LikelihoodFieldTest, IncrementalOccupiedUpdateIncreasesLikelihoodNearNewObstacle)
{
  // Adding a new occupied cell should raise likelihood nearby without drifting elsewhere.
  OccupancyMapper mapper = makeMapper();
  slam_fastslam::LikelihoodField field;

  mapper.updateWithScanData(makeSingleHitScan(1.0), 1.5, 1.5, 0.0);
  clearMapperChanges(mapper);
  field.build(mapper, kMaxDistance, kSigma);

  const double before_old_obstacle = field.valueAtWorld(2.5, 1.5);
  const double before_new_obstacle = field.valueAtWorld(7.5, 1.5);
  const double before_near_new_obstacle = field.valueAtWorld(6.5, 1.5);
  const double before_unaffected_cell = field.valueAtWorld(4.5, 1.5);

  mapper.updateWithScanData(makeSingleHitScan(1.0), 6.5, 1.5, 0.0);

  std::vector<int> newly_occupied;
  std::vector<int> newly_freed;
  mapper.getAndClearChanges(newly_occupied, newly_freed);

  ASSERT_FALSE(newly_occupied.empty());
  ASSERT_TRUE(newly_freed.empty());

  field.incrementalUpdate(
    mapper,
    kMaxDistance,
    kSigma,
    newly_occupied,
    newly_freed,
    100U);

  EXPECT_NEAR(field.valueAtWorld(2.5, 1.5), before_old_obstacle, 1e-12);
  EXPECT_GT(field.valueAtWorld(7.5, 1.5), before_new_obstacle);
  EXPECT_GT(field.valueAtWorld(6.5, 1.5), before_near_new_obstacle);
  EXPECT_NEAR(field.valueAtWorld(4.5, 1.5), before_unaffected_cell, 1e-12);
}

TEST(LikelihoodFieldTest, IncrementalUpdateRebuildsWhenObstacleIsFreed)
{
  // Freed cells should force a full rebuild, and the result should match a fresh field.
  OccupancyMapper mapper = makeMapper();
  slam_fastslam::LikelihoodField field;

  mapper.updateWithScanData(makeSingleHitScan(1.0), 1.5, 1.5, 0.0);
  mapper.updateWithScanData(makeSingleHitScan(1.0), 6.5, 1.5, 0.0);
  clearMapperChanges(mapper);
  field.build(mapper, kMaxDistance, kSigma);

  const double before_old_obstacle = field.valueAtWorld(2.5, 1.5);
  const double before_removed_obstacle = field.valueAtWorld(7.5, 1.5);

  mapper.updateWithScanData(makeNoHitScan(3.0), 6.5, 1.5, 0.0);

  std::vector<int> newly_occupied;
  std::vector<int> newly_freed;
  mapper.getAndClearChanges(newly_occupied, newly_freed);

  ASSERT_TRUE(newly_occupied.empty());
  ASSERT_FALSE(newly_freed.empty());

  field.incrementalUpdate(
    mapper,
    kMaxDistance,
    kSigma,
    newly_occupied,
    newly_freed,
    0U);

  slam_fastslam::LikelihoodField rebuilt_field;
  rebuilt_field.build(mapper, kMaxDistance, kSigma);

  EXPECT_NEAR(field.valueAtWorld(2.5, 1.5), before_old_obstacle, 1e-12);
  EXPECT_LT(field.valueAtWorld(7.5, 1.5), before_removed_obstacle);
  EXPECT_NEAR(
    field.valueAtWorld(7.5, 1.5),
    rebuilt_field.valueAtWorld(7.5, 1.5),
    1e-12);
}
