#include <vector>

#include <gtest/gtest.h>

#include "mapping/occupancy_mapper.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "slam_fastslam/fastslam.hpp"

namespace
{

slam_fastslam::FastSlamParameters makeParameters()
{
  slam_fastslam::FastSlamParameters parameters;
  parameters.num_particles = 3;
  parameters.min_translation_for_update = 0.10;
  parameters.min_rotation_for_update = 0.08;
  return parameters;
}

OccupancyMapper::Config makeMapConfig()
{
  OccupancyMapper::Config config;
  config.resolution = 1.0;
  config.width = 10;
  config.height = 10;
  config.origin_x = 0.0;
  config.origin_y = 0.0;
  return config;
}

sensor_msgs::msg::LaserScan makeSingleBeamScan(double range)
{
  sensor_msgs::msg::LaserScan scan;
  scan.ranges = {static_cast<float>(range)};
  scan.angle_min = 0.0F;
  scan.angle_increment = 0.0F;
  scan.range_min = 0.1F;
  scan.range_max = 10.0F;
  return scan;
}

}  // namespace

TEST(FastSlamTest, ConfigureInitializesParticlesWithEqualWeights)
{
  // Configure should recreate the particle set with predictable initial state.
  slam_fastslam::FastSlam slam(makeParameters(), makeMapConfig());

  const auto & particles = slam.particles();
  ASSERT_EQ(particles.size(), 3U);

  for (std::size_t i = 0; i < particles.size(); ++i) {
    EXPECT_EQ(particles[i].id, i);
    EXPECT_NEAR(particles[i].weight, 1.0 / 3.0, 1e-12);
    EXPECT_DOUBLE_EQ(particles[i].pose.x, 0.0);
    EXPECT_DOUBLE_EQ(particles[i].pose.y, 0.0);
    EXPECT_DOUBLE_EQ(particles[i].pose.theta, 0.0);
    EXPECT_FALSE(particles[i].has_map);
    EXPECT_TRUE(particles[i].trajectory.empty());
  }
}

TEST(FastSlamTest, ShouldAcceptUpdateUsesTranslationAndRotationThresholds)
{
  // A scan update should only be accepted once translation or rotation crosses the threshold.
  slam_fastslam::FastSlam slam(makeParameters(), makeMapConfig());

  const slam_fastslam::Pose2D previous_pose{0.0, 0.0, 0.0};

  EXPECT_FALSE(
    slam.shouldAcceptUpdate(
      previous_pose,
      slam_fastslam::Pose2D{0.05, 0.0, 0.0}));

  EXPECT_TRUE(
    slam.shouldAcceptUpdate(
      previous_pose,
      slam_fastslam::Pose2D{0.10, 0.0, 0.0}));

  EXPECT_FALSE(
    slam.shouldAcceptUpdate(
      previous_pose,
      slam_fastslam::Pose2D{0.0, 0.0, 0.07}));

  EXPECT_TRUE(
    slam.shouldAcceptUpdate(
      previous_pose,
      slam_fastslam::Pose2D{0.0, 0.0, 0.08}));
}

TEST(FastSlamTest, SelectedParticleStaysStableAcrossRepeatedEqualUpdates)
{
  // With zero motion noise and identical scans, the selected particle should stay stable.
  slam_fastslam::FastSlamParameters parameters = makeParameters();
  parameters.translation_noise_from_translation = 0.0;
  parameters.translation_noise_from_rotation = 0.0;
  parameters.translation_noise_base = 0.0;
  parameters.rotation_noise_from_rotation = 0.0;
  parameters.rotation_noise_from_translation = 0.0;
  parameters.rotation_noise_base = 0.0;
  slam_fastslam::FastSlam slam(parameters, makeMapConfig());

  const auto scan = makeSingleBeamScan(1.0);
  const slam_fastslam::Pose2D previous_pose{};
  const slam_fastslam::Pose2D current_pose{};

  const auto first_update = slam.update(scan, previous_pose, current_pose);
  const auto second_update = slam.update(scan, previous_pose, current_pose);
  const auto third_update = slam.update(scan, previous_pose, current_pose);

  EXPECT_TRUE(first_update.selected_particle_changed);
  EXPECT_FALSE(second_update.selected_particle_changed);
  EXPECT_FALSE(third_update.selected_particle_changed);
  EXPECT_EQ(first_update.selected_particle_index, 0U);
  EXPECT_EQ(second_update.selected_particle_index, 0U);
  EXPECT_EQ(third_update.selected_particle_index, 0U);
}

TEST(FastSlamTest, TrajectoryGrowsAndIsTrimmedToConfiguredLimit)
{
  // Each accepted update appends one pose per particle, then trims old history past the limit.
  slam_fastslam::FastSlamParameters parameters = makeParameters();
  parameters.num_particles = 2;
  parameters.traj_max_poses = 2;
  parameters.translation_noise_from_translation = 0.0;
  parameters.translation_noise_from_rotation = 0.0;
  parameters.translation_noise_base = 0.0;
  parameters.rotation_noise_from_rotation = 0.0;
  parameters.rotation_noise_from_translation = 0.0;
  parameters.rotation_noise_base = 0.0;
  slam_fastslam::FastSlam slam(parameters, makeMapConfig());

  const auto scan = makeSingleBeamScan(1.0);
  const slam_fastslam::Pose2D pose0{0.0, 0.0, 0.0};
  const slam_fastslam::Pose2D pose1{0.2, 0.0, 0.0};
  const slam_fastslam::Pose2D pose2{0.4, 0.0, 0.0};

  slam.update(scan, pose0, pose1);
  ASSERT_EQ(slam.particles().front().trajectory.size(), 1U);

  slam.update(scan, pose1, pose2);
  ASSERT_EQ(slam.particles().front().trajectory.size(), 2U);

  slam.update(scan, pose2, pose2);
  for (const auto & particle : slam.particles()) {
    EXPECT_EQ(particle.trajectory.size(), 2U);
  }
}

TEST(FastSlamTest, UpdateResultStatsArePopulatedAfterFirstMapExists)
{
  // After the first scan update builds a map, subsequent updates should fill
  // best_score, min_score, and average_score with finite values.
  slam_fastslam::FastSlamParameters parameters = makeParameters();
  parameters.translation_noise_from_translation = 0.0;
  parameters.translation_noise_from_rotation = 0.0;
  parameters.translation_noise_base = 0.0;
  parameters.rotation_noise_from_rotation = 0.0;
  parameters.rotation_noise_from_translation = 0.0;
  parameters.rotation_noise_base = 0.0;
  slam_fastslam::FastSlam slam(parameters, makeMapConfig());

  const auto scan = makeSingleBeamScan(1.0);
  const slam_fastslam::Pose2D pose0{0.0, 0.0, 0.0};
  const slam_fastslam::Pose2D pose1{0.2, 0.0, 0.0};
  const slam_fastslam::Pose2D pose2{0.4, 0.0, 0.0};

  // First update builds the map but has no prior map to score against,
  // so stats are not populated yet.
  slam.update(scan, pose0, pose1);

  // Second update scores particles against their just-built maps.
  const auto result = slam.update(scan, pose1, pose2);
  ASSERT_TRUE(result.updated);

  EXPECT_TRUE(std::isfinite(result.stats.best_score));
  EXPECT_TRUE(std::isfinite(result.stats.min_score));
  EXPECT_TRUE(std::isfinite(result.stats.average_score));
  EXPECT_GE(result.stats.best_score, result.stats.average_score);
  EXPECT_GE(result.stats.average_score, result.stats.min_score);
}
