#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <iomanip>
#include <functional>
#include <limits>
#include <sstream>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "mapping/occupancy_mapper.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

struct Settings
{
  int num_particles{20};
  std::size_t scan_beam_step{10};
  double likelihood_max_distance{0.5};
  double translation_noise_from_translation{0.01};
  double translation_noise_base{0.001};
  double rotation_noise_from_rotation{0.02};
  double rotation_noise_from_translation{0.005};
  double rotation_noise_base{0.001};
  double min_translation_for_update{0.10};
  double min_rotation_for_update{0.08};
};

struct ParticleStats
{
  double best_score{0.0};
  double average_score{0.0};
  double min_score{0.0};
  double effective_particle_count{0.0};
  std::vector<double> raw_scores{};
  std::vector<double> normalized_scores{};
};

struct OdomSample
{
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  Pose2D pose{};
};

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);

  geometry_msgs::msg::Quaternion msg;
  msg.x = quaternion.x();
  msg.y = quaternion.y();
  msg.z = quaternion.z();
  msg.w = quaternion.w();
  return msg;
}

Pose2D odomMsgToPose(const nav_msgs::msg::Odometry & msg)
{
  Pose2D pose;
  pose.x = msg.pose.pose.position.x;
  pose.y = msg.pose.pose.position.y;

  const auto & orientation = msg.pose.pose.orientation;
  tf2::Quaternion quaternion(orientation.x, orientation.y, orientation.z, orientation.w);
  double roll = 0.0;
  double pitch = 0.0;
  tf2::Matrix3x3(quaternion).getRPY(roll, pitch, pose.theta);
  return pose;
}

std::string formatScoreList(const std::vector<double> & values)
{
  std::ostringstream stream;
  stream << "[";
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i > 0U) {
      stream << ", ";
    }
    stream << std::fixed << std::setprecision(4) << values[i];
  }
  stream << "]";
  return stream.str();
}

class LikelihoodField
{
public:
  void build(const nav_msgs::msg::OccupancyGrid & map, double max_distance_m)
  {
    field_map_ = map;

    const int width = static_cast<int>(map.info.width);
    const int height = static_cast<int>(map.info.height);
    const int total_cells = width * height;
    if (total_cells <= 0 || max_distance_m <= 0.0) {
      field_map_.data.clear();
      return;
    }

    const double resolution = map.info.resolution;
    const int max_distance_cells = static_cast<int>(std::ceil(max_distance_m / resolution));

    std::vector<int> distance_to_wall(static_cast<std::size_t>(total_cells), max_distance_cells);
    std::vector<int> queue(static_cast<std::size_t>(total_cells), 0);
    int queue_head = 0;
    int queue_tail = 0;

    for (int i = 0; i < total_cells; ++i) {
      if (map.data[static_cast<std::size_t>(i)] > 50) {
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

    field_map_.data.assign(static_cast<std::size_t>(total_cells), 0);
    for (int i = 0; i < total_cells; ++i) {
      const double distance_m = distance_to_wall[static_cast<std::size_t>(i)] * resolution;
      const double likelihood = 1.0 - std::min(distance_m / max_distance_m, 1.0);
      field_map_.data[static_cast<std::size_t>(i)] =
        static_cast<int8_t>(std::round(likelihood * 100.0));
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
};

struct Particle
{
  Pose2D pose{};
  double weight{0.0};
  struct TrajectoryPose
  {
    Pose2D pose{};
    builtin_interfaces::msg::Time stamp{};
  };
  std::vector<TrajectoryPose> trajectory{};
  OccupancyMapper mapper{};
  nav_msgs::msg::OccupancyGrid map_msg{};
  LikelihoodField likelihood_field{};
  bool has_map{false};
};

class FastSlamNode : public rclcpp::Node
{
public:
  FastSlamNode()
  : Node("fastslam_node")
  {
    configureMapConfig();
    initializeParticles();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      rclcpp::SensorDataQoS(),
      std::bind(&FastSlamNode::odomCallback, this, std::placeholders::_1));

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      rclcpp::SensorDataQoS(),
      std::bind(&FastSlamNode::scanCallback, this, std::placeholders::_1));

    const auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", map_qos);
    particle_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/particlecloud", 10);
    best_path_pub_ = create_publisher<nav_msgs::msg::Path>("/best_path", 10);
    estimated_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/estimated_pose", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(
      get_logger(),
      "fastslam_node started with %d particles and motion thresholds (%.2f m, %.2f rad). "
      "Outputs include TF map->odom.",
      settings_.num_particles,
      settings_.min_translation_for_update,
      settings_.min_rotation_for_update);
  }

private:
  void initializeParticles()
  {
    particles_.assign(static_cast<std::size_t>(settings_.num_particles), Particle{});
    const double initial_weight = 1.0 / static_cast<double>(settings_.num_particles);
    for (auto & particle : particles_) {
      particle.pose = Pose2D{};
      particle.weight = initial_weight;
      particle.trajectory.clear();
      particle.mapper.configure(map_config_);
      particle.map_msg = nav_msgs::msg::OccupancyGrid{};
      particle.has_map = false;
    }
  }

  void configureMapConfig()
  {
    map_config_.resolution = 0.05;
    map_config_.width = 500;
    map_config_.height = 500;
    map_config_.origin_x =
      -static_cast<double>(map_config_.width) * map_config_.resolution / 2.0;
    map_config_.origin_y =
      -static_cast<double>(map_config_.height) * map_config_.resolution / 2.0;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const Pose2D odom_pose = odomMsgToPose(*msg);
    const rclcpp::Time odom_stamp(msg->header.stamp);
    odom_buffer_.push_back(OdomSample{odom_stamp, odom_pose});
    pruneOdomBuffer(odom_stamp);
    have_odom_ = true;

    if (!have_last_accepted_odom_) {
      last_accepted_odom_pose_ = odom_pose;
      have_last_accepted_odom_ = true;
      RCLCPP_INFO_ONCE(get_logger(), "Received first odometry message.");
    }
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (!have_odom_ || !have_last_accepted_odom_) {
      return;
    }

    Pose2D scan_odom_pose;
    if (!lookupOdomPoseAtStamp(msg->header.stamp, scan_odom_pose)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Could not align /odom to scan stamp. Skipping scan update.");
      return;
    }

    if (!shouldAcceptUpdate(scan_odom_pose)) {
      return;
    }

    propagateParticles(last_accepted_odom_pose_, scan_odom_pose);

    ParticleStats particle_stats;
    if (anyParticleHasMap()) {
      particle_stats = scoreParticles(*msg);
    } else {
      const double equal_weight = 1.0 / static_cast<double>(particles_.size());
      for (auto & particle : particles_) {
        particle.weight = equal_weight;
      }
      particle_stats.raw_scores.assign(particles_.size(), 0.0);
      particle_stats.normalized_scores.assign(
        particles_.size(),
        1.0 / static_cast<double>(particles_.size()));
    }

    const std::size_t best_index = bestParticleIndex();
    const Pose2D best_pose = particles_[best_index].pose;
    const std::size_t best_keyframe_count = particles_[best_index].trajectory.size() + 1U;
    for (auto & particle : particles_) {
      Particle::TrajectoryPose trajectory_pose;
      trajectory_pose.pose = particle.pose;
      trajectory_pose.stamp = msg->header.stamp;
      particle.trajectory.push_back(trajectory_pose);
      particle.mapper.updateWithScan(*msg, particle.pose.x, particle.pose.y, particle.pose.theta);
      particle.map_msg = particle.mapper.buildOccupancyGridMsg("map", msg->header.stamp);
      particle.likelihood_field.build(particle.map_msg, settings_.likelihood_max_distance);
      particle.has_map = true;
    }

    const ParticleStats normalized_stats = computeParticleStats();
    publishState(best_index, msg->header.stamp, scan_odom_pose);
    const bool resampled = shouldResample(normalized_stats.effective_particle_count);
    if (resampled) {
      resampleParticles();
    }

    accepted_update_count_ += 1;
    last_accepted_odom_pose_ = scan_odom_pose;

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "accepted_updates=%zu keyframes=%zu best_particle=%zu resampled=%s N_eff=%.2f best_score=%.4f avg_score=%.4f min_score=%.4f best_pose=(%.3f, %.3f, %.3f)",
      accepted_update_count_,
      best_keyframe_count,
      best_index,
      resampled ? "yes" : "no",
      normalized_stats.effective_particle_count,
      normalized_stats.best_score,
      normalized_stats.average_score,
      normalized_stats.min_score,
      best_pose.x,
      best_pose.y,
      best_pose.theta);
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "log_likelihoods=%s normalized_weights=%s",
      formatScoreList(particle_stats.raw_scores).c_str(),
      formatScoreList(normalized_stats.normalized_scores).c_str());
  }

  bool shouldAcceptUpdate(const Pose2D & current_odom_pose) const
  {
    const double dx = current_odom_pose.x - last_accepted_odom_pose_.x;
    const double dy = current_odom_pose.y - last_accepted_odom_pose_.y;
    const double translation = std::hypot(dx, dy);
    const double rotation = std::abs(
      normalizeAngle(current_odom_pose.theta - last_accepted_odom_pose_.theta));

    return translation >= settings_.min_translation_for_update ||
           rotation >= settings_.min_rotation_for_update;
  }

  void pruneOdomBuffer(const rclcpp::Time & newest_stamp)
  {
    const rclcpp::Duration max_history = rclcpp::Duration::from_seconds(5.0);
    while (!odom_buffer_.empty() && (newest_stamp - odom_buffer_.front().stamp) > max_history) {
      odom_buffer_.pop_front();
    }
  }

  bool lookupOdomPoseAtStamp(
    const builtin_interfaces::msg::Time & target_stamp_msg,
    Pose2D & interpolated_pose) const
  {
    if (odom_buffer_.empty()) {
      return false;
    }

    const rclcpp::Time target_stamp(target_stamp_msg);
    const rclcpp::Duration tolerance = rclcpp::Duration::from_seconds(0.10);

    if (target_stamp <= odom_buffer_.front().stamp) {
      if ((odom_buffer_.front().stamp - target_stamp) <= tolerance) {
        interpolated_pose = odom_buffer_.front().pose;
        return true;
      }
      return false;
    }

    if (target_stamp >= odom_buffer_.back().stamp) {
      if ((target_stamp - odom_buffer_.back().stamp) <= tolerance) {
        interpolated_pose = odom_buffer_.back().pose;
        return true;
      }
      return false;
    }

    for (std::size_t i = 1; i < odom_buffer_.size(); ++i) {
      const OdomSample & older = odom_buffer_[i - 1];
      const OdomSample & newer = odom_buffer_[i];

      if (target_stamp > newer.stamp) {
        continue;
      }

      const double dt = (newer.stamp - older.stamp).seconds();
      if (dt <= 1e-9) {
        interpolated_pose = newer.pose;
        return true;
      }

      const double ratio = (target_stamp - older.stamp).seconds() / dt;
      interpolated_pose.x = older.pose.x + ratio * (newer.pose.x - older.pose.x);
      interpolated_pose.y = older.pose.y + ratio * (newer.pose.y - older.pose.y);
      interpolated_pose.theta = normalizeAngle(
        older.pose.theta + ratio * normalizeAngle(newer.pose.theta - older.pose.theta));
      return true;
    }

    return false;
  }

  void propagateParticles(const Pose2D & old_pose, const Pose2D & new_pose)
  {
    double delta_rot1 = 0.0;
    const double delta_x = new_pose.x - old_pose.x;
    const double delta_y = new_pose.y - old_pose.y;
    const double delta_trans = std::hypot(delta_x, delta_y);
    if (delta_trans > 1e-6) {
      delta_rot1 = normalizeAngle(std::atan2(delta_y, delta_x) - old_pose.theta);
    }
    const double delta_rot2 = normalizeAngle(new_pose.theta - old_pose.theta - delta_rot1);

    const double trans_noise_std =
      settings_.translation_noise_from_translation * delta_trans +
      settings_.translation_noise_base;
    const double rot1_noise_std =
      settings_.rotation_noise_from_rotation * std::abs(delta_rot1) +
      settings_.rotation_noise_from_translation * delta_trans +
      settings_.rotation_noise_base;
    const double rot2_noise_std =
      settings_.rotation_noise_from_rotation * std::abs(delta_rot2) +
      settings_.rotation_noise_from_translation * delta_trans +
      settings_.rotation_noise_base;

    std::normal_distribution<double> trans_noise(0.0, trans_noise_std);
    std::normal_distribution<double> rot1_noise(0.0, rot1_noise_std);
    std::normal_distribution<double> rot2_noise(0.0, rot2_noise_std);

    for (auto & particle : particles_) {
      const double noisy_rot1 = delta_rot1 + rot1_noise(rng_);
      const double noisy_trans = delta_trans + trans_noise(rng_);
      const double noisy_rot2 = delta_rot2 + rot2_noise(rng_);

      particle.pose.x += noisy_trans * std::cos(particle.pose.theta + noisy_rot1);
      particle.pose.y += noisy_trans * std::sin(particle.pose.theta + noisy_rot1);
      particle.pose.theta = normalizeAngle(particle.pose.theta + noisy_rot1 + noisy_rot2);
    }
  }

  bool anyParticleHasMap() const
  {
    return std::any_of(
      particles_.begin(),
      particles_.end(),
      [](const Particle & particle) { return particle.has_map; });
  }

  ParticleStats scoreParticles(const sensor_msgs::msg::LaserScan & scan)
  {
    ParticleStats stats;
    stats.raw_scores.resize(particles_.size(), 0.0);
    stats.normalized_scores.resize(particles_.size(), 0.0);
    constexpr double kMinBeamLikelihood = 1e-9;
    const double invalid_log_likelihood = -std::numeric_limits<double>::infinity();
    double max_log_likelihood = invalid_log_likelihood;
    for (std::size_t particle_index = 0; particle_index < particles_.size(); ++particle_index) {
      auto & particle = particles_[particle_index];
      if (!particle.has_map || !particle.likelihood_field.hasMap()) {
        particle.weight = 0.0;
        stats.raw_scores[particle_index] = invalid_log_likelihood;
        continue;
      }

      double log_likelihood = 0.0;
      std::size_t used_beams = 0;

      for (std::size_t i = 0; i < scan.ranges.size(); i += settings_.scan_beam_step) {
        const float range = scan.ranges[i];
        if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
          continue;
        }

        const double beam_angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
        const double hit_x = particle.pose.x + range * std::cos(particle.pose.theta + beam_angle);
        const double hit_y = particle.pose.y + range * std::sin(particle.pose.theta + beam_angle);
        const double beam_score = particle.likelihood_field.valueAtWorld(hit_x, hit_y);
        log_likelihood += std::log(std::max(beam_score, kMinBeamLikelihood));
        used_beams++;
      }

      if (used_beams == 0U) {
        log_likelihood = invalid_log_likelihood;
      }

      particle.weight = 0.0;
      stats.raw_scores[particle_index] = log_likelihood;
      if (std::isfinite(log_likelihood)) {
        max_log_likelihood = std::max(max_log_likelihood, log_likelihood);
      }
    }

    if (!std::isfinite(max_log_likelihood)) {
      const double equal_weight = 1.0 / static_cast<double>(particles_.size());
      for (std::size_t particle_index = 0; particle_index < particles_.size(); ++particle_index) {
        auto & particle = particles_[particle_index];
        particle.weight = equal_weight;
        stats.normalized_scores[particle_index] = equal_weight;
      }
      return stats;
    }

    double weight_sum = 0.0;
    for (std::size_t particle_index = 0; particle_index < particles_.size(); ++particle_index) {
      auto & particle = particles_[particle_index];
      const double log_likelihood = stats.raw_scores[particle_index];
      if (std::isfinite(log_likelihood)) {
        particle.weight = std::exp(log_likelihood - max_log_likelihood);
        weight_sum += particle.weight;
      } else {
        particle.weight = 0.0;
      }
    }

    if (weight_sum <= 0.0 || !std::isfinite(weight_sum)) {
      const double equal_weight = 1.0 / static_cast<double>(particles_.size());
      for (std::size_t particle_index = 0; particle_index < particles_.size(); ++particle_index) {
        auto & particle = particles_[particle_index];
        particle.weight = equal_weight;
        stats.normalized_scores[particle_index] = equal_weight;
      }
      return stats;
    }

    for (std::size_t particle_index = 0; particle_index < particles_.size(); ++particle_index) {
      auto & particle = particles_[particle_index];
      particle.weight /= weight_sum;
      stats.normalized_scores[particle_index] = particle.weight;
    }

    return stats;
  }

  std::size_t bestParticleIndex() const
  {
    return static_cast<std::size_t>(std::distance(
      particles_.begin(),
      std::max_element(
        particles_.begin(),
        particles_.end(),
        [](const Particle & lhs, const Particle & rhs) { return lhs.weight < rhs.weight; })));
  }

  ParticleStats computeParticleStats() const
  {
    ParticleStats stats;
    if (particles_.empty()) {
      return stats;
    }

    stats.raw_scores.reserve(particles_.size());
    stats.normalized_scores.reserve(particles_.size());
    stats.min_score = std::numeric_limits<double>::max();
    double weight_square_sum = 0.0;

    for (const auto & particle : particles_) {
      stats.best_score = std::max(stats.best_score, particle.weight);
      stats.min_score = std::min(stats.min_score, particle.weight);
      stats.average_score += particle.weight;
      weight_square_sum += particle.weight * particle.weight;
      stats.normalized_scores.push_back(particle.weight);
    }

    stats.average_score /= static_cast<double>(particles_.size());
    if (stats.min_score == std::numeric_limits<double>::max()) {
      stats.min_score = 0.0;
    }
    if (weight_square_sum > 0.0 && std::isfinite(weight_square_sum)) {
      stats.effective_particle_count = 1.0 / weight_square_sum;
    }

    return stats;
  }

  bool shouldResample(double effective_particle_count) const
  {
    if (effective_particle_count <= 0.0 || !std::isfinite(effective_particle_count)) {
      return false;
    }
    return effective_particle_count < (0.5 * static_cast<double>(particles_.size()));
  }

  void resampleParticles()
  {
    double weight_sum = 0.0;
    for (const auto & particle : particles_) {
      weight_sum += particle.weight;
    }

    if (weight_sum <= 0.0 || !std::isfinite(weight_sum)) {
      const double equal_weight = 1.0 / static_cast<double>(particles_.size());
      for (auto & particle : particles_) {
        particle.weight = equal_weight;
      }
      weight_sum = 1.0;
    }

    std::vector<double> cumulative_weights;
    cumulative_weights.reserve(particles_.size());
    double cumulative_sum = 0.0;
    for (auto & particle : particles_) {
      particle.weight /= weight_sum;
      cumulative_sum += particle.weight;
      cumulative_weights.push_back(cumulative_sum);
    }

    cumulative_weights.back() = 1.0;

    std::uniform_real_distribution<double> start_distribution(
      0.0, 1.0 / static_cast<double>(particles_.size()));

    std::vector<Particle> new_particles;
    new_particles.reserve(particles_.size());

    double pointer = start_distribution(rng_);
    const double step = 1.0 / static_cast<double>(particles_.size());
    std::size_t particle_index = 0;

    for (std::size_t i = 0; i < particles_.size(); ++i) {
      while (pointer > cumulative_weights[particle_index]) {
        particle_index++;
      }

      Particle copied = particles_[particle_index];
      copied.weight = 1.0 / static_cast<double>(particles_.size());
      new_particles.push_back(std::move(copied));
      pointer += step;
    }

    particles_ = std::move(new_particles);
  }

  void publishState(
    std::size_t best_index,
    const builtin_interfaces::msg::Time & stamp,
    const Pose2D & odom_pose_at_stamp)
  {
    const Particle & best_particle = particles_[best_index];
    if (best_particle.has_map) {
      map_pub_->publish(best_particle.map_msg);
    }

    geometry_msgs::msg::PoseArray particle_cloud;
    particle_cloud.header.frame_id = "map";
    particle_cloud.header.stamp = stamp;
    particle_cloud.poses.reserve(particles_.size());
    for (const auto & particle : particles_) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = particle.pose.x;
      pose.position.y = particle.pose.y;
      pose.orientation = yawToQuaternion(particle.pose.theta);
      particle_cloud.poses.push_back(pose);
    }
    particle_pub_->publish(particle_cloud);

    nav_msgs::msg::Path best_path;
    best_path.header.frame_id = "map";
    best_path.header.stamp = stamp;
    best_path.poses.reserve(best_particle.trajectory.size());
    for (const auto & trajectory_pose : best_particle.trajectory) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = "map";
      pose_stamped.header.stamp = trajectory_pose.stamp;
      pose_stamped.pose.position.x = trajectory_pose.pose.x;
      pose_stamped.pose.position.y = trajectory_pose.pose.y;
      pose_stamped.pose.orientation = yawToQuaternion(trajectory_pose.pose.theta);
      best_path.poses.push_back(pose_stamped);
    }
    best_path_pub_->publish(best_path);

    geometry_msgs::msg::PoseStamped estimated_pose;
    estimated_pose.header.frame_id = "map";
    estimated_pose.header.stamp = stamp;
    estimated_pose.pose.position.x = best_particle.pose.x;
    estimated_pose.pose.position.y = best_particle.pose.y;
    estimated_pose.pose.orientation = yawToQuaternion(best_particle.pose.theta);
    estimated_pose_pub_->publish(estimated_pose);

    publishMapToOdomTf(best_particle.pose, odom_pose_at_stamp, stamp);
  }

  void publishMapToOdomTf(
    const Pose2D & slam_pose,
    const Pose2D & odom_pose,
    const builtin_interfaces::msg::Time & stamp)
  {
    tf2::Quaternion map_to_base_rotation;
    map_to_base_rotation.setRPY(0.0, 0.0, slam_pose.theta);

    tf2::Transform map_to_base;
    map_to_base.setOrigin(tf2::Vector3(slam_pose.x, slam_pose.y, 0.0));
    map_to_base.setRotation(map_to_base_rotation);

    tf2::Quaternion odom_to_base_rotation;
    odom_to_base_rotation.setRPY(0.0, 0.0, odom_pose.theta);

    tf2::Transform odom_to_base;
    odom_to_base.setOrigin(tf2::Vector3(odom_pose.x, odom_pose.y, 0.0));
    odom_to_base.setRotation(odom_to_base_rotation);

    tf2::Transform map_to_odom = map_to_base * odom_to_base.inverse();

    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "map";
    msg.child_frame_id = "odom";
    msg.transform = tf2::toMsg(map_to_odom);
    tf_broadcaster_->sendTransform(msg);
  }

  Settings settings_{};
  std::default_random_engine rng_{42U};

  OccupancyMapper::Config map_config_{};

  std::vector<Particle> particles_{};

  std::deque<OdomSample> odom_buffer_{};
  Pose2D last_accepted_odom_pose_{};
  bool have_odom_{false};
  bool have_last_accepted_odom_{false};
  std::size_t accepted_update_count_{0};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr best_path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr estimated_pose_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FastSlamNode>());
  rclcpp::shutdown();
  return 0;
}
