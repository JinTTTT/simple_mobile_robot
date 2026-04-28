#include "slam_fastslam/fastslam.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace slam_fastslam
{

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

FastSlam::FastSlam()
{
  configure(parameters_, map_config_);
}

FastSlam::FastSlam(
  const FastSlamParameters & parameters,
  const OccupancyMapper::Config & map_config)
{
  configure(parameters, map_config);
}

void FastSlam::configure(
  const FastSlamParameters & parameters,
  const OccupancyMapper::Config & map_config)
{
  parameters_ = parameters;
  map_config_ = map_config;
  ScanScorerOptions scorer_options;
  scorer_options.scan_beam_step = parameters_.scan_beam_step;
  scorer_options.ray_occupied_threshold = parameters_.ray_occupied_threshold;
  scorer_options.ray_occupied_crossing_penalty = parameters_.ray_occupied_crossing_penalty;
  scorer_options.ray_penalty_max_per_beam = parameters_.ray_penalty_max_per_beam;
  scorer_options.ray_endpoint_margin_cells = parameters_.ray_endpoint_margin_cells;
  scorer_options.free_space_reward_per_cell = parameters_.free_space_reward_per_cell;
  scan_scorer_.configure(scorer_options);
  initializeParticles();
}

void FastSlam::setLaserOffset(const Pose2D & laser_offset)
{
  laser_offset_ = laser_offset;
}

const FastSlamParameters & FastSlam::parameters() const
{
  return parameters_;
}

const std::vector<FastSlamParticle> & FastSlam::particles() const
{
  return particles_;
}

FastSlamUpdateResult FastSlam::update(
  const sensor_msgs::msg::LaserScan & scan,
  const Pose2D & previous_odom_pose,
  const Pose2D & current_odom_pose)
{
  FastSlamUpdateResult result;
  if (particles_.empty()) {
    return result;
  }

  propagateParticles(previous_odom_pose, current_odom_pose);

  const std::vector<CachedScanBeam> scoring_beams = scan_scorer_.buildScoringBeams(scan);

  if (all_particles_have_map_) {
    result.stats = scoreParticles(scoring_beams);
  } else {
    const double equal_weight = 1.0 / static_cast<double>(particles_.size());
    for (auto & particle : particles_) {
      particle.weight = equal_weight;
    }
    result.stats.effective_particle_count = static_cast<double>(particles_.size());
  }

  const std::size_t best_index = bestParticleIndex();
  result.best_weight = particles_[best_index].weight;
  result.best_log_likelihood = particles_[best_index].log_likelihood;

  const std::size_t pre_select_id = published_particle_id_;
  std::size_t published_index = selectPublishedParticleIndex(best_index);
  result.particle_switched = (published_particle_id_ != pre_select_id);

  result.resampled = shouldResample(result.stats.effective_particle_count);
  if (result.resampled) {
    const std::size_t pre_resample_id = particles_[published_index].id;
    resampleParticles(pre_resample_id);
    const std::size_t post_resample_index = findParticleById(pre_resample_id);
    published_index =
      (post_resample_index < particles_.size()) ? post_resample_index : bestParticleIndex();
    published_particle_id_ = particles_[published_index].id;
  }

  std::vector<int> newly_occupied;
  std::vector<int> newly_freed;

  for (auto & particle : particles_) {
    TrajectoryPose trajectory_pose;
    trajectory_pose.pose = particle.pose;
    trajectory_pose.stamp = scan.header.stamp;
    particle.trajectory.push_back(trajectory_pose);
    if (particle.trajectory.size() > parameters_.traj_max_poses) {
      particle.trajectory.pop_front();
    }

    const double cos_p = std::cos(particle.pose.theta);
    const double sin_p = std::sin(particle.pose.theta);
    const double map_laser_x =
      particle.pose.x + laser_offset_.x * cos_p - laser_offset_.y * sin_p;
    const double map_laser_y =
      particle.pose.y + laser_offset_.x * sin_p + laser_offset_.y * cos_p;
    const double map_laser_theta = particle.pose.theta + laser_offset_.theta;
    particle.mapper.updateWithScan(scan, map_laser_x, map_laser_y, map_laser_theta);

    particle.mapper.getAndClearChanges(newly_occupied, newly_freed);
    particle.likelihood_field.incrementalUpdate(
      particle.mapper, parameters_.likelihood_max_distance, parameters_.likelihood_sigma,
      newly_occupied, newly_freed, parameters_.freed_cells_rebuild_threshold);
    particle.has_map = true;
  }

  all_particles_have_map_ = true;
  result.published_index = published_index;
  result.updated = true;
  return result;
}

void FastSlam::initializeParticles()
{
  particles_.assign(static_cast<std::size_t>(parameters_.num_particles), FastSlamParticle{});
  const double initial_weight = 1.0 / static_cast<double>(parameters_.num_particles);
  next_particle_id_ = 0U;
  published_particle_id_ = kInvalidParticleId;
  for (auto & particle : particles_) {
    particle.id = next_particle_id_++;
    particle.pose = Pose2D{};
    particle.weight = initial_weight;
    particle.trajectory.clear();
    particle.mapper.configure(map_config_);
    particle.has_map = false;
  }
}

bool FastSlam::shouldAcceptUpdate(
  const Pose2D & previous_odom_pose,
  const Pose2D & current_odom_pose) const
{
  const double dx = current_odom_pose.x - previous_odom_pose.x;
  const double dy = current_odom_pose.y - previous_odom_pose.y;
  const double translation = std::hypot(dx, dy);
  const double rotation = std::abs(
    normalizeAngle(current_odom_pose.theta - previous_odom_pose.theta));

  return translation >= parameters_.min_translation_for_update ||
         rotation >= parameters_.min_rotation_for_update;
}

void FastSlam::propagateParticles(const Pose2D & old_pose, const Pose2D & new_pose)
{
  double delta_rot1 = 0.0;
  const double delta_x = new_pose.x - old_pose.x;
  const double delta_y = new_pose.y - old_pose.y;
  const double delta_trans = std::hypot(delta_x, delta_y);
  if (delta_trans > 1e-6) {
    delta_rot1 = normalizeAngle(std::atan2(delta_y, delta_x) - old_pose.theta);
  }
  const double delta_rot2 = normalizeAngle(new_pose.theta - old_pose.theta - delta_rot1);

  const double total_rotation = std::abs(delta_rot1) + std::abs(delta_rot2);
  const double trans_noise_std =
    parameters_.translation_noise_from_translation * delta_trans +
    parameters_.translation_noise_from_rotation * total_rotation +
    parameters_.translation_noise_base;
  const double rot1_noise_std =
    parameters_.rotation_noise_from_rotation * std::abs(delta_rot1) +
    parameters_.rotation_noise_from_translation * delta_trans +
    parameters_.rotation_noise_base;
  const double rot2_noise_std =
    parameters_.rotation_noise_from_rotation * std::abs(delta_rot2) +
    parameters_.rotation_noise_from_translation * delta_trans +
    parameters_.rotation_noise_base;

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

FastSlamParticleStats FastSlam::scoreParticles(
  const std::vector<CachedScanBeam> & scoring_beams)
{
  FastSlamParticleStats stats;
  stats.raw_scores.resize(particles_.size(), 0.0);
  stats.normalized_scores.resize(particles_.size(), 0.0);
  const double invalid_log_likelihood = -std::numeric_limits<double>::infinity();
  double max_log_likelihood = invalid_log_likelihood;
  for (std::size_t particle_index = 0; particle_index < particles_.size(); ++particle_index) {
    auto & particle = particles_[particle_index];
    if (!particle.has_map || !particle.likelihood_field.hasMap()) {
      particle.weight = 0.0;
      particle.log_likelihood = invalid_log_likelihood;
      stats.raw_scores[particle_index] = invalid_log_likelihood;
      continue;
    }

    const double log_likelihood = scan_scorer_.scoreParticle(
      scoring_beams,
      particle.pose,
      laser_offset_,
      particle.mapper,
      particle.likelihood_field);
    particle.weight = 0.0;
    particle.log_likelihood = log_likelihood;
    stats.raw_scores[particle_index] = log_likelihood;
    if (std::isfinite(log_likelihood)) {
      max_log_likelihood = std::max(max_log_likelihood, log_likelihood);
    }
  }

  // Helper: compute N_eff = 1/sum(w^2) from current particle weights.
  auto computeNeff = [&]() {
      double w2 = 0.0;
      for (const auto & p : particles_) {
        w2 += p.weight * p.weight;
      }
      return (w2 > 0.0 && std::isfinite(w2)) ? 1.0 / w2 : 0.0;
    };

  if (!std::isfinite(max_log_likelihood)) {
    const double equal_weight = 1.0 / static_cast<double>(particles_.size());
    for (std::size_t particle_index = 0; particle_index < particles_.size(); ++particle_index) {
      auto & particle = particles_[particle_index];
      particle.weight = equal_weight;
      stats.normalized_scores[particle_index] = equal_weight;
    }
    stats.effective_particle_count = static_cast<double>(particles_.size());
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
    stats.effective_particle_count = static_cast<double>(particles_.size());
    return stats;
  }

  for (std::size_t particle_index = 0; particle_index < particles_.size(); ++particle_index) {
    auto & particle = particles_[particle_index];
    particle.weight /= weight_sum;
    stats.normalized_scores[particle_index] = particle.weight;
  }

  stats.effective_particle_count = computeNeff();
  return stats;
}

std::size_t FastSlam::bestParticleIndex() const
{
  return static_cast<std::size_t>(std::distance(
           particles_.begin(),
           std::max_element(
             particles_.begin(),
             particles_.end(),
             [](const FastSlamParticle & lhs, const FastSlamParticle & rhs) {
               return lhs.weight < rhs.weight;
             })));
}

std::size_t FastSlam::findParticleById(std::size_t id) const
{
  const auto iterator = std::find_if(
    particles_.begin(),
    particles_.end(),
    [id](const FastSlamParticle & particle) {return particle.id == id;});
  if (iterator == particles_.end()) {
    return particles_.size();
  }

  return static_cast<std::size_t>(std::distance(particles_.begin(), iterator));
}

std::size_t FastSlam::selectPublishedParticleIndex(std::size_t best_index)
{
  if (particles_.empty()) {
    return 0U;
  }

  const std::size_t current_index = findParticleById(published_particle_id_);
  if (current_index >= particles_.size() || !particles_[current_index].has_map) {
    published_particle_id_ = particles_[best_index].id;
    leading_particle_id_ = kInvalidParticleId;
    leading_wins_ = 0;
    return best_index;
  }

  // Published particle is still best: reset the challenger streak.
  if (best_index == current_index) {
    leading_particle_id_ = kInvalidParticleId;
    leading_wins_ = 0;
    return current_index;
  }

  // A different particle is ahead: track its consecutive-win streak.
  const std::size_t best_id = particles_[best_index].id;
  if (best_id == leading_particle_id_) {
    ++leading_wins_;
  } else {
    leading_particle_id_ = best_id;
    leading_wins_ = 1;
  }

  // Switch only when streak AND weight-ratio thresholds are both met.
  const double published_weight = particles_[current_index].weight;
  const double best_weight = particles_[best_index].weight;
  if (leading_wins_ >= parameters_.consecutive_wins_to_switch &&
    best_weight >= parameters_.switch_weight_ratio * published_weight)
  {
    published_particle_id_ = best_id;
    leading_particle_id_ = kInvalidParticleId;
    leading_wins_ = 0;
    return best_index;
  }

  return current_index;
}

bool FastSlam::shouldResample(double effective_particle_count) const
{
  if (effective_particle_count <= 0.0 || !std::isfinite(effective_particle_count)) {
    return false;
  }
  return effective_particle_count <
         (parameters_.resample_min_eff_ratio * static_cast<double>(particles_.size()));
}

void FastSlam::resampleParticles(std::size_t preserved_particle_id)
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

  std::vector<FastSlamParticle> new_particles;
  new_particles.reserve(particles_.size());
  bool published_id_preserved = false;

  double pointer = start_distribution(rng_);
  const double step = 1.0 / static_cast<double>(particles_.size());
  std::size_t particle_index = 0;

  for (std::size_t i = 0; i < particles_.size(); ++i) {
    while (pointer > cumulative_weights[particle_index]) {
      particle_index++;
    }

    FastSlamParticle copied = particles_[particle_index];
    if (copied.id == preserved_particle_id && !published_id_preserved) {
      published_id_preserved = true;
    } else {
      copied.id = next_particle_id_++;
    }
    copied.weight = 1.0 / static_cast<double>(particles_.size());
    new_particles.push_back(std::move(copied));
    pointer += step;
  }

  particles_ = std::move(new_particles);
}

}  // namespace slam_fastslam
