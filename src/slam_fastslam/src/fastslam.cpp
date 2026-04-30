#include "slam_fastslam/fastslam.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "slam_fastslam/utils.hpp"

namespace slam_fastslam
{

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

  MotionModelOptions motion_options;
  motion_options.translation_noise_from_translation =
    parameters_.translation_noise_from_translation;
  motion_options.translation_noise_from_rotation = parameters_.translation_noise_from_rotation;
  motion_options.translation_noise_base = parameters_.translation_noise_base;
  motion_options.rotation_noise_from_rotation = parameters_.rotation_noise_from_rotation;
  motion_options.rotation_noise_from_translation = parameters_.rotation_noise_from_translation;
  motion_options.rotation_noise_base = parameters_.rotation_noise_base;
  motion_model_.configure(motion_options);

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

  motion_model_.propagate(particles_, previous_odom_pose, current_odom_pose, rng_);

  const std::vector<CachedScanBeam> scoring_beams = scan_scorer_.buildScoringBeams(scan);

  if (particles_have_maps_) {
    result.stats = scoreParticles(scoring_beams);
  } else {
    const double equal_weight = 1.0 / static_cast<double>(particles_.size());
    for (auto & particle : particles_) {
      particle.weight = equal_weight;
    }
    result.stats.effective_particle_count = static_cast<double>(particles_.size());
  }

  const std::size_t highest_weight_index = highestWeightParticleIndex();
  result.highest_weight = particles_[highest_weight_index].weight;
  result.highest_log_likelihood = particles_[highest_weight_index].log_likelihood;

  const std::size_t previous_selected_particle_id = selected_particle_id_;
  std::size_t selected_particle_index = selectOutputParticleIndex(highest_weight_index);
  result.selected_particle_changed = (selected_particle_id_ != previous_selected_particle_id);

  result.resampled = particle_resampler_.shouldResample(
    result.stats.effective_particle_count,
    particles_.size(),
    parameters_.resample_min_eff_ratio);
  if (result.resampled) {
    const std::size_t pre_resample_id = particles_[selected_particle_index].id;
    particle_resampler_.resample(particles_, pre_resample_id, next_particle_id_, rng_);
    const std::size_t post_resample_index = findParticleById(pre_resample_id);
    // Keep publishing the same selected hypothesis if resampling preserved it.
    selected_particle_index =
      (post_resample_index <
      particles_.size()) ? post_resample_index : highestWeightParticleIndex();
    selected_particle_id_ = particles_[selected_particle_index].id;
  }

  // Every particle carries its own map hypothesis; update each map from the
  // same scan transformed through that particle's pose.
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

    const auto changes = particle.mapper.takeAndClearMapChanges();
    particle.likelihood_field.incrementalUpdate(
      particle.mapper, parameters_.likelihood_max_distance, parameters_.likelihood_sigma,
      changes.newly_occupied, changes.newly_freed, parameters_.freed_cells_rebuild_threshold);
    particle.has_map = true;
  }

  particles_have_maps_ = true;
  result.selected_particle_index = selected_particle_index;
  result.updated = true;
  return result;
}

void FastSlam::initializeParticles()
{
  particles_.assign(static_cast<std::size_t>(parameters_.num_particles), FastSlamParticle{});
  const double initial_weight = 1.0 / static_cast<double>(parameters_.num_particles);
  next_particle_id_ = 0U;
  selected_particle_id_ = kInvalidParticleId;
  particles_have_maps_ = false;
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

FastSlamParticleStats FastSlam::scoreParticles(
  const std::vector<CachedScanBeam> & scoring_beams)
{
  FastSlamParticleStats stats;
  // Scratch buffer for log-likelihoods so the normalization loop doesn't
  // re-invoke the scorer. Not exposed on the result struct.
  std::vector<double> raw_scores(particles_.size(), 0.0);
  const double invalid_ll = -std::numeric_limits<double>::infinity();
  double max_log_likelihood = invalid_ll;

  for (std::size_t i = 0; i < particles_.size(); ++i) {
    auto & particle = particles_[i];
    if (!particle.has_map || !particle.likelihood_field.hasMap()) {
      particle.weight = 0.0;
      particle.log_likelihood = invalid_ll;
      raw_scores[i] = invalid_ll;
      continue;
    }
    const double ll = scan_scorer_.scoreParticle(
      scoring_beams, particle.pose, laser_offset_, particle.mapper, particle.likelihood_field);
    particle.weight = 0.0;
    particle.log_likelihood = ll;
    raw_scores[i] = ll;
    if (std::isfinite(ll)) {
      max_log_likelihood = std::max(max_log_likelihood, ll);
    }
  }

  // Aggregate statistics from the raw log-likelihoods.
  {
    double min_ll = std::numeric_limits<double>::infinity();
    double sum_ll = 0.0;
    int finite_count = 0;
    for (const double s : raw_scores) {
      if (std::isfinite(s)) {
        min_ll = std::min(min_ll, s);
        sum_ll += s;
        ++finite_count;
      }
    }
    const double no_val = invalid_ll;
    stats.best_score = std::isfinite(max_log_likelihood) ? max_log_likelihood : no_val;
    stats.min_score = (finite_count > 0) ? min_ll : no_val;
    stats.average_score =
      (finite_count > 0) ? sum_ll / static_cast<double>(finite_count) : no_val;
  }

  auto computeNeff = [&]() {
      double w2 = 0.0;
      for (const auto & p : particles_) {
        w2 += p.weight * p.weight;
      }
      return (w2 > 0.0 && std::isfinite(w2)) ? 1.0 / w2 : 0.0;
    };

  if (!std::isfinite(max_log_likelihood)) {
    const double equal_weight = 1.0 / static_cast<double>(particles_.size());
    for (auto & particle : particles_) {
      particle.weight = equal_weight;
    }
    stats.effective_particle_count = static_cast<double>(particles_.size());
    return stats;
  }

  double weight_sum = 0.0;
  for (std::size_t i = 0; i < particles_.size(); ++i) {
    auto & particle = particles_[i];
    if (std::isfinite(raw_scores[i])) {
      particle.weight = std::exp(raw_scores[i] - max_log_likelihood);
      weight_sum += particle.weight;
    } else {
      particle.weight = 0.0;
    }
  }

  if (weight_sum <= 0.0 || !std::isfinite(weight_sum)) {
    const double equal_weight = 1.0 / static_cast<double>(particles_.size());
    for (auto & particle : particles_) {
      particle.weight = equal_weight;
    }
    stats.effective_particle_count = static_cast<double>(particles_.size());
    return stats;
  }

  for (auto & particle : particles_) {
    particle.weight /= weight_sum;
  }

  stats.effective_particle_count = computeNeff();
  return stats;
}

std::size_t FastSlam::highestWeightParticleIndex() const
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

std::size_t FastSlam::selectOutputParticleIndex(std::size_t candidate_index)
{
  if (particles_.empty()) {
    return 0U;
  }

  const std::size_t current_index = findParticleById(selected_particle_id_);
  if (current_index >= particles_.size() || !particles_[current_index].has_map) {
    selected_particle_id_ = particles_[candidate_index].id;
    candidate_particle_id_ = kInvalidParticleId;
    candidate_consecutive_wins_ = 0;
    return candidate_index;
  }

  // The selected output particle is still strongest: reset the challenger streak.
  if (candidate_index == current_index) {
    candidate_particle_id_ = kInvalidParticleId;
    candidate_consecutive_wins_ = 0;
    return current_index;
  }

  // A different particle is ahead: track its consecutive-win streak before switching output.
  const std::size_t candidate_id = particles_[candidate_index].id;
  if (candidate_id == candidate_particle_id_) {
    ++candidate_consecutive_wins_;
  } else {
    candidate_particle_id_ = candidate_id;
    candidate_consecutive_wins_ = 1;
  }

  // Switch only when streak and weight-ratio thresholds are both met.
  const double current_weight = particles_[current_index].weight;
  const double candidate_weight = particles_[candidate_index].weight;
  if (candidate_consecutive_wins_ >= parameters_.consecutive_wins_to_switch &&
    candidate_weight >= parameters_.switch_weight_ratio * current_weight)
  {
    selected_particle_id_ = candidate_id;
    candidate_particle_id_ = kInvalidParticleId;
    candidate_consecutive_wins_ = 0;
    return candidate_index;
  }

  return current_index;
}

}  // namespace slam_fastslam
