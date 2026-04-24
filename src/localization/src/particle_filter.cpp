#include "localization/particle_filter.hpp"
#include "localization/geometry_utils.hpp"

#include <algorithm>
#include <limits>
#include <random>
#include <cmath>

ParticleFilter::ParticleFilter(const ParticleFilterParameters & parameters)
{
    configure(parameters);
}

void ParticleFilter::configure(const ParticleFilterParameters & parameters)
{
    parameters_ = parameters;
    rng_.seed(parameters_.random_seed);
    particles_.resize(parameters_.num_particles);
}

void ParticleFilter::initializeUniform(const nav_msgs::msg::OccupancyGrid & map)
{
    // Extract map properties
    double res    = map.info.resolution;
    int    width  = map.info.width;
    int    height = map.info.height;
    double orig_x = map.info.origin.position.x;
    double orig_y = map.info.origin.position.y;

    // Random number generators
    std::uniform_int_distribution<int> rand_col(0, width - 1);
    std::uniform_int_distribution<int> rand_row(0, height - 1);
    std::uniform_real_distribution<double> rand_theta(-M_PI, M_PI);

    int placed = 0;
    while (placed < parameters_.num_particles) {
        int col = rand_col(rng_);
        int row = rand_row(rng_);

        // map.data is a flat array: index = row * width + col
        // value 0 = free, 100 = occupied, -1 = unknown
        int index = row * width + col;
        if (map.data[index] != 0) {
            continue;  // skip occupied and unknown cells
        }

        // Convert grid cell back to world coordinates
        particles_[placed].x     = orig_x + (col + 0.5) * res;
        particles_[placed].y     = orig_y + (row + 0.5) * res;
        particles_[placed].theta = rand_theta(rng_);
        particles_[placed].weight = 1.0 / parameters_.num_particles;
        placed++;
    }
}

void ParticleFilter::buildLikelihoodField(const nav_msgs::msg::OccupancyGrid & map)
{
    likelihood_field_.build(map, parameters_.likelihood_max_distance);
}

void ParticleFilter::sampleMotionModel(
    double old_x, double old_y, double old_theta,
    double new_x, double new_y, double new_theta)
{
    // Decompose odometry into 3 primitive moves:
    // 1. rotate to face the direction of travel
    // 2. translate forward
    // 3. rotate to final heading
    double delta_rot1  = std::atan2(new_y - old_y, new_x - old_x) - old_theta;
    double delta_trans = std::sqrt(std::pow(new_x - old_x, 2) + std::pow(new_y - old_y, 2));
    double delta_rot2  = new_theta - old_theta - delta_rot1;

    double trans_noise_std =
        parameters_.translation_noise_from_translation * delta_trans +
        parameters_.translation_noise_base;
    double rot1_noise_std =
        parameters_.rotation_noise_from_rotation * std::abs(delta_rot1) +
        parameters_.rotation_noise_from_translation * delta_trans +
        parameters_.rotation_noise_base;
    double rot2_noise_std =
        parameters_.rotation_noise_from_rotation * std::abs(delta_rot2) +
        parameters_.rotation_noise_from_translation * delta_trans +
        parameters_.rotation_noise_base;

    std::normal_distribution<double> trans_noise(0.0, trans_noise_std);
    std::normal_distribution<double> rot1_noise(0.0, rot1_noise_std);
    std::normal_distribution<double> rot2_noise(0.0, rot2_noise_std);

    for (auto & p : particles_) {
        double noisy_rot1 = delta_rot1 + rot1_noise(rng_);
        double noisy_trans = delta_trans + trans_noise(rng_);
        double noisy_rot2 = delta_rot2 + rot2_noise(rng_);

        p.x     += noisy_trans * std::cos(p.theta + noisy_rot1);
        p.y     += noisy_trans * std::sin(p.theta + noisy_rot1);
        p.theta = normalizeAngle(p.theta + noisy_rot1 + noisy_rot2);
    }
}

ScanScoreStats ParticleFilter::scoreParticlesWithScan(const sensor_msgs::msg::LaserScan & scan)
{
    ScanScoreStats stats;
    stats.best_score = 0.0;
    stats.worst_score = std::numeric_limits<double>::max();
    stats.average_score = 0.0;

    if (!likelihood_field_.hasMap()) {
        return stats;
    }

    double score_sum = 0.0;

    for (auto & p : particles_) {
        double particle_score = 0.0;
        int used_beams = 0;

        for (size_t i = 0; i < scan.ranges.size(); i += parameters_.scan_beam_step) {
            float range = scan.ranges[i];

            if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
                continue;
            }

            double beam_angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
            double hit_x = p.x + range * std::cos(p.theta + beam_angle);
            double hit_y = p.y + range * std::sin(p.theta + beam_angle);

            particle_score += likelihood_field_.valueAtWorld(hit_x, hit_y);
            used_beams++;
        }

        if (used_beams > 0) {
            particle_score /= used_beams;
        }

        p.weight = particle_score;
        score_sum += particle_score;
        stats.best_score = std::max(stats.best_score, particle_score);
        stats.worst_score = std::min(stats.worst_score, particle_score);
    }

    stats.average_score = score_sum / particles_.size();

    if (stats.worst_score == std::numeric_limits<double>::max()) {
        stats.worst_score = 0.0;
    }

    return stats;
}

void ParticleFilter::resample()
{
    double weight_sum = 0.0;
    for (const auto & p : particles_) {
        weight_sum += p.weight;
    }

    if (weight_sum <= 0.0 || !std::isfinite(weight_sum)) {
        double equal_weight = 1.0 / parameters_.num_particles;
        for (auto & p : particles_) {
            p.weight = equal_weight;
        }
        return;
    }

    std::vector<double> cumulative_weights;
    cumulative_weights.reserve(particles_.size());
    double cumulative_sum = 0.0;

    for (auto & p : particles_) {
        p.weight /= weight_sum;
        cumulative_sum += p.weight;
        cumulative_weights.push_back(cumulative_sum);
    }

    // Avoid tiny floating-point error, so the final pointer can always find a particle.
    cumulative_weights.back() = 1.0;

    std::normal_distribution<double> xy_noise(0.0, parameters_.resample_xy_noise_std);
    std::normal_distribution<double> theta_noise(0.0, parameters_.resample_theta_noise_std);
    std::uniform_real_distribution<double> start_distribution(
        0.0, 1.0 / parameters_.num_particles);

    std::vector<Particle> new_particles;
    new_particles.reserve(parameters_.num_particles);

    double pointer = start_distribution(rng_);
    double step = 1.0 / parameters_.num_particles;
    size_t particle_index = 0;

    for (int i = 0; i < parameters_.num_particles; ++i) {
        while (pointer > cumulative_weights[particle_index]) {
            particle_index++;
        }

        Particle copied = particles_[particle_index];

        copied.x += xy_noise(rng_);
        copied.y += xy_noise(rng_);
        copied.theta = normalizeAngle(copied.theta + theta_noise(rng_));
        copied.weight = 1.0 / parameters_.num_particles;

        new_particles.push_back(copied);
        pointer += step;
    }

    particles_ = new_particles;
}

EstimatedPose ParticleFilter::estimatePose() const
{
    EstimatedPose pose;
    pose.x = 0.0;
    pose.y = 0.0;
    pose.theta = 0.0;

    if (particles_.empty()) {
        return pose;
    }

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_sin = 0.0;
    double sum_cos = 0.0;

    for (const auto & p : particles_) {
        sum_x += p.x;
        sum_y += p.y;
        sum_sin += std::sin(p.theta);
        sum_cos += std::cos(p.theta);
    }

    pose.x = sum_x / particles_.size();
    pose.y = sum_y / particles_.size();
    pose.theta = std::atan2(sum_sin, sum_cos);

    return pose;
}

const std::vector<Particle> & ParticleFilter::particles() const
{
    return particles_;
}

const nav_msgs::msg::OccupancyGrid & ParticleFilter::likelihoodFieldMap() const
{
    return likelihood_field_.message();
}
