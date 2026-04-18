#include "localization/particle_filter.hpp"
#include <algorithm>
#include <limits>
#include <queue>
#include <random>
#include <cmath>

ParticleFilter::ParticleFilter(int num_particles)
    : num_particles_(num_particles), rng_(42)
{
    particles_.resize(num_particles_);
}

void ParticleFilter::initUniform(const nav_msgs::msg::OccupancyGrid & map)
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
    while (placed < num_particles_) {
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
        particles_[placed].weight = 1.0 / num_particles_;
        placed++;
    }
}

void ParticleFilter::buildLikelihoodField(const nav_msgs::msg::OccupancyGrid & map)
{
    likelihood_field_map_ = map;

    int width = map.info.width;
    int height = map.info.height;
    int total_cells = width * height;
    double resolution = map.info.resolution;

    double max_distance_m = 1.0;
    int max_distance_cells = static_cast<int>(std::ceil(max_distance_m / resolution));

    // Distance starts as "far from wall".
    // Later, wall cells spread smaller distance values to their neighbors.
    std::vector<int> distance_to_wall(total_cells, max_distance_cells);
    std::queue<int> cells_to_visit;

    for (int i = 0; i < total_cells; ++i) {
        if (map.data[i] > 50) {
            distance_to_wall[i] = 0;
            cells_to_visit.push(i);
        }
    }

    const int neighbor_offsets[4][2] = {
        {1, 0},
        {-1, 0},
        {0, 1},
        {0, -1}
    };

    // This is like dropping ink on all wall cells.
    // The ink spreads one cell at a time into nearby free cells.
    while (!cells_to_visit.empty()) {
        int index = cells_to_visit.front();
        cells_to_visit.pop();

        int row = index / width;
        int col = index % width;
        int next_distance = distance_to_wall[index] + 1;

        if (next_distance > max_distance_cells) {
            continue;
        }

        for (const auto & offset : neighbor_offsets) {
            int next_col = col + offset[0];
            int next_row = row + offset[1];

            if (next_col < 0 || next_row < 0 || next_col >= width || next_row >= height) {
                continue;
            }

            int next_index = next_row * width + next_col;
            if (next_distance >= distance_to_wall[next_index]) {
                continue;
            }

            distance_to_wall[next_index] = next_distance;
            cells_to_visit.push(next_index);
        }
    }

    likelihood_field_map_.data.assign(total_cells, 0);

    for (int i = 0; i < total_cells; ++i) {
        double distance_m = distance_to_wall[i] * resolution;
        double likelihood = 1.0 - std::min(distance_m / max_distance_m, 1.0);

        likelihood_field_map_.data[i] =
            static_cast<int8_t>(std::round(likelihood * 100.0));
    }
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

    double trans_noise_std = 0.02 * delta_trans + 0.005;
    double rot1_noise_std = 0.05 * std::abs(delta_rot1) + 0.01 * delta_trans + 0.002;
    double rot2_noise_std = 0.05 * std::abs(delta_rot2) + 0.01 * delta_trans + 0.002;

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

    if (likelihood_field_map_.data.empty()) {
        return stats;
    }

    const size_t beam_step = 10;
    double score_sum = 0.0;

    for (auto & p : particles_) {
        double particle_score = 0.0;
        int used_beams = 0;

        for (size_t i = 0; i < scan.ranges.size(); i += beam_step) {
            float range = scan.ranges[i];

            if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
                continue;
            }

            double beam_angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
            double hit_x = p.x + range * std::cos(p.theta + beam_angle);
            double hit_y = p.y + range * std::sin(p.theta + beam_angle);

            particle_score += likelihoodAtWorld(hit_x, hit_y);
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
        double equal_weight = 1.0 / num_particles_;
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

    std::normal_distribution<double> xy_noise(0.0, 0.02);
    std::normal_distribution<double> theta_noise(0.0, 0.03);
    std::uniform_real_distribution<double> start_distribution(0.0, 1.0 / num_particles_);

    std::vector<Particle> new_particles;
    new_particles.reserve(num_particles_);

    double pointer = start_distribution(rng_);
    double step = 1.0 / num_particles_;
    size_t particle_index = 0;

    for (int i = 0; i < num_particles_; ++i) {
        while (pointer > cumulative_weights[particle_index]) {
            particle_index++;
        }

        Particle copied = particles_[particle_index];

        copied.x += xy_noise(rng_);
        copied.y += xy_noise(rng_);
        copied.theta = normalizeAngle(copied.theta + theta_noise(rng_));
        copied.weight = 1.0 / num_particles_;

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

const std::vector<Particle> & ParticleFilter::getParticles() const
{
    return particles_;
}

const nav_msgs::msg::OccupancyGrid & ParticleFilter::getLikelihoodFieldMap() const
{
    return likelihood_field_map_;
}

bool ParticleFilter::worldToLikelihoodMap(double x, double y, int & col, int & row) const
{
    if (likelihood_field_map_.data.empty()) {
        return false;
    }

    double resolution = likelihood_field_map_.info.resolution;
    double origin_x = likelihood_field_map_.info.origin.position.x;
    double origin_y = likelihood_field_map_.info.origin.position.y;

    col = static_cast<int>(std::floor((x - origin_x) / resolution));
    row = static_cast<int>(std::floor((y - origin_y) / resolution));

    return col >= 0 &&
        row >= 0 &&
        col < static_cast<int>(likelihood_field_map_.info.width) &&
        row < static_cast<int>(likelihood_field_map_.info.height);
}

double ParticleFilter::likelihoodAtWorld(double x, double y) const
{
    int col = 0;
    int row = 0;
    if (!worldToLikelihoodMap(x, y, col, row)) {
        return 0.0;
    }

    int index = row * likelihood_field_map_.info.width + col;
    return likelihood_field_map_.data[index] / 100.0;
}

double ParticleFilter::normalizeAngle(double angle) const
{
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}
