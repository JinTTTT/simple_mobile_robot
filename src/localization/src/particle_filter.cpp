#include "localization/particle_filter.hpp"
#include <algorithm>
#include <queue>
#include <random>
#include <cmath>

ParticleFilter::ParticleFilter(int num_particles)
    : num_particles_(num_particles)
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
    std::default_random_engine rng(42);  // 42 = fixed seed, so results are repeatable
    std::uniform_int_distribution<int> rand_col(0, width - 1);
    std::uniform_int_distribution<int> rand_row(0, height - 1);
    std::uniform_real_distribution<double> rand_theta(-M_PI, M_PI);

    int placed = 0;
    while (placed < num_particles_) {
        int col = rand_col(rng);
        int row = rand_row(rng);

        // map.data is a flat array: index = row * width + col
        // value 0 = free, 100 = occupied, -1 = unknown
        int index = row * width + col;
        if (map.data[index] != 0) {
            continue;  // skip occupied and unknown cells
        }

        // Convert grid cell back to world coordinates
        particles_[placed].x     = orig_x + (col + 0.5) * res;
        particles_[placed].y     = orig_y + (row + 0.5) * res;
        particles_[placed].theta = rand_theta(rng);
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

    // Apply the exact same delta to every particle (v1.0: zero noise)
    for (auto & p : particles_) {
        p.x     += delta_trans * std::cos(p.theta + delta_rot1);
        p.y     += delta_trans * std::sin(p.theta + delta_rot1);
        p.theta += delta_rot1 + delta_rot2;
    }
}

const std::vector<Particle> & ParticleFilter::getParticles() const
{
    return particles_;
}

const nav_msgs::msg::OccupancyGrid & ParticleFilter::getLikelihoodFieldMap() const
{
    return likelihood_field_map_;
}
