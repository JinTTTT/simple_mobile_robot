#include "localization/particle_filter.hpp"
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