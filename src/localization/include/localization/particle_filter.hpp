#pragma once

#include "localization/likelihood_field.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <vector>
#include <random>

struct ParticleFilterParameters {
    int num_particles = 500;
    unsigned int random_seed = 42;
    double likelihood_max_distance = 1.0;
    std::size_t scan_beam_step = 10;
    double translation_noise_from_translation = 0.02;
    double translation_noise_base = 0.005;
    double rotation_noise_from_rotation = 0.05;
    double rotation_noise_from_translation = 0.01;
    double rotation_noise_base = 0.002;
    double resample_xy_noise_std = 0.02;
    double resample_theta_noise_std = 0.03;
};

struct Particle {
    double x;
    double y;
    double theta;
    double weight;
};

struct ScanScoreStats {
    double best_score;
    double worst_score;
    double average_score;
};

struct EstimatedPose {
    double x;
    double y;
    double theta;
};

class ParticleFilter
{
public:
    explicit ParticleFilter(
        const ParticleFilterParameters & parameters = ParticleFilterParameters());

    void configure(const ParticleFilterParameters & parameters);

    void initializeUniform(const nav_msgs::msg::OccupancyGrid & map);

    void buildLikelihoodField(const nav_msgs::msg::OccupancyGrid & map);

    void sampleMotionModel(double old_x, double old_y, double old_theta,
                            double new_x, double new_y, double new_theta);

    ScanScoreStats scoreParticlesWithScan(const sensor_msgs::msg::LaserScan& scan);

    void resample();

    EstimatedPose estimatePose() const;

    const nav_msgs::msg::OccupancyGrid & likelihoodFieldMap() const;
    
    const std::vector<Particle> & particles() const;

private:
    ParticleFilterParameters parameters_;
    std::vector<Particle> particles_;
    LikelihoodField likelihood_field_;
    std::default_random_engine rng_;
};
