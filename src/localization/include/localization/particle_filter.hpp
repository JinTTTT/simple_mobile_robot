#pragma once

#include <vector>
#include <cmath>
#include <random>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

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
    ParticleFilter(int num_particles);

    void initUniform(const nav_msgs::msg::OccupancyGrid& map);

    void buildLikelihoodField(const nav_msgs::msg::OccupancyGrid& map);

    void sampleMotionModel(double old_x, double old_y, double old_theta,
                            double new_x, double new_y, double new_theta);

    ScanScoreStats scoreParticlesWithScan(const sensor_msgs::msg::LaserScan& scan);

    void resample();

    EstimatedPose estimatePose() const;

    const nav_msgs::msg::OccupancyGrid & getLikelihoodFieldMap() const;
    
    const std::vector<Particle> & getParticles() const;

private:
    bool worldToLikelihoodMap(double x, double y, int & col, int & row) const;
    double likelihoodAtWorld(double x, double y) const;
    double normalizeAngle(double angle) const;

    int num_particles_;
    std::vector<Particle> particles_;
    nav_msgs::msg::OccupancyGrid likelihood_field_map_;
    std::default_random_engine rng_;
};
