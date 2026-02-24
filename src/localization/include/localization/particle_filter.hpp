#pragma once

#include <vector>
#include <cmath>
#include "nav_msgs/msg/occupancy_grid.hpp"

struct Particle {
    double x;
    double y;
    double theta;
    double weight;
};

class ParticleFilter
{
public:
    ParticleFilter(int num_particles);

    void initUniform(const nav_msgs::msg::OccupancyGrid& map);

    void sampleMotionModel(double old_x, double old_y, double old_theta,
                            double new_x, double new_y, double new_theta);
    
    const std::vector<Particle> & getParticles() const;

private:
    int num_particles_;
    std::vector<Particle> particles_;
};