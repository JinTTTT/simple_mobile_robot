#pragma once

#include "localization/likelihood_field.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

struct ScanMatcherParameters {
    double search_xy_range = 0.20;
    double search_theta_range = 0.20;
    double search_xy_step = 0.05;
    double search_theta_step = 0.05;
    std::size_t beam_step = 10;
    double likelihood_max_distance = 1.0;
};

struct ScanMatchPose {
    double x;
    double y;
    double theta;
};

struct ScanMatchResult {
    ScanMatchPose pose;
    double score;
    bool valid;
};

class ScanMatcher
{
public:
    explicit ScanMatcher(const ScanMatcherParameters & parameters = ScanMatcherParameters());

    void setMap(const nav_msgs::msg::OccupancyGrid & map);

    bool hasMap() const;

    ScanMatchResult match(
        const sensor_msgs::msg::LaserScan & scan,
        const ScanMatchPose & initial_guess) const;

    const nav_msgs::msg::OccupancyGrid & likelihoodFieldMap() const;

private:
    double scoreScanAtPose(
        const sensor_msgs::msg::LaserScan & scan,
        const ScanMatchPose & pose) const;

    ScanMatcherParameters parameters_;
    LikelihoodField likelihood_field_;
};
