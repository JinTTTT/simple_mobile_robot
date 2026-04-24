#pragma once

#include "nav_msgs/msg/occupancy_grid.hpp"

class LikelihoodField
{
public:
    void build(
        const nav_msgs::msg::OccupancyGrid & map,
        double max_distance_m,
        int occupied_threshold = 50);

    bool hasMap() const;
    bool worldToGrid(double x, double y, int & col, int & row) const;
    double valueAtWorld(double x, double y) const;

    const nav_msgs::msg::OccupancyGrid & message() const;

private:
    nav_msgs::msg::OccupancyGrid field_map_;
};

