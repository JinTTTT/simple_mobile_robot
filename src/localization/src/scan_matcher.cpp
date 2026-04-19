#include "localization/scan_matcher.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

ScanMatcher::ScanMatcher(const ScanMatcherParameters & parameters)
: parameters_(parameters)
{
}

void ScanMatcher::setMap(const nav_msgs::msg::OccupancyGrid & map)
{
    buildLikelihoodField(map);
}

bool ScanMatcher::hasMap() const
{
    return !likelihood_field_map_.data.empty();
}

ScanMatchResult ScanMatcher::match(
    const sensor_msgs::msg::LaserScan & scan,
    const ScanMatchPose & initial_guess) const
{
    ScanMatchResult result;
    result.pose = initial_guess;
    result.score = 0.0;
    result.valid = false;

    if (!hasMap() || scan.ranges.empty()) {
        return result;
    }

    double best_score = -std::numeric_limits<double>::infinity();
    ScanMatchPose best_pose = initial_guess;

    for (double dx = -parameters_.search_xy_range;
        dx <= parameters_.search_xy_range + 1e-9;
        dx += parameters_.search_xy_step)
    {
        for (double dy = -parameters_.search_xy_range;
            dy <= parameters_.search_xy_range + 1e-9;
            dy += parameters_.search_xy_step)
        {
            for (double dtheta = -parameters_.search_theta_range;
                dtheta <= parameters_.search_theta_range + 1e-9;
                dtheta += parameters_.search_theta_step)
            {
                ScanMatchPose candidate;
                candidate.x = initial_guess.x + dx;
                candidate.y = initial_guess.y + dy;
                candidate.theta = normalizeAngle(initial_guess.theta + dtheta);

                double score = scoreScanAtPose(scan, candidate);
                if (score > best_score) {
                    best_score = score;
                    best_pose = candidate;
                }
            }
        }
    }

    if (std::isfinite(best_score)) {
        result.pose = best_pose;
        result.score = best_score;
        result.valid = true;
    }

    return result;
}

const nav_msgs::msg::OccupancyGrid & ScanMatcher::likelihoodFieldMap() const
{
    return likelihood_field_map_;
}

void ScanMatcher::buildLikelihoodField(const nav_msgs::msg::OccupancyGrid & map)
{
    likelihood_field_map_ = map;

    int width = map.info.width;
    int height = map.info.height;
    int total_cells = width * height;
    double resolution = map.info.resolution;

    int max_distance_cells =
        static_cast<int>(std::ceil(parameters_.likelihood_max_distance / resolution));

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
        double likelihood =
            1.0 - std::min(distance_m / parameters_.likelihood_max_distance, 1.0);

        likelihood_field_map_.data[i] =
            static_cast<int8_t>(std::round(likelihood * 100.0));
    }
}

double ScanMatcher::scoreScanAtPose(
    const sensor_msgs::msg::LaserScan & scan,
    const ScanMatchPose & pose) const
{
    double score = 0.0;
    int used_beams = 0;

    for (std::size_t i = 0; i < scan.ranges.size(); i += parameters_.beam_step) {
        float range = scan.ranges[i];

        if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
            continue;
        }

        double beam_angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
        double hit_x = pose.x + range * std::cos(pose.theta + beam_angle);
        double hit_y = pose.y + range * std::sin(pose.theta + beam_angle);

        score += likelihoodAtWorld(hit_x, hit_y);
        used_beams++;
    }

    if (used_beams == 0) {
        return 0.0;
    }

    return score / used_beams;
}

bool ScanMatcher::worldToLikelihoodMap(double x, double y, int & col, int & row) const
{
    if (!hasMap()) {
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

double ScanMatcher::likelihoodAtWorld(double x, double y) const
{
    int col = 0;
    int row = 0;
    if (!worldToLikelihoodMap(x, y, col, row)) {
        return 0.0;
    }

    int index = row * likelihood_field_map_.info.width + col;
    return likelihood_field_map_.data[index] / 100.0;
}

double ScanMatcher::normalizeAngle(double angle) const
{
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}
