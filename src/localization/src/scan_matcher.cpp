#include "localization/scan_matcher.hpp"
#include "localization/geometry_utils.hpp"

#include <cmath>
#include <limits>

ScanMatcher::ScanMatcher(const ScanMatcherParameters & parameters)
: parameters_(parameters)
{
}

void ScanMatcher::setMap(const nav_msgs::msg::OccupancyGrid & map)
{
    likelihood_field_.build(map, parameters_.likelihood_max_distance);
}

bool ScanMatcher::hasMap() const
{
    return likelihood_field_.hasMap();
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
    return likelihood_field_.message();
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

        score += likelihood_field_.valueAtWorld(hit_x, hit_y);
        used_beams++;
    }

    if (used_beams == 0) {
        return 0.0;
    }

    return score / used_beams;
}
