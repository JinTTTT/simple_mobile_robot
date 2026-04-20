#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

struct Pose2D
{
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

struct ScanMatchResult
{
    Pose2D pose;
    double score = 0.0;
    bool used_scan_matching = false;
};

class SimpleSlamNode : public rclcpp::Node
{
public:
    SimpleSlamNode();

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    void initializeMap();
    void rebuildLikelihoodField();
    ScanMatchResult matchScan(const sensor_msgs::msg::LaserScan & scan, const Pose2D & predicted);
    double scoreScanAtPose(const sensor_msgs::msg::LaserScan & scan, const Pose2D & pose) const;
    double likelihoodAtWorld(double x, double y) const;

    void updateMapWithScan(const sensor_msgs::msg::LaserScan & scan, const Pose2D & pose);
    void publishMap();
    void publishEstimatedPose(const rclcpp::Time & stamp);
    void publishScanMatchedPose(const Pose2D & pose, const rclcpp::Time & stamp);
    void publishMapToOdomTf(const rclcpp::Time & stamp);

    Pose2D odomMsgToPose(const nav_msgs::msg::Odometry & msg) const;
    Pose2D applyOdomDeltaToSlamPose(const Pose2D & old_odom, const Pose2D & new_odom) const;
    bool odomMovedEnough(const Pose2D & old_odom, const Pose2D & new_odom) const;
    geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) const;

    void worldToGrid(double wx, double wy, int & gx, int & gy) const;
    bool isValidCell(int x, int y) const;
    std::vector<std::pair<int, int>> bresenhamLine(int x0, int y0, int x1, int y1) const;
    double normalizeAngle(double angle) const;
    double clamp(double value, double low, double high) const;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr estimated_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr scan_matched_pose_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    nav_msgs::msg::OccupancyGrid map_msg_;
    nav_msgs::msg::OccupancyGrid likelihood_field_msg_;
    std::vector<double> map_log_odds_;

    Pose2D slam_pose_;
    Pose2D last_odom_pose_;
    Pose2D current_odom_pose_;
    bool odom_initialized_ = false;
    bool scan_received_ = false;

    double resolution_ = 0.05;
    int width_ = 500;
    int height_ = 500;
    double origin_x_ = -12.5;
    double origin_y_ = -12.5;

    double log_odds_hit_ = 2.89;
    double log_odds_free_ = -2.25;
    double log_odds_min_ = -10.0;
    double log_odds_max_ = 10.0;

    int occupied_cell_count_ = 0;
    int min_occupied_cells_for_matching_ = 80;
    bool likelihood_field_dirty_ = true;
    double likelihood_max_distance_ = 1.0;

    double search_xy_range_ = 0.15;
    double search_xy_step_ = 0.05;
    double search_theta_range_ = 0.17;
    double search_theta_step_ = 0.085;
    std::size_t scan_match_beam_step_ = 10;
    double min_scan_match_score_ = 0.20;
    double min_update_translation_ = 0.01;
    double min_update_rotation_ = 0.01;

    int scans_integrated_ = 0;
    int stationary_scans_skipped_ = 0;
};

SimpleSlamNode::SimpleSlamNode()
: Node("simple_slam_node")
{
    initializeMap();

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&SimpleSlamNode::odomCallback, this, std::placeholders::_1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&SimpleSlamNode::scanCallback, this, std::placeholders::_1));

    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", map_qos);
    estimated_pose_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>("/estimated_pose", 10);
    scan_matched_pose_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>("/scan_matched_pose", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(this->get_logger(), "Simple SLAM node started.");
    RCLCPP_INFO(
        this->get_logger(),
        "Inputs: /odom, /scan. Outputs: /map, /estimated_pose, /scan_matched_pose, TF map->odom.");
}

void SimpleSlamNode::initializeMap()
{
    origin_x_ = -width_ * resolution_ / 2.0;
    origin_y_ = -height_ * resolution_ / 2.0;

    map_log_odds_.assign(width_ * height_, 0.0);

    map_msg_.header.frame_id = "map";
    map_msg_.info.resolution = resolution_;
    map_msg_.info.width = width_;
    map_msg_.info.height = height_;
    map_msg_.info.origin.position.x = origin_x_;
    map_msg_.info.origin.position.y = origin_y_;
    map_msg_.info.origin.orientation.w = 1.0;
    map_msg_.data.assign(width_ * height_, -1);

    likelihood_field_msg_ = map_msg_;
    likelihood_field_msg_.data.assign(width_ * height_, 0);
}

void SimpleSlamNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_odom_pose_ = odomMsgToPose(*msg);

    if (!odom_initialized_) {
        last_odom_pose_ = current_odom_pose_;
        odom_initialized_ = true;
        publishEstimatedPose(msg->header.stamp);
        publishMapToOdomTf(msg->header.stamp);
        RCLCPP_INFO(this->get_logger(), "First odom received. SLAM pose starts at map origin.");
    }
}

void SimpleSlamNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (!odom_initialized_) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000, "Waiting for /odom before using scans.");
        return;
    }

    bool moved_enough = odomMovedEnough(last_odom_pose_, current_odom_pose_);
    Pose2D predicted_pose = applyOdomDeltaToSlamPose(last_odom_pose_, current_odom_pose_);
    last_odom_pose_ = current_odom_pose_;

    if (scan_received_ && !moved_enough) {
        stationary_scans_skipped_++;
        publishEstimatedPose(msg->header.stamp);
        publishMapToOdomTf(msg->header.stamp);
        return;
    }

    ScanMatchResult result = matchScan(*msg, predicted_pose);
    slam_pose_ = result.pose;

    updateMapWithScan(*msg, slam_pose_);
    publishMap();
    publishEstimatedPose(msg->header.stamp);
    if (result.used_scan_matching) {
        publishScanMatchedPose(result.pose, msg->header.stamp);
    }
    publishMapToOdomTf(msg->header.stamp);

    scan_received_ = true;
    scans_integrated_++;
    likelihood_field_dirty_ = true;

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000,
        "SLAM integrated=%d stationary_skipped=%d pose=(%.2f, %.2f, %.2f) match=%s score=%.3f occupied=%d",
        scans_integrated_,
        stationary_scans_skipped_,
        slam_pose_.x,
        slam_pose_.y,
        slam_pose_.theta,
        result.used_scan_matching ? "yes" : "no",
        result.score,
        occupied_cell_count_);
}

Pose2D SimpleSlamNode::applyOdomDeltaToSlamPose(
    const Pose2D & old_odom, const Pose2D & new_odom) const
{
    double odom_dx = new_odom.x - old_odom.x;
    double odom_dy = new_odom.y - old_odom.y;
    double old_odom_theta = old_odom.theta;

    double local_dx =
        std::cos(old_odom_theta) * odom_dx + std::sin(old_odom_theta) * odom_dy;
    double local_dy =
        -std::sin(old_odom_theta) * odom_dx + std::cos(old_odom_theta) * odom_dy;
    double local_dtheta = normalizeAngle(new_odom.theta - old_odom.theta);

    Pose2D predicted = slam_pose_;
    predicted.x += std::cos(slam_pose_.theta) * local_dx -
        std::sin(slam_pose_.theta) * local_dy;
    predicted.y += std::sin(slam_pose_.theta) * local_dx +
        std::cos(slam_pose_.theta) * local_dy;
    predicted.theta = normalizeAngle(slam_pose_.theta + local_dtheta);
    return predicted;
}

ScanMatchResult SimpleSlamNode::matchScan(
    const sensor_msgs::msg::LaserScan & scan, const Pose2D & predicted)
{
    ScanMatchResult result;
    result.pose = predicted;

    if (!scan_received_) {
        return result;
    }

    if (occupied_cell_count_ < min_occupied_cells_for_matching_) {
        return result;
    }

    if (likelihood_field_dirty_) {
        rebuildLikelihoodField();
        likelihood_field_dirty_ = false;
    }

    double best_score = -std::numeric_limits<double>::infinity();
    Pose2D best_pose = predicted;

    for (double dx = -search_xy_range_; dx <= search_xy_range_ + 1e-9; dx += search_xy_step_) {
        for (double dy = -search_xy_range_; dy <= search_xy_range_ + 1e-9; dy += search_xy_step_) {
            for (double dtheta = -search_theta_range_;
                dtheta <= search_theta_range_ + 1e-9;
                dtheta += search_theta_step_)
            {
                Pose2D candidate;
                candidate.x = predicted.x + dx;
                candidate.y = predicted.y + dy;
                candidate.theta = normalizeAngle(predicted.theta + dtheta);

                double score = scoreScanAtPose(scan, candidate);
                if (score > best_score) {
                    best_score = score;
                    best_pose = candidate;
                }
            }
        }
    }

    result.score = std::max(best_score, 0.0);

    if (best_score >= min_scan_match_score_) {
        result.pose = best_pose;
        result.used_scan_matching = true;
    }

    return result;
}

void SimpleSlamNode::rebuildLikelihoodField()
{
    likelihood_field_msg_ = map_msg_;
    likelihood_field_msg_.data.assign(width_ * height_, 0);

    int max_distance_cells = static_cast<int>(std::ceil(likelihood_max_distance_ / resolution_));
    std::vector<int> distance_to_wall(width_ * height_, max_distance_cells + 1);
    std::queue<int> cells_to_visit;

    for (int row = 0; row < height_; ++row) {
        for (int col = 0; col < width_; ++col) {
            int index = row * width_ + col;
            if (map_msg_.data[index] > 50) {
                distance_to_wall[index] = 0;
                cells_to_visit.push(index);
            }
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

        int row = index / width_;
        int col = index % width_;
        int next_distance = distance_to_wall[index] + 1;

        if (next_distance > max_distance_cells) {
            continue;
        }

        for (const auto & offset : neighbor_offsets) {
            int next_col = col + offset[0];
            int next_row = row + offset[1];

            if (!isValidCell(next_col, next_row)) {
                continue;
            }

            int next_index = next_row * width_ + next_col;
            if (next_distance >= distance_to_wall[next_index]) {
                continue;
            }

            distance_to_wall[next_index] = next_distance;
            cells_to_visit.push(next_index);
        }
    }

    for (int i = 0; i < width_ * height_; ++i) {
        if (distance_to_wall[i] > max_distance_cells) {
            continue;
        }

        double distance_m = distance_to_wall[i] * resolution_;
        double likelihood = 1.0 - std::min(distance_m / likelihood_max_distance_, 1.0);
        likelihood_field_msg_.data[i] = static_cast<int8_t>(std::round(likelihood * 100.0));
    }
}

double SimpleSlamNode::scoreScanAtPose(
    const sensor_msgs::msg::LaserScan & scan, const Pose2D & pose) const
{
    double score = 0.0;
    int used_beams = 0;

    for (std::size_t i = 0; i < scan.ranges.size(); i += scan_match_beam_step_) {
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

double SimpleSlamNode::likelihoodAtWorld(double x, double y) const
{
    int col = 0;
    int row = 0;
    worldToGrid(x, y, col, row);

    if (!isValidCell(col, row)) {
        return 0.0;
    }

    int index = row * width_ + col;
    return likelihood_field_msg_.data[index] / 100.0;
}

void SimpleSlamNode::updateMapWithScan(
    const sensor_msgs::msg::LaserScan & scan, const Pose2D & pose)
{
    int robot_grid_x = 0;
    int robot_grid_y = 0;
    worldToGrid(pose.x, pose.y, robot_grid_x, robot_grid_y);

    if (!isValidCell(robot_grid_x, robot_grid_y)) {
        RCLCPP_WARN(this->get_logger(), "SLAM pose outside map bounds; scan skipped.");
        return;
    }

    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
        float range = scan.ranges[i];

        if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
            continue;
        }

        double beam_angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
        double hit_x = pose.x + range * std::cos(pose.theta + beam_angle);
        double hit_y = pose.y + range * std::sin(pose.theta + beam_angle);

        int hit_grid_x = 0;
        int hit_grid_y = 0;
        worldToGrid(hit_x, hit_y, hit_grid_x, hit_grid_y);

        if (!isValidCell(hit_grid_x, hit_grid_y)) {
            continue;
        }

        auto cells = bresenhamLine(robot_grid_x, robot_grid_y, hit_grid_x, hit_grid_y);
        if (cells.empty()) {
            continue;
        }

        for (std::size_t cell_index = 0; cell_index + 1 < cells.size(); ++cell_index) {
            int x = cells[cell_index].first;
            int y = cells[cell_index].second;
            int index = y * width_ + x;
            map_log_odds_[index] =
                clamp(map_log_odds_[index] + log_odds_free_, log_odds_min_, log_odds_max_);
        }

        int hit_index = cells.back().second * width_ + cells.back().first;
        map_log_odds_[hit_index] =
            clamp(map_log_odds_[hit_index] + log_odds_hit_, log_odds_min_, log_odds_max_);
    }
}

void SimpleSlamNode::publishMap()
{
    map_msg_.header.stamp = this->now();
    occupied_cell_count_ = 0;

    for (int i = 0; i < width_ * height_; ++i) {
        double probability = 1.0 / (1.0 + std::exp(-map_log_odds_[i]));

        if (map_log_odds_[i] == 0.0) {
            map_msg_.data[i] = -1;
        } else {
            map_msg_.data[i] = static_cast<int8_t>(std::round(probability * 100.0));
        }

        if (map_msg_.data[i] > 50) {
            occupied_cell_count_++;
        }
    }

    map_pub_->publish(map_msg_);
}

void SimpleSlamNode::publishEstimatedPose(const rclcpp::Time & stamp)
{
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "map";
    msg.pose.position.x = slam_pose_.x;
    msg.pose.position.y = slam_pose_.y;
    msg.pose.orientation = yawToQuaternion(slam_pose_.theta);
    estimated_pose_pub_->publish(msg);
}

void SimpleSlamNode::publishScanMatchedPose(const Pose2D & pose, const rclcpp::Time & stamp)
{
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "map";
    msg.pose.position.x = pose.x;
    msg.pose.position.y = pose.y;
    msg.pose.orientation = yawToQuaternion(pose.theta);
    scan_matched_pose_pub_->publish(msg);
}

void SimpleSlamNode::publishMapToOdomTf(const rclcpp::Time & stamp)
{
    tf2::Quaternion map_to_base_rotation;
    map_to_base_rotation.setRPY(0.0, 0.0, slam_pose_.theta);

    tf2::Transform map_to_base;
    map_to_base.setOrigin(tf2::Vector3(slam_pose_.x, slam_pose_.y, 0.0));
    map_to_base.setRotation(map_to_base_rotation);

    tf2::Quaternion odom_to_base_rotation;
    odom_to_base_rotation.setRPY(0.0, 0.0, current_odom_pose_.theta);

    tf2::Transform odom_to_base;
    odom_to_base.setOrigin(tf2::Vector3(current_odom_pose_.x, current_odom_pose_.y, 0.0));
    odom_to_base.setRotation(odom_to_base_rotation);

    tf2::Transform map_to_odom = map_to_base * odom_to_base.inverse();

    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "map";
    msg.child_frame_id = "odom";
    msg.transform = tf2::toMsg(map_to_odom);
    tf_broadcaster_->sendTransform(msg);
}

Pose2D SimpleSlamNode::odomMsgToPose(const nav_msgs::msg::Odometry & msg) const
{
    Pose2D pose;
    pose.x = msg.pose.pose.position.x;
    pose.y = msg.pose.pose.position.y;
    pose.theta = tf2::getYaw(msg.pose.pose.orientation);
    return pose;
}

bool SimpleSlamNode::odomMovedEnough(const Pose2D & old_odom, const Pose2D & new_odom) const
{
    double dx = new_odom.x - old_odom.x;
    double dy = new_odom.y - old_odom.y;
    double translation = std::sqrt(dx * dx + dy * dy);
    double rotation = std::abs(normalizeAngle(new_odom.theta - old_odom.theta));

    return translation >= min_update_translation_ || rotation >= min_update_rotation_;
}

geometry_msgs::msg::Quaternion SimpleSlamNode::yawToQuaternion(double yaw) const
{
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw);

    geometry_msgs::msg::Quaternion msg;
    msg.x = quaternion.x();
    msg.y = quaternion.y();
    msg.z = quaternion.z();
    msg.w = quaternion.w();
    return msg;
}

void SimpleSlamNode::worldToGrid(double wx, double wy, int & gx, int & gy) const
{
    gx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
    gy = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
}

bool SimpleSlamNode::isValidCell(int x, int y) const
{
    return x >= 0 && x < width_ && y >= 0 && y < height_;
}

std::vector<std::pair<int, int>> SimpleSlamNode::bresenhamLine(
    int x0, int y0, int x1, int y1) const
{
    std::vector<std::pair<int, int>> cells;

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;

    int x = x0;
    int y = y0;

    while (true) {
        cells.push_back({x, y});

        if (x == x1 && y == y1) {
            break;
        }

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }

    return cells;
}

double SimpleSlamNode::normalizeAngle(double angle) const
{
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

double SimpleSlamNode::clamp(double value, double low, double high) const
{
    return std::min(std::max(value, low), high);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleSlamNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
