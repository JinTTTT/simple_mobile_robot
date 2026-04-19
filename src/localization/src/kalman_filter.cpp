#include "localization/kalman_filter.hpp"

#include <cmath>

KalmanFilter::KalmanFilter(const KalmanFilterParameters & parameters)
: parameters_(parameters),
  state_{0.0, 0.0, 0.0},
  covariance_{},
  initialized_(false)
{
}

void KalmanFilter::initialize(double x, double y, double theta)
{
    state_.x = x;
    state_.y = y;
    state_.theta = normalizeAngle(theta);
    covariance_ = makeInitialCovariance();
    initialized_ = true;
}

void KalmanFilter::predictFromOdometry(
    double old_x, double old_y, double old_theta,
    double new_x, double new_y, double new_theta)
{
    if (!initialized_) {
        return;
    }

    double dx = new_x - old_x;
    double dy = new_y - old_y;
    double dtheta = normalizeAngle(new_theta - old_theta);
    double distance = std::sqrt(dx * dx + dy * dy);
    double rotation = std::abs(dtheta);

    if (distance < parameters_.min_translation_delta &&
        rotation < parameters_.min_rotation_delta) {
        return;
    }

    state_.x += dx;
    state_.y += dy;
    state_.theta = normalizeAngle(state_.theta + dtheta);

    Matrix3x3 process_noise = makeProcessNoise(distance, rotation);

    for (std::size_t i = 0; i < covariance_.size(); ++i) {
        covariance_[i] += process_noise[i];
    }
}

void KalmanFilter::correctWithPoseMeasurement(
    double measured_x, double measured_y, double measured_theta,
    const Matrix3x3 & measurement_covariance)
{
    if (!initialized_) {
        return;
    }

    double innovation_x = measured_x - state_.x;
    double innovation_y = measured_y - state_.y;
    double innovation_theta = normalizeAngle(measured_theta - state_.theta);

    double innovation[3] = {innovation_x, innovation_y, innovation_theta};
    int diagonal_indices[3] = {0, 4, 8};

    for (int i = 0; i < 3; ++i) {
        int index = diagonal_indices[i];
        double predicted_variance = covariance_[index];
        double measurement_variance = measurement_covariance[index];
        double innovation_variance = predicted_variance + measurement_variance;

        if (innovation_variance <= 0.0) {
            continue;
        }

        double kalman_gain = predicted_variance / innovation_variance;
        double correction = kalman_gain * innovation[i];

        if (i == 0) {
            state_.x += correction;
        } else if (i == 1) {
            state_.y += correction;
        } else {
            state_.theta = normalizeAngle(state_.theta + correction);
        }

        covariance_[index] = (1.0 - kalman_gain) * predicted_variance;
    }
}

bool KalmanFilter::isInitialized() const
{
    return initialized_;
}

KalmanPose KalmanFilter::estimatePose() const
{
    return state_;
}

const KalmanFilter::Matrix3x3 & KalmanFilter::covariance() const
{
    return covariance_;
}

KalmanFilter::Matrix3x3 KalmanFilter::makeInitialCovariance() const
{
    return {
        square(parameters_.initial_std_x), 0.0, 0.0,
        0.0, square(parameters_.initial_std_y), 0.0,
        0.0, 0.0, square(parameters_.initial_std_theta)
    };
}

KalmanFilter::Matrix3x3 KalmanFilter::makeProcessNoise(
    double distance, double rotation) const
{
    double std_x =
        parameters_.base_process_std_x + parameters_.distance_noise_scale * distance;
    double std_y =
        parameters_.base_process_std_y + parameters_.distance_noise_scale * distance;
    double std_theta =
        parameters_.base_process_std_theta + parameters_.rotation_noise_scale * rotation;

    return {
        square(std_x), 0.0, 0.0,
        0.0, square(std_y), 0.0,
        0.0, 0.0, square(std_theta)
    };
}

double KalmanFilter::normalizeAngle(double angle) const
{
    while (angle > M_PI) {
        angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2.0 * M_PI;
    }
    return angle;
}

double KalmanFilter::square(double value) const
{
    return value * value;
}
