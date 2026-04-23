#pragma once

#include <array>

struct KalmanFilterParameters {
    double initial_std_x = 0.02;
    double initial_std_y = 0.02;
    double initial_std_theta = 0.02;

    double base_process_std_x = 0.005;
    double base_process_std_y = 0.005;
    double base_process_std_theta = 0.002;

    double distance_noise_scale = 0.10;
    double rotation_noise_scale = 0.10;

    double min_translation_delta = 0.0002;
    double min_rotation_delta = 0.0002;
};

struct KalmanPose {
    double x;
    double y;
    double theta;
};

class KalmanFilter
{
public:
    using Matrix3x3 = std::array<double, 9>;

    explicit KalmanFilter(const KalmanFilterParameters & parameters = KalmanFilterParameters());

    void initialize(double x, double y, double theta);

    void predictFromOdometry(
        double old_x, double old_y, double old_theta,
        double new_x, double new_y, double new_theta);

    void correctWithPoseMeasurement(
        double measured_x, double measured_y, double measured_theta,
        const Matrix3x3 & measurement_covariance);

    bool isInitialized() const;

    KalmanPose estimatePose() const;

    const Matrix3x3 & covariance() const;

private:
    Matrix3x3 makeInitialCovariance() const;
    Matrix3x3 makeProcessNoise(double distance, double rotation) const;
    double normalizeAngle(double angle) const;
    double square(double value) const;

    KalmanFilterParameters parameters_;
    KalmanPose state_;
    Matrix3x3 covariance_;
    bool initialized_;
};
