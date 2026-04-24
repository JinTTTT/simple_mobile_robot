#pragma once

#include "geometry_msgs/msg/quaternion.hpp"

double normalizeAngle(double angle);
double square(double value);
geometry_msgs::msg::Quaternion yawToQuaternion(double yaw);

