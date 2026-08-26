#include "fusion/fusion.hpp"
#include <cmath>

namespace imu {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

double accelToPitchDeg(double accel_y, double accel_z, double accel_x) {
  return std::atan2(accel_y, std::sqrt(accel_x * accel_x + accel_z * accel_z)) * (180.0 / kPi);
}

}  // namespace imu
