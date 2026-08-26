#pragma once

// Two independent ways to fuse an accelerometer + gyroscope into one
// stable angle estimate -- algorithm-for-algorithm identical to
// ../../../sim/imu_sim/fusion.py (Python). See that file's module
// docstring for the full "why fuse two sensors at all" explanation; this
// header is the same math, in C++, for the real host app.

namespace imu {

// The angle gravity implies, straight from the accelerometer, in degrees.
// No long-term drift (gravity's direction never changes), but noisy
// sample-to-sample and wrong under real linear acceleration.
double accelToPitchDeg(double accel_y, double accel_z, double accel_x);

class ComplementaryFilter {
 public:
  explicit ComplementaryFilter(double alpha = 0.98) : alpha_(alpha) {}

  double update(double accelAngleDeg, double gyroRateDegS, double dtSeconds) {
    const double gyroEstimate = angle_ + gyroRateDegS * dtSeconds;
    angle_ = alpha_ * gyroEstimate + (1.0 - alpha_) * accelAngleDeg;
    return angle_;
  }

  double angle() const { return angle_; }

 private:
  double alpha_;
  double angle_ = 0.0;
};

// A 2-state (angle, gyro bias) Kalman filter for one axis -- the same
// well-known formulation ported in imu_sim/fusion.py's KalmanFilter1D.
// Tracks its own uncertainty and computes the statistically optimal blend
// weight every update, rather than a fixed hand-picked alpha.
class KalmanFilter1D {
 public:
  KalmanFilter1D(double qAngle = 0.001, double qBias = 0.003, double rMeasure = 0.03)
      : q_angle_(qAngle), q_bias_(qBias), r_measure_(rMeasure) {}

  double update(double accelAngleDeg, double gyroRateDegS, double dtSeconds) {
    // --- predict ---
    const double rate = gyroRateDegS - bias_;
    angle_ += dtSeconds * rate;

    p00_ += dtSeconds * (dtSeconds * p11_ - p01_ - p10_ + q_angle_);
    p01_ -= dtSeconds * p11_;
    p10_ -= dtSeconds * p11_;
    p11_ += q_bias_ * dtSeconds;

    // --- measurement update ---
    const double innovation = accelAngleDeg - angle_;
    const double s = p00_ + r_measure_;
    const double k0 = p00_ / s;
    const double k1 = p10_ / s;

    angle_ += k0 * innovation;
    bias_ += k1 * innovation;

    const double p00Prev = p00_;
    const double p01Prev = p01_;
    p00_ -= k0 * p00Prev;
    p01_ -= k0 * p01Prev;
    p10_ -= k1 * p00Prev;
    p11_ -= k1 * p01Prev;

    return angle_;
  }

  double angle() const { return angle_; }
  double bias() const { return bias_; }

 private:
  double q_angle_, q_bias_, r_measure_;
  double angle_ = 0.0;
  double bias_ = 0.0;
  double p00_ = 0.0, p01_ = 0.0, p10_ = 0.0, p11_ = 0.0;
};

}  // namespace imu
