#include <gtest/gtest.h>
#include <cmath>
#include <random>
#include <vector>

#include "fusion/fusion.hpp"

using namespace imu;

namespace {

struct SyntheticRow {
  double true_pitch_deg;
  double accel_x, accel_y, accel_z;
  double gyro_y;
};

// Same synthetic-motion idea as
// ../../sim/imu_sim/synth.py's generate_motion() -- a device gently
// rocking in pitch, no real linear acceleration, gyro carrying a fixed
// bias -- reimplemented here rather than shared, so this test file has
// no dependency on Python at all.
std::vector<SyntheticRow> generateMotion(double durationS, double sampleRateHz, unsigned seed,
                                          double gyroBiasDegS = 2.0) {
  std::mt19937 rng(seed);
  std::normal_distribution<double> accelNoise(0.0, 0.03);
  std::normal_distribution<double> gyroNoise(0.0, 0.5);

  const double dt = 1.0 / sampleRateHz;
  const int n = static_cast<int>(durationS * sampleRateHz);
  const double amplitudeDeg = 30.0;
  const double omega = 2.0 * 3.14159265358979323846 * 0.1;

  std::vector<SyntheticRow> rows;
  rows.reserve(n);
  for (int i = 0; i < n; ++i) {
    double t = i * dt;
    double truePitch = amplitudeDeg * std::sin(omega * t);
    double trueRate = amplitudeDeg * omega * std::cos(omega * t);
    double pitchRad = truePitch * (3.14159265358979323846 / 180.0);

    SyntheticRow row;
    row.true_pitch_deg = truePitch;
    row.accel_x = accelNoise(rng);
    row.accel_y = std::sin(pitchRad) + accelNoise(rng);
    row.accel_z = std::cos(pitchRad) + accelNoise(rng);
    row.gyro_y = trueRate + gyroBiasDegS + gyroNoise(rng);
    rows.push_back(row);
  }
  return rows;
}

double rmse(const std::vector<double>& a, const std::vector<double>& b) {
  double sum = 0.0;
  for (size_t i = 0; i < a.size(); ++i) {
    double d = a[i] - b[i];
    sum += d * d;
  }
  return std::sqrt(sum / static_cast<double>(a.size()));
}

}  // namespace

TEST(AccelToPitch, LevelDeviceReadsZero) {
  EXPECT_NEAR(accelToPitchDeg(0.0, 1.0, 0.0), 0.0, 1e-6);
}

TEST(AccelToPitch, NinetyDegreeTilt) {
  EXPECT_NEAR(accelToPitchDeg(1.0, 0.0, 0.0), 90.0, 1e-6);
}

TEST(Fusion, ComplementaryFilterBeatsGyroOnlyOverTime) {
  auto rows = generateMotion(20.0, 200.0, 1);
  const double dt = 1.0 / 200.0;

  ComplementaryFilter comp(0.98);
  double gyroOnlyAngle = 0.0;
  std::vector<double> truth, fused, gyroOnly;
  for (const auto& row : rows) {
    double accelAngle = accelToPitchDeg(row.accel_y, row.accel_z, row.accel_x);
    fused.push_back(comp.update(accelAngle, row.gyro_y, dt));
    gyroOnlyAngle += row.gyro_y * dt;
    gyroOnly.push_back(gyroOnlyAngle);
    truth.push_back(row.true_pitch_deg);
  }

  size_t half = truth.size() / 2;
  std::vector<double> truthHalf(truth.begin() + half, truth.end());
  std::vector<double> fusedHalf(fused.begin() + half, fused.end());
  std::vector<double> gyroOnlyHalf(gyroOnly.begin() + half, gyroOnly.end());

  EXPECT_LT(rmse(truthHalf, fusedHalf), rmse(truthHalf, gyroOnlyHalf));
}

TEST(Fusion, KalmanFilterBeatsGyroOnlyOverTime) {
  auto rows = generateMotion(20.0, 200.0, 1);
  const double dt = 1.0 / 200.0;

  KalmanFilter1D kf;
  double gyroOnlyAngle = 0.0;
  std::vector<double> truth, fused, gyroOnly;
  for (const auto& row : rows) {
    double accelAngle = accelToPitchDeg(row.accel_y, row.accel_z, row.accel_x);
    fused.push_back(kf.update(accelAngle, row.gyro_y, dt));
    gyroOnlyAngle += row.gyro_y * dt;
    gyroOnly.push_back(gyroOnlyAngle);
    truth.push_back(row.true_pitch_deg);
  }

  size_t half = truth.size() / 2;
  std::vector<double> truthHalf(truth.begin() + half, truth.end());
  std::vector<double> fusedHalf(fused.begin() + half, fused.end());
  std::vector<double> gyroOnlyHalf(gyroOnly.begin() + half, gyroOnly.end());

  EXPECT_LT(rmse(truthHalf, fusedHalf), rmse(truthHalf, gyroOnlyHalf));
}

TEST(Fusion, KalmanEstimatesTheGyroBias) {
  auto rows = generateMotion(30.0, 200.0, 1, /*gyroBiasDegS=*/2.0);
  const double dt = 1.0 / 200.0;
  KalmanFilter1D kf;
  for (const auto& row : rows) {
    double accelAngle = accelToPitchDeg(row.accel_y, row.accel_z, row.accel_x);
    kf.update(accelAngle, row.gyro_y, dt);
  }
  EXPECT_NEAR(kf.bias(), 2.0, 0.5);
}
