#include <iostream>
#include <stdexcept>
#include "../timing_bootstrap.h"

using namespace ov_offline;

void require(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}

int main() {
  const Eigen::Matrix3d R = Eigen::AngleAxisd(0.7, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
  std::vector<GyroSample> imu;
  for (int i = 0; i <= 1400; ++i) {
    const double t = i * 0.005;
    imu.push_back({t, Eigen::Vector3d(0.5 * std::sin(4 * t), 0.3 * std::cos(7 * t), 0.4 * std::sin(2 * t * t))});
  }
  auto observe = [&](double offset) {
    std::vector<VisualRotation> visual;
    for (int i = 0; i < 54; ++i) {
      const double t = 0.5 + 0.1 * i;
      Eigen::Vector3d v;
      require(gyro_rotation(imu, t + offset, t + 0.1 + offset, R, v), "Synthetic IMU coverage");
      // Constant bias and sparse visual outliers must not choose the lag.
      v += Eigen::Vector3d(0.001, -0.002, 0.0005);
      if (i % 11 == 0) v += Eigen::Vector3d(0.02, -0.01, 0.03);
      visual.push_back({t, t + 0.1, v});
    }
    return visual;
  };
  const auto visual = observe(-0.010);
  auto result = estimate_timing(visual, imu, R, -0.048);
  require(result.applied && std::abs(result.selected + 0.010) < 0.003, "Known offset with bias and outliers");
  result = estimate_timing(observe(0.026), imu, R, -0.048);
  require(result.applied && std::abs(result.selected - 0.026) < 0.003, "Positive offset convention");
  result = estimate_timing(observe(-0.048), imu, R, -0.048);
  require(!result.applied && result.reason == "calibration_not_disproved", "Do not replace correct calibration");
  result = estimate_timing(observe(0.15), imu, R, -0.048);
  require(!result.applied, "Do not accept out-of-range lag");
  auto inconsistent = observe(-0.010), other = observe(0.026);
  std::copy(other.begin() + 27, other.end(), inconsistent.begin() + 27);
  result = estimate_timing(inconsistent, imu, R, -0.048);
  require(!result.applied && result.reason == "inconsistent_subwindows", "Reject changing lag");
  std::vector<VisualRotation> constant = visual;
  for (auto &s : constant) s.rotation = Eigen::Vector3d(0, 0, 0.05);
  result = estimate_timing(constant, imu, R, -0.048);
  require(!result.applied && result.reason == "insufficient_rotational_excitation", "Constant rate is unobservable");
  require(!estimate_timing({}, imu, R, -0.048).applied, "Empty visual observations");
  require(!estimate_timing(visual, {}, R, -0.048).applied, "Empty IMU");
  Eigen::Vector3d rotation;
  std::vector<GyroSample> constant_imu;
  for (int i = 0; i <= 200; ++i) constant_imu.push_back({i * 0.005, Eigen::Vector3d(0, 0, 0.5)});
  require(gyro_rotation(constant_imu, 0.123, 0.456, R, rotation), "Fractional endpoints are bracketed");
  require((rotation + R * Eigen::Vector3d(0, 0, 0.5) * (0.456 - 0.123)).norm() < 1e-10,
          "Independent constant-axis integration/sign/extrinsic check");
  require(!gyro_rotation(imu, -0.1, 0.1, R, rotation), "No left endpoint extrapolation");
  require(!gyro_rotation(imu, 6.9, 7.1, R, rotation), "No right endpoint extrapolation");
  auto gap = imu;
  gap.erase(gap.begin() + 300, gap.begin() + 310);
  require(!gyro_rotation(gap, 1.4, 1.6, R, rotation), "Do not bridge missing IMU");
  std::cout << "Timing bootstrap tests passed\n";
}
