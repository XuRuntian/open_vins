#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <Eigen/Geometry>

namespace ov_offline {

struct GyroSample {
  double time;
  Eigen::Vector3d rate;
};

struct VisualRotation {
  double begin, end;
  Eigen::Vector3d rotation;
};

struct TimingBootstrap {
  std::string reason = "disabled";
  bool applied = false;
  size_t intervals = 0;
  double configured = 0, candidate = 0, selected = 0;
  double error_before = 0, error_after = 0, split_difference = 0;
};

inline double median(std::vector<double> values) {
  std::sort(values.begin(), values.end());
  const size_t middle = values.size() / 2;
  return values.size() % 2 ? values[middle] : (values[middle - 1] + values[middle]) / 2;
}

// Integrate the camera-coordinate passive rotation. Never extrapolate the IMU
// endpoints or integrate across missing data, even during the offset search.
inline bool gyro_rotation(const std::vector<GyroSample> &imu, double begin, double end,
                          const Eigen::Matrix3d &R_imu_to_camera, Eigen::Vector3d &rotation) {
  if (imu.size() < 2 || begin < imu.front().time || end > imu.back().time || end <= begin) return false;
  auto upper = std::upper_bound(imu.begin(), imu.end(), begin,
                               [](double time, const GyroSample &s) { return time < s.time; });
  size_t i = static_cast<size_t>(upper - imu.begin()) - 1;
  Eigen::Matrix3d integrated = Eigen::Matrix3d::Identity();
  double time = begin;
  while (time < end && i + 1 < imu.size()) {
    const auto &a = imu[i];
    const auto &b = imu[i + 1];
    const double dt = b.time - a.time;
    if (dt <= 0 || dt > 0.02 || !a.rate.allFinite() || !b.rate.allFinite()) return false;
    const double stop = std::min(end, b.time);
    Eigen::Vector3d rate = a.rate + ((time + stop) / 2 - a.time) / dt * (b.rate - a.rate);
    Eigen::Vector3d step = -R_imu_to_camera * rate * (stop - time);
    if (step.norm() > 0) integrated = Eigen::AngleAxisd(step.norm(), step.normalized()).toRotationMatrix() * integrated;
    time = stop;
    ++i;
  }
  if (time < end) return false;
  const Eigen::AngleAxisd angle(integrated);
  rotation = angle.angle() * angle.axis();
  return rotation.allFinite();
}

// Bias is a nuisance parameter, not a change to the estimator's bias state.
// Centering each component prevents a constant gyro bias from looking like lag.
inline double timing_error(const std::vector<Eigen::Vector3d> &residuals, size_t begin, size_t end) {
  Eigen::Vector3d bias;
  for (size_t axis = 0; axis < 3; ++axis) {
    std::vector<double> values;
    for (size_t i = begin; i < end; ++i) values.push_back(residuals[i](axis));
    bias(axis) = median(values);
  }
  std::vector<double> errors;
  for (size_t i = begin; i < end; ++i) errors.push_back((residuals[i] - bias).norm());
  return median(errors);
}

inline TimingBootstrap estimate_timing(const std::vector<VisualRotation> &visual,
                                      const std::vector<GyroSample> &imu,
                                      const Eigen::Matrix3d &R_imu_to_camera, double configured) {
  TimingBootstrap result;
  result.configured = result.candidate = result.selected = configured;
  result.reason = "insufficient_observations";
  // A bounded local search, not permission to repair arbitrary timestamp faults.
  constexpr double radius = 0.100, step = 0.002;
  constexpr int count = 101, center = 50;
  std::vector<VisualRotation> common;
  for (const auto &sample : visual) {
    Eigen::Vector3d unused;
    if (sample.rotation.allFinite() && sample.end > sample.begin &&
        gyro_rotation(imu, sample.begin + configured - radius, sample.end + configured + radius,
                      R_imu_to_camera, unused)) common.push_back(sample);
  }
  result.intervals = common.size();
  if (common.size() < 24 || common.back().end - common.front().begin < 2.4) return result;
  std::vector<Eigen::Vector3d> rates;
  for (const auto &s : common) rates.push_back(s.rotation / (s.end - s.begin));
  if (timing_error(rates, 0, rates.size()) < 0.10) {
    result.reason = "insufficient_rotational_excitation";
    return result;
  }
  std::vector<double> scores, first, second;
  for (int index = 0; index < count; ++index) {
    const double offset = configured + (index - center) * step;
    std::vector<Eigen::Vector3d> residuals;
    for (const auto &s : common) {
      Eigen::Vector3d prediction;
      if (!gyro_rotation(imu, s.begin + offset, s.end + offset, R_imu_to_camera, prediction)) {
        result.reason = "imu_unavailable";
        return result;
      }
      residuals.push_back((s.rotation - prediction) / (s.end - s.begin));
    }
    scores.push_back(timing_error(residuals, 0, residuals.size()));
    first.push_back(timing_error(residuals, 0, residuals.size() / 2));
    second.push_back(timing_error(residuals, residuals.size() / 2, residuals.size()));
  }
  auto best = [](const std::vector<double> &v) { return static_cast<int>(std::min_element(v.begin(), v.end()) - v.begin()); };
  const int index = best(scores), half1 = best(first), half2 = best(second);
  result.candidate = configured + (index - center) * step;
  result.error_before = scores[center];
  result.error_after = scores[index];
  result.split_difference = std::abs(half1 - half2) * step;
  if (index < 5 || index >= count - 5 || half1 < 5 || half1 >= count - 5 || half2 < 5 || half2 >= count - 5)
    result.reason = "search_boundary";
  else if (result.split_difference > 0.012 || std::abs(index - half1) * step > 0.012 || std::abs(index - half2) * step > 0.012)
    result.reason = "inconsistent_subwindows";
  else if (scores[index] > 0.06)
    result.reason = "poor_rotation_fit";
  else if (scores[center] - scores[index] < 0.005 || scores[index] > 0.75 * scores[center])
    result.reason = "calibration_not_disproved";
  else {
    double alternative = std::numeric_limits<double>::infinity();
    for (int i = 0; i < count; ++i)
      if (std::abs(i - index) * step >= 0.012) alternative = std::min(alternative, scores[i]);
    if (alternative - scores[index] < std::max(0.003, 0.10 * scores[index]))
      result.reason = "ambiguous_minimum";
    else {
      result.reason = "accepted";
      result.applied = true;
      result.selected = result.candidate;
    }
  }
  return result;
}

} // namespace ov_offline
