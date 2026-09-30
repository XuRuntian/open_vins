// Replay adapter only: estimation and initialization remain in official OpenVINS.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include "core/VioManager.h"
#include "state/State.h"
#include "utils/print.h"
#include "utils/sensor_data.h"
#include "timing_bootstrap.h"

using Row = std::map<std::string, double>;

std::vector<std::string> split(const std::string &line) {
  std::vector<std::string> values;
  std::stringstream stream(line);
  std::string value;
  while (std::getline(stream, value, ',')) {
    if (!value.empty() && value.back() == '\r') value.pop_back();
    values.push_back(value);
  }
  return values;
}

double number(const std::string &value) {
  size_t used = 0;
  double result = std::stod(value, &used);
  if (used != value.size() || !std::isfinite(result)) throw std::runtime_error("Invalid numeric value: " + value);
  return result;
}

std::vector<Row> read_csv(const std::string &path) {
  std::ifstream input(path);
  std::string line;
  if (!input || !std::getline(input, line)) throw std::runtime_error("Cannot read CSV: " + path);
  auto header = split(line);
  std::vector<Row> rows;
  double previous = -std::numeric_limits<double>::infinity();
  while (std::getline(input, line)) {
    if (line.empty()) continue;
    auto values = split(line);
    if (values.size() != header.size()) throw std::runtime_error("CSV column mismatch: " + path);
    Row row;
    for (size_t i = 0; i < values.size(); ++i) row.emplace(header[i], number(values[i]));
    if (row.at("timestamp") <= previous) throw std::runtime_error("CSV timestamps must strictly increase: " + path);
    previous = row.at("timestamp");
    rows.push_back(row);
  }
  if (rows.empty()) throw std::runtime_error("Empty CSV: " + path);
  return rows;
}

class Video {
  cv::VideoCapture capture;
  int next_index = 0;
public:
  explicit Video(const std::string &path) : capture(path) {
    if (!capture.isOpened()) throw std::runtime_error("Cannot open video: " + path);
  }
  cv::Mat frame(double index) {
    if (index < next_index || index != std::floor(index) || index > std::numeric_limits<int>::max())
      throw std::runtime_error("Video frame indices must increase and be nonnegative integers");
    // Sequential decoding avoids inaccurate keyframe seeking in compressed video.
    while (next_index < static_cast<int>(index)) {
      if (!capture.grab()) throw std::runtime_error("Video shorter than timestamp table");
      ++next_index;
    }
    cv::Mat image, gray;
    if (!capture.read(image)) throw std::runtime_error("Cannot decode requested video frame");
    ++next_index;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    return gray;
  }
};

ov_offline::TimingBootstrap bootstrap_timing(const std::string &video, const std::vector<Row> &pairs,
                                            const std::vector<Row> &imu, double begin, double end,
                                            const ov_msckf::VioManagerOptions &params) {
  // Offline look-ahead is restricted to the requested window. Images are read
  // again for VIO; no future measurements or extra history are fed to the filter.
  const uint64_t rng_state = cv::theRNG().state;
  struct RestoreRng {
    uint64_t state;
    ~RestoreRng() { cv::theRNG().state = state; }
  } restore{rng_state};
  cv::setRNGSeed(0);
  Video source(video);
  cv::Mat previous;
  double previous_time = 0;
  std::vector<ov_offline::VisualRotation> rotations;
  auto camera = params.camera_intrinsics.at(0);
  const double focal = camera->get_value()(0);
  cv::Mat allowed;
  if (params.use_mask) cv::compare(params.masks.at(0), 0, allowed, cv::CMP_EQ);
  for (const auto &pair : pairs) {
    const double time = pair.at("timestamp");
    if (time < begin) continue;
    if (time >= std::min(end, begin + 6.0)) break;
    if (!previous.empty() && time - previous_time < 0.095) continue;
    cv::Mat gray = source.frame(pair.at("left_frame_index"));
    if (params.downsample_cameras) cv::pyrDown(gray, gray);
    if (!previous.empty() && time - previous_time < 0.15) {
      std::vector<cv::Point2f> points, next, back, a, b;
      cv::goodFeaturesToTrack(previous, points, 600, 0.01, 15, allowed);
      if (points.size() >= 30) {
        std::vector<uchar> forward_ok, back_ok;
        std::vector<float> error;
        cv::calcOpticalFlowPyrLK(previous, gray, points, next, forward_ok, error);
        cv::calcOpticalFlowPyrLK(gray, previous, next, back, back_ok, error);
        for (size_t i = 0; i < points.size(); ++i) {
          if (!forward_ok[i] || !back_ok[i] || cv::norm(points[i] - back[i]) > 1.0 ||
              next[i].x < 0 || next[i].y < 0 || next[i].x >= gray.cols || next[i].y >= gray.rows) continue;
          if (!allowed.empty() && !allowed.at<uchar>(static_cast<int>(next[i].y), static_cast<int>(next[i].x))) continue;
          const auto p = camera->undistort_cv(points[i]), q = camera->undistort_cv(next[i]);
          if (!std::isfinite(p.x + p.y + q.x + q.y) || cv::norm(p) > 2 || cv::norm(q) > 2) continue;
          a.push_back(p);
          b.push_back(q);
        }
        if (a.size() >= 30) {
          cv::Mat mask;
          const cv::Mat essential = cv::findEssentialMat(a, b, 1.0, cv::Point2d(0, 0), cv::RANSAC, 0.999, 1.0 / focal, mask);
          if (essential.rows == 3 && essential.cols == 3 && cv::countNonZero(mask) >= std::max(30.0, 0.4 * a.size())) {
            cv::Mat r1, r2, translation, v1, v2;
            cv::decomposeEssentialMat(essential, r1, r2, translation);
            cv::Rodrigues(r1, v1);
            cv::Rodrigues(r2, v2);
            const cv::Mat &v = cv::norm(v1) < cv::norm(v2) ? v1 : v2;
            // At this short baseline the alternate essential rotation is near pi.
            if (cv::norm(v) < 0.30)
              rotations.push_back({previous_time, time, Eigen::Vector3d(v.at<double>(0), v.at<double>(1), v.at<double>(2))});
          }
        }
      }
    }
    previous = gray;
    previous_time = time;
  }
  std::vector<ov_offline::GyroSample> gyroscope;
  const Eigen::Matrix3d Dw = ov_msckf::State::Dm(params.state_options.imu_model, params.vec_dw);
  const Eigen::Matrix3d Da = ov_msckf::State::Dm(params.state_options.imu_model, params.vec_da);
  const Eigen::Matrix3d Tg = ov_msckf::State::Tg(params.vec_tg);
  const Eigen::Matrix3d R_acc = ov_core::quat_2_Rot(params.q_ACCtoIMU);
  const Eigen::Matrix3d R_gyro = ov_core::quat_2_Rot(params.q_GYROtoIMU);
  for (const auto &row : imu) {
    const Eigen::Vector3d wm(row.at("angular_velocity_x"), row.at("angular_velocity_y"), row.at("angular_velocity_z"));
    const Eigen::Vector3d am(row.at("linear_acceleration_x"), row.at("linear_acceleration_y"), row.at("linear_acceleration_z"));
    // Same intrinsic correction order as Propagator; these are local predictions,
    // not edits to the input measurements or the filter's bias state.
    const Eigen::Vector3d acceleration = R_acc * Da * (am - params.init_options.init_dyn_bias_a);
    gyroscope.push_back({row.at("timestamp"), R_gyro * Dw * (wm - params.init_options.init_dyn_bias_g - Tg * acceleration)});
  }
  const Eigen::Matrix3d R = ov_core::quat_2_Rot(params.camera_extrinsics.at(0).head<4>());
  return ov_offline::estimate_timing(rotations, gyroscope, R, params.calib_camimu_dt);
}

int run(int argc, char **argv) {
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "roboego_openvins_offline --config FILE --left_video FILE --right_video FILE "
                 "--stereo_pairs CSV --imu CSV --output_dir DIR [--start_s 0] [--duration_s SECONDS] "
                 "[--time-offset-mode config|estimate] [--timing-bootstrap auto|off]\n";
    return 0;
  }
  std::map<std::string, std::string> args;
  const std::vector<std::string> required = {"--config", "--left_video", "--right_video", "--stereo_pairs", "--imu", "--output_dir"};
  for (int i = 1; i < argc; i += 2) {
    if (i + 1 >= argc || !args.emplace(argv[i], argv[i + 1]).second)
      throw std::runtime_error("Missing value or duplicate argument");
  }
  for (const auto &key : required) if (!args.count(key)) throw std::runtime_error("Missing argument: " + key);
  for (const auto &arg : args) {
    if (std::find(required.begin(), required.end(), arg.first) == required.end() &&
        arg.first != "--start_s" && arg.first != "--duration_s" && arg.first != "--time-offset-mode" &&
        arg.first != "--timing-bootstrap")
      throw std::runtime_error("Unknown argument: " + arg.first);
  }
  double start = args.count("--start_s") ? number(args.at("--start_s")) : 0.0;
  double duration = args.count("--duration_s") ? number(args.at("--duration_s")) : std::numeric_limits<double>::infinity();
  if (start < 0 || duration <= 0) throw std::runtime_error("Invalid replay time range");
  auto pairs = read_csv(args.at("--stereo_pairs"));
  auto imu = read_csv(args.at("--imu"));
  const double begin = pairs.front().at("timestamp") + start;
  const double end = begin + duration;
  if (begin > pairs.back().at("timestamp")) throw std::runtime_error("Replay starts after session end");
  if (!std::filesystem::is_regular_file(args.at("--config"))) throw std::runtime_error("Missing estimator config");
  auto parser = std::make_shared<ov_core::YamlParser>(args.at("--config"));
  std::string verbosity = "INFO";
  parser->parse_config("verbosity", verbosity);
  ov_core::Printer::setPrintLevel(verbosity);
  ov_msckf::VioManagerOptions params;
  params.print_and_load(parser);
  const std::string time_mode = args.count("--time-offset-mode") ? args.at("--time-offset-mode") : "config";
  if (time_mode != "config" && time_mode != "estimate") throw std::runtime_error("Invalid time-offset mode");
  // Session timing can differ from the calibration recording. Reuse the
  // official offset state instead of hard-coding a correction for one session.
  if (time_mode == "estimate") params.state_options.do_calib_camera_timeoffset = true;
  if (!parser->successful()) throw std::runtime_error("OpenVINS configuration is incomplete");
  if (params.state_options.num_cameras != 2) throw std::runtime_error("Adapter requires two cameras");
  // Offline replay must wait for initialization; otherwise fast replay can end
  // while the initializer is still running in a detached thread.
  params.use_multi_threading_pubs = false;
  params.use_multi_threading_subs = false;
  const auto wall_start = std::chrono::steady_clock::now();
  const std::string bootstrap_mode = args.count("--timing-bootstrap") ? args.at("--timing-bootstrap") : "auto";
  if (bootstrap_mode != "auto" && bootstrap_mode != "off") throw std::runtime_error("Invalid timing-bootstrap mode");
  ov_offline::TimingBootstrap timing;
  timing.configured = timing.candidate = timing.selected = params.calib_camimu_dt;
  if (bootstrap_mode == "auto" && time_mode == "estimate") {
    timing = bootstrap_timing(args.at("--left_video"), pairs, imu, begin, end, params);
    params.calib_camimu_dt = timing.selected;
    params.init_options.calib_camimu_dt = timing.selected;
    std::cout << "[timing-bootstrap] " << timing.reason << " intervals=" << timing.intervals
              << " configured=" << timing.configured << " candidate=" << timing.candidate
              << " selected=" << timing.selected << " error=" << timing.error_before << " -> " << timing.error_after
              << " split_difference=" << timing.split_difference << '\n';
  }
  ov_msckf::VioManager system(params);
  Video left(args.at("--left_video")), right(args.at("--right_video"));
  const std::filesystem::path output(args.at("--output_dir"));
  std::filesystem::create_directories(output);
  if (std::filesystem::exists(output / "states.csv")) throw std::runtime_error("Refusing to overwrite states.csv; use a new output directory");
  std::ofstream states(output / "states.csv");
  std::ofstream diagnostics(output / "frame_status.csv");
  if (!states || !diagnostics) throw std::runtime_error("Cannot create output files");
  states << "timestamp,timestamp_imu,left_frame_index,right_frame_index,px,py,pz,qx,qy,qz,qw,vx,vy,vz,legacy_qx,legacy_qy,legacy_qz,legacy_qw\n" << std::setprecision(17);
  diagnostics << "timestamp,initialized,state_advanced,msckf_features,slam_features,time_offset_s,left_frame_index,right_frame_index,timestamp_imu,status\n" << std::setprecision(17);
  size_t imu_index = 0, processed = 0, poses = 0, unbracketed = 0, published_imu = 0;
  while (imu_index + 1 < imu.size() && imu[imu_index + 1].at("timestamp") < begin + params.calib_camimu_dt - params.init_options.init_window_time - 1.0) ++imu_index;
  for (const auto &pair : pairs) {
    const double timestamp = pair.at("timestamp");
    if (timestamp < begin) continue;
    if (timestamp >= end) break;
    const double timestamp_imu = timestamp + system.get_state()->_calib_dt_CAMtoIMU->value()(0);
    // Match official ROS replay: feed IMU through the first sample strictly
    // after camera time, allowing the propagator to interpolate both endpoints.
    if (timestamp_imu < imu.front().at("timestamp") || timestamp_imu >= imu.back().at("timestamp")) {
      ++unbracketed;
      diagnostics << timestamp << ',' << (system.initialized_time() >= 0) << ",0,0,0,"
                  << system.get_state()->_calib_dt_CAMtoIMU->value()(0) << ','
                  << pair.at("left_frame_index") << ',' << pair.at("right_frame_index") << ','
                  << timestamp_imu << ",imu_unavailable\n";
      continue;
    }
    while (imu_index < imu.size() && (imu_index == 0 || imu[imu_index - 1].at("timestamp") <= timestamp_imu)) {
      const auto &row = imu[imu_index++];
      ov_core::ImuData message;
      message.timestamp = row.at("timestamp");
      message.wm << row.at("angular_velocity_x"), row.at("angular_velocity_y"), row.at("angular_velocity_z");
      message.am << row.at("linear_acceleration_x"), row.at("linear_acceleration_y"), row.at("linear_acceleration_z");
      system.feed_measurement_imu(message);
      ++published_imu;
      if (message.timestamp > timestamp_imu) break;
    }
    ov_core::CameraData message;
    message.timestamp = timestamp;
    message.sensor_ids = {0, 1};
    message.images = {left.frame(pair.at("left_frame_index")), right.frame(pair.at("right_frame_index"))};
    for (const auto &image : message.images) message.masks.push_back(cv::Mat::zeros(image.size(), CV_8UC1));
    system.feed_measurement_camera(message);
    ++processed;
    const auto state = system.get_state();
    // initialized() additionally requires a visual update; a valid ZUPT-only
    // startup has an initialization time but no visual update yet.
    bool initialized = system.initialized_time() >= 0;
    bool advanced = initialized && std::abs(state->_timestamp - timestamp) < 1e-6;
    bool finite = state->_imu->pos().allFinite() && state->_imu->vel().allFinite()
                  && state->_imu->quat().allFinite();
    const char *status = !initialized ? "initializing" : !advanced ? "state_not_advanced"
                         : !finite ? "invalid_state" : "valid";
    diagnostics << timestamp << ',' << initialized << ',' << advanced << ','
                << system.get_good_features_MSCKF().size() << ',' << system.get_features_SLAM().size() << ','
                << state->_calib_dt_CAMtoIMU->value()(0) << ','
                << pair.at("left_frame_index") << ',' << pair.at("right_frame_index") << ','
                << timestamp + state->_calib_dt_CAMtoIMU->value()(0) << ',' << status << '\n';
    if (advanced) {
      // OpenVINS stores R_global_to_imu; export Hamilton quaternion for T_world_imu.
      const Eigen::Quaterniond quaternion(state->_imu->Rot().transpose());
      const Eigen::Vector3d position = state->_imu->pos();
      const Eigen::Vector3d velocity = state->_imu->vel();
      const auto legacy_quaternion = state->_imu->quat();
      if (!position.allFinite() || !quaternion.coeffs().allFinite() || !velocity.allFinite() || !legacy_quaternion.allFinite())
        throw std::runtime_error("Non-finite VIO state");
      states << timestamp << ',' << timestamp + state->_calib_dt_CAMtoIMU->value()(0) << ','
             << pair.at("left_frame_index") << ',' << pair.at("right_frame_index")
             << ',' << position.x() << ',' << position.y() << ',' << position.z()
             << ',' << quaternion.x() << ',' << quaternion.y() << ',' << quaternion.z() << ',' << quaternion.w()
             << ',' << velocity.x() << ',' << velocity.y() << ',' << velocity.z()
             << ',' << legacy_quaternion(0) << ',' << legacy_quaternion(1) << ',' << legacy_quaternion(2) << ',' << legacy_quaternion(3) << '\n';
      ++poses;
    }
  }
  double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
  std::ofstream summary(output / "offline_report.json");
  const std::string status = poses == 0 ? "not_initialized" : (poses < 2 ? "insufficient_poses" : "completed");
  summary << std::setprecision(17) << "{\"status\":\"" << status << "\",\"processed_camera_frames\":" << processed
          << ",\"published_imu_samples\":" << published_imu
          << ",\"pose_count\":" << poses << ",\"unbracketed_camera_frames\":" << unbracketed
          << ",\"wall_duration_s\":" << elapsed
          << ",\"time_offset_estimated\":" << (params.state_options.do_calib_camera_timeoffset ? "true" : "false")
          << ",\"initial_time_offset_s\":" << params.calib_camimu_dt
          << ",\"final_time_offset_s\":" << system.get_state()->_calib_dt_CAMtoIMU->value()(0)
          << ",\"timing_bootstrap\":{\"reason\":\"" << timing.reason << "\",\"applied\":" << (timing.applied ? "true" : "false")
          << ",\"configured_offset_s\":" << timing.configured << ",\"candidate_offset_s\":" << timing.candidate
          << ",\"intervals\":" << timing.intervals << ",\"lookahead_limit_s\":6"
          << ",\"error_before_rad_s\":" << timing.error_before << ",\"error_after_rad_s\":" << timing.error_after
          << ",\"split_difference_s\":" << timing.split_difference << "}}\n";
  std::cout << "VIO status=" << status << " frames=" << processed << " poses=" << poses << '\n';
  return poses >= 2 ? 0 : 3;
}

int main(int argc, char **argv) {
  try { return run(argc, argv); }
  catch (const std::exception &error) { std::cerr << "[VIO ERROR] " << error.what() << '\n'; return 2; }
}
