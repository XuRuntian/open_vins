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

int run(int argc, char **argv) {
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "roboego_openvins_offline --config FILE --left_video FILE --right_video FILE "
                 "--stereo_pairs CSV --imu CSV --output_dir DIR [--start_s 0] [--duration_s SECONDS]\n";
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
        arg.first != "--start_s" && arg.first != "--duration_s") throw std::runtime_error("Unknown argument: " + arg.first);
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
  if (!parser->successful()) throw std::runtime_error("OpenVINS configuration is incomplete");
  if (params.state_options.num_cameras != 2) throw std::runtime_error("Adapter requires two cameras");
  // Offline replay must wait for initialization; otherwise fast replay can end
  // while the initializer is still running in a detached thread.
  params.use_multi_threading_pubs = false;
  params.use_multi_threading_subs = false;
  ov_msckf::VioManager system(params);
  Video left(args.at("--left_video")), right(args.at("--right_video"));
  const std::filesystem::path output(args.at("--output_dir"));
  std::filesystem::create_directories(output);
  if (std::filesystem::exists(output / "states.csv")) throw std::runtime_error("Refusing to overwrite states.csv; use a new output directory");
  std::ofstream states(output / "states.csv");
  std::ofstream diagnostics(output / "frame_status.csv");
  if (!states || !diagnostics) throw std::runtime_error("Cannot create output files");
  states << "timestamp,timestamp_imu,left_frame_index,right_frame_index,px,py,pz,qx,qy,qz,qw\n" << std::setprecision(17);
  diagnostics << "timestamp,initialized,state_advanced,msckf_features,slam_features\n" << std::setprecision(17);
  size_t imu_index = 0, processed = 0, poses = 0, unbracketed = 0;
  while (imu_index + 1 < imu.size() && imu[imu_index + 1].at("timestamp") < begin + params.calib_camimu_dt - params.init_options.init_window_time - 1.0) ++imu_index;
  const auto wall_start = std::chrono::steady_clock::now();
  for (const auto &pair : pairs) {
    const double timestamp = pair.at("timestamp");
    if (timestamp < begin) continue;
    if (timestamp >= end) break;
    const double timestamp_imu = timestamp + system.get_state()->_calib_dt_CAMtoIMU->value()(0);
    // Match official ROS replay: feed IMU through the first sample strictly
    // after camera time, allowing the propagator to interpolate both endpoints.
    if (timestamp_imu < imu.front().at("timestamp") || timestamp_imu >= imu.back().at("timestamp")) {
      ++unbracketed;
      continue;
    }
    while (imu_index < imu.size() && (imu_index == 0 || imu[imu_index - 1].at("timestamp") <= timestamp_imu)) {
      const auto &row = imu[imu_index++];
      ov_core::ImuData message;
      message.timestamp = row.at("timestamp");
      message.wm << row.at("angular_velocity_x"), row.at("angular_velocity_y"), row.at("angular_velocity_z");
      message.am << row.at("linear_acceleration_x"), row.at("linear_acceleration_y"), row.at("linear_acceleration_z");
      system.feed_measurement_imu(message);
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
    diagnostics << timestamp << ',' << initialized << ',' << advanced << ','
                << system.get_good_features_MSCKF().size() << ',' << system.get_features_SLAM().size() << '\n';
    if (advanced) {
      // OpenVINS stores R_global_to_imu; export Hamilton quaternion for T_world_imu.
      const Eigen::Quaterniond quaternion(state->_imu->Rot().transpose());
      const Eigen::Vector3d position = state->_imu->pos();
      if (!position.allFinite() || !quaternion.coeffs().allFinite()) throw std::runtime_error("Non-finite VIO pose");
      states << timestamp << ',' << timestamp_imu << ',' << pair.at("left_frame_index") << ',' << pair.at("right_frame_index")
             << ',' << position.x() << ',' << position.y() << ',' << position.z()
             << ',' << quaternion.x() << ',' << quaternion.y() << ',' << quaternion.z() << ',' << quaternion.w() << '\n';
      ++poses;
    }
  }
  double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall_start).count();
  std::ofstream summary(output / "offline_report.json");
  const std::string status = poses == 0 ? "not_initialized" : (poses < 2 ? "insufficient_poses" : "completed");
  summary << "{\"status\":\"" << status << "\",\"processed_camera_frames\":" << processed
          << ",\"pose_count\":" << poses << ",\"unbracketed_camera_frames\":" << unbracketed
          << ",\"wall_duration_s\":" << elapsed << "}\n";
  std::cout << "VIO status=" << status << " frames=" << processed << " poses=" << poses << '\n';
  return poses >= 2 ? 0 : 3;
}

int main(int argc, char **argv) {
  try { return run(argc, argv); }
  catch (const std::exception &error) { std::cerr << "[VIO ERROR] " << error.what() << '\n'; return 2; }
}
