/**
 * Localization mode counterpart to mono_inertial_gopro_vi.
 *
 * Loads a pre-built atlas (via System.LoadAtlasFromFile in the settings YAML)
 * and runs ORB-SLAM3 in localization-only mode against a GoPro mp4 + telemetry
 * JSON. Writes the per-frame trajectory in EuRoC format to a caller-specified
 * path so downstream tooling can pick it up without scraping cwd.
 *
 * Usage:
 *   ./mono_inertial_gopro_vi_localize \
 *       path_to_vocabulary \
 *       path_to_settings \
 *       path_to_video \
 *       path_to_telemetry \
 *       path_to_trajectory_output
 *
 * The settings YAML must contain `System.LoadAtlasFromFile: <atlas_path>`.
 * PolyUMI's slam_step.py injects this into a temp copy of the YAML before
 * invoking this binary.
 */

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>

#include <opencv2/core/core.hpp>

#include <System.h>

#include <json.h>

using namespace std;
using nlohmann::json;
const double MS_TO_S = 1e-3;

bool LoadTelemetry(const string &strImuPath,
                   vector<double> &vTimeStamps,
                   vector<cv::Point3f> &vAcc,
                   vector<cv::Point3f> &vGyro);

int main(int argc, char **argv) {
  if (argc != 6) {
    cerr << endl
         << "Usage: ./mono_inertial_gopro_vi_localize path_to_vocabulary path_to_settings path_to_video path_to_telemetry path_to_trajectory_output"
         << endl;
    return 1;
  }

  const string traj_out = argv[5];

  vector<double> imuTimestamps;
  vector<cv::Point3f> vAcc, vGyr;
  if (!LoadTelemetry(argv[4], imuTimestamps, vAcc, vGyr)) {
    cerr << "Failed to load telemetry from: " << argv[4] << endl;
    return 1;
  }

  cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    cerr << "Failed to open settings file at: " << argv[2] << endl;
    return 1;
  }
  cv::Size img_size(fsSettings["Camera.width"], fsSettings["Camera.height"]);
  fsSettings.release();

  // Create the SLAM system; LoadAtlasFromFile in the YAML triggers atlas
  // restoration during System construction.
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, false);

  // The Cheng fork's System constructor already calls
  // mpAtlas->ChangeMap(map_vector.at(0)) after LoadAtlas(), so the loaded map
  // is the current/active map by the time we get here — no extra restore call
  // needed (vanilla ORB-SLAM3 needs one because it leaves CreateNewMap() live
  // after the load, making a fresh empty map the active one).

  // Tracking-only: don't grow the loaded map. Must come after construction.
  SLAM.ActivateLocalizationMode();

  cv::VideoCapture cap(argv[3]);
  if (!cap.isOpened()) {
    cerr << "Error opening video stream or file: " << argv[3] << endl;
    return 1;
  }

  vector<float> vTimesTrack;
  std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
  size_t last_imu_idx = 0;
  int cnt_empty_frame = 0;
  int img_id = 0;
  double fps = cap.get(cv::CAP_PROP_FPS);
  double frame_diff_s = 1. / fps;

  while (1) {
    cv::Mat im, im_track;
    bool success = cap.read(im);
    if (!success) {
      cnt_empty_frame++;
      if (cnt_empty_frame > 100) break;
      continue;
    }
    im_track = im.clone();
    double tframe = cap.get(cv::CAP_PROP_POS_MSEC) * MS_TO_S;
    ++img_id;

    cv::resize(im_track, im_track, img_size);

    vImuMeas.clear();
    while (last_imu_idx < imuTimestamps.size()
           && imuTimestamps[last_imu_idx] <= tframe
           && tframe > 0) {
      vImuMeas.push_back(ORB_SLAM3::IMU::Point(
          vAcc[last_imu_idx].x, vAcc[last_imu_idx].y, vAcc[last_imu_idx].z,
          vGyr[last_imu_idx].x, vGyr[last_imu_idx].y, vGyr[last_imu_idx].z,
          imuTimestamps[last_imu_idx]));
      last_imu_idx++;
    }

    auto t1 = std::chrono::steady_clock::now();
    SLAM.TrackMonocular(im_track, tframe, vImuMeas);
    auto t2 = std::chrono::steady_clock::now();

    double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
    if (img_id % 100 == 0) {
      cout << "Video FPS: " << 1. / frame_diff_s
           << "  ORB-SLAM3 FPS: " << 1. / ttrack << endl;
    }
    vTimesTrack.push_back(ttrack);

    if (ttrack < frame_diff_s)
      usleep((frame_diff_s - ttrack) * 1e6);
  }

  SLAM.Shutdown();

  cout << "Saving trajectory to: " << traj_out << endl;
  SLAM.SaveTrajectoryEuRoC(traj_out);

  return 0;
}

bool LoadTelemetry(const string &path_to_telemetry_file,
                   vector<double> &vTimeStamps,
                   vector<cv::Point3f> &vAcc,
                   vector<cv::Point3f> &vGyro) {
  std::ifstream file(path_to_telemetry_file.c_str());
  if (!file.is_open()) return false;

  json j;
  file >> j;
  const auto accl = j["1"]["streams"]["ACCL"]["samples"];
  const auto gyro = j["1"]["streams"]["GYRO"]["samples"];

  std::map<double, cv::Point3f> sorted_acc;
  std::map<double, cv::Point3f> sorted_gyr;
  for (const auto &e : accl) {
    cv::Point3f v((float)e["value"][1], (float)e["value"][2], (float)e["value"][0]);
    sorted_acc.insert(std::make_pair((double)e["cts"] * MS_TO_S, v));
  }
  for (const auto &e : gyro) {
    cv::Point3f v((float)e["value"][1], (float)e["value"][2], (float)e["value"][0]);
    sorted_gyr.insert(std::make_pair((double)e["cts"] * MS_TO_S, v));
  }

  double imu_start_t = sorted_acc.begin()->first;
  for (auto acc : sorted_acc) {
    vTimeStamps.push_back(acc.first - imu_start_t);
    vAcc.push_back(acc.second);
  }
  for (auto gyr : sorted_gyr) {
    vGyro.push_back(gyr.second);
  }
  return true;
}
