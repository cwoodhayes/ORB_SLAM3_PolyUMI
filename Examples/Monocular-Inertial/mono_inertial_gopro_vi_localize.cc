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
 *       path_to_trajectory_output \
 *       [path_to_reverse_trajectory_output]
 *
 * With the optional 6th argument, the video is decoded once into memory and
 * localized twice against the same loaded atlas: a normal forward pass (written
 * to path_to_trajectory_output) and a second reverse-order pass (written to
 * path_to_reverse_trajectory_output).  Because the run is offline, the reverse
 * pass can recover the lead-in frames the forward pass loses before it manages
 * to relocalize — it reaches them from the well-tracked middle with a motion
 * prior instead of a cold relocalization.  Its trajectory is on a flipped clock
 * (timestamp = t_last - t_video); slam_step.py maps it back and merges.
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

// Run one localization sweep over a pre-decoded frame buffer against a freshly
// loaded atlas, then write the EuRoC trajectory.  ``reversed`` walks the buffer
// back-to-front on a time-reversed clock; the caller passes IMU arrays already
// transformed to match (see main()).  A fresh System per sweep is what gives the
// reverse pass a clean tracker that starts LOST and relocalizes into the loaded
// map, without ORB-SLAM3 needing a "reset-tracking-but-keep-map" entry point.
static void RunLocalizationPass(const char *vocab, const char *settings,
                                const vector<cv::Mat> &frames,
                                const vector<double> &frameTimes,
                                const vector<double> &imuTs,
                                const vector<cv::Point3f> &acc,
                                const vector<cv::Point3f> &gyr,
                                bool reversed, const string &traj_out,
                                const char *tag) {
  // LoadAtlasFromFile in the YAML triggers atlas restoration in the ctor; the
  // Cheng fork already ChangeMap()s the loaded map active, so no restore call.
  ORB_SLAM3::System SLAM(vocab, settings, ORB_SLAM3::System::IMU_MONOCULAR, false);
  // The ctor clamps verbosity to QUIET; re-enable reloc/lost/IMU-init messages.
  ORB_SLAM3::Verbose::SetTh(ORB_SLAM3::Verbose::VERBOSITY_NORMAL);
  // Tracking-only: don't grow the loaded map. Must come after construction.
  SLAM.ActivateLocalizationMode();
  SLAM.PrintLoadedAtlasState(tag);

  const size_t N = frames.size();
  const double t_last = frameTimes[N - 1];
  size_t last_imu_idx = 0;
  std::vector<ORB_SLAM3::IMU::Point> vImuMeas;

  for (size_t k = 0; k < N; ++k) {
    const size_t j = reversed ? (N - 1 - k) : k;  // original frame index
    // Reversed sweep runs on a flipped clock so timestamps still increase.
    const double tframe = reversed ? (t_last - frameTimes[j]) : frameTimes[j];

    vImuMeas.clear();
    while (last_imu_idx < imuTs.size() && imuTs[last_imu_idx] <= tframe && tframe > 0) {
      vImuMeas.push_back(ORB_SLAM3::IMU::Point(
          acc[last_imu_idx].x, acc[last_imu_idx].y, acc[last_imu_idx].z,
          gyr[last_imu_idx].x, gyr[last_imu_idx].y, gyr[last_imu_idx].z,
          imuTs[last_imu_idx]));
      last_imu_idx++;
    }

    SLAM.TrackMonocular(frames[j], tframe, vImuMeas);
    if ((k + 1) % 100 == 0)
      cout << tag << " tracked " << (k + 1) << "/" << N << " frames" << endl;
  }

  SLAM.Shutdown();
  cout << "Saving trajectory to: " << traj_out << endl;
  SLAM.SaveTrajectoryEuRoC(traj_out);
}

int main(int argc, char **argv) {
  if (argc != 6 && argc != 7) {
    cerr << endl
         << "Usage: ./mono_inertial_gopro_vi_localize path_to_vocabulary "
            "path_to_settings path_to_video path_to_telemetry "
            "path_to_trajectory_output [path_to_reverse_trajectory_output]"
         << endl
         << "  With the optional 6th arg, a second reverse-order pass is run "
            "against the same atlas and its trajectory written there; this "
            "offline back-pass recovers lead-in frames the forward pass loses "
            "before it can relocalize.  Decode happens once, in memory."
         << endl;
    return 1;
  }

  const string traj_out = argv[5];
  const bool do_reverse = (argc == 7);
  const string traj_out_reverse = do_reverse ? argv[6] : "";

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

  // Decode + resize every frame once, up front, so the optional reverse pass
  // reuses the buffer instead of re-decoding (or paying an ffmpeg re-encode).
  // Frames are stored at tracking resolution (img_size), bounding the memory.
  vector<cv::Mat> frames;
  vector<double> frameTimes;
  {
    cv::VideoCapture cap(argv[3]);
    if (!cap.isOpened()) {
      cerr << "Error opening video stream or file: " << argv[3] << endl;
      return 1;
    }
    // Fallback frame spacing for sanitizing bad timestamps (see below).
    double fps = cap.get(cv::CAP_PROP_FPS);
    const double dt_fallback = (fps > 1e-6) ? (1.0 / fps) : (1.0 / 60.0);
    int cnt_empty_frame = 0;
    int n_fixed_ts = 0;
    while (true) {
      cv::Mat im;
      if (!cap.read(im)) {
        if (++cnt_empty_frame > 100) break;
        continue;
      }
      cnt_empty_frame = 0;
      double tframe = cap.get(cv::CAP_PROP_POS_MSEC) * MS_TO_S;
      // CAP_PROP_POS_MSEC is unreliable on the last frame (the FFmpeg backend
      // often reports 0 once the demuxer hits EOF), which would poison both the
      // forward "timestamp older than previous" guard and, worse, the reverse
      // pass's t_anchor (= frameTimes.back()).  Enforce a strictly-increasing
      // clock by extrapolating any non-monotonic timestamp from the last good
      // one.  All frames are kept — only the bad timestamp is repaired.
      if (!frameTimes.empty() && tframe <= frameTimes.back()) {
        tframe = frameTimes.back() + dt_fallback;
        n_fixed_ts++;
      }
      cv::Mat resized;
      cv::resize(im, resized, img_size);
      frames.push_back(std::move(resized));
      frameTimes.push_back(tframe);
    }
    if (n_fixed_ts > 0)
      cout << "Repaired " << n_fixed_ts
           << " non-monotonic frame timestamp(s) via " << dt_fallback
           << "s fallback spacing" << endl;
  }
  if (frames.size() < 2) {
    cerr << "Decoded fewer than 2 frames from " << argv[3] << endl;
    return 1;
  }
  cout << "Decoded " << frames.size() << " frames into memory" << endl;

  // Forward pass — identical behaviour to the original single-pass binary.
  RunLocalizationPass(argv[1], argv[2], frames, frameTimes, imuTimestamps, vAcc,
                      vGyr, /*reversed=*/false, traj_out,
                      "[localizer:forward] post-load");

  if (do_reverse) {
    // Time-reversed IMU: reverse sample order, negate gyro (angular velocity
    // flips under t -> -t), keep accelerometer (specific force is invariant),
    // and remap timestamps onto the flipped clock (t' = t_anchor - t) so they
    // still increase.  Mirrors the reversal validated in the Python prototype.
    //
    // The anchor MUST be the same reference RunLocalizationPass uses to flip the
    // frame clock (the last frame time), NOT the last IMU time.  The IMU stream
    // (200 Hz) outlasts the video (60 Hz), so anchoring the IMU at its own end
    // would shift the whole reversed IMU/frame alignment by (imu_end - vid_end).
    // Samples past the last frame get a negative t' and are simply consumed at
    // the first reversed frame, which is where they belong temporally.
    const double t_anchor = frameTimes.back();
    vector<double> imuTsRev;
    vector<cv::Point3f> vAccRev, vGyrRev;
    imuTsRev.reserve(imuTimestamps.size());
    vAccRev.reserve(vAcc.size());
    vGyrRev.reserve(vGyr.size());
    for (size_t i = imuTimestamps.size(); i-- > 0;) {
      imuTsRev.push_back(t_anchor - imuTimestamps[i]);
      vAccRev.push_back(vAcc[i]);
      vGyrRev.push_back(cv::Point3f(-vGyr[i].x, -vGyr[i].y, -vGyr[i].z));
    }
    RunLocalizationPass(argv[1], argv[2], frames, frameTimes, imuTsRev, vAccRev,
                        vGyrRev, /*reversed=*/true, traj_out_reverse,
                        "[localizer:reverse] post-load");
  }

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
