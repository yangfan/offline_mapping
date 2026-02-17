#include "mapping/FrontEnd.h"
#include "mapping/tools.h"

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>

bool FrontEnd::config() {

  try {
    auto yaml = YAML::LoadFile(config_file_);
    bag_player_ = std::make_unique<BagIO>(yaml["bag_path"].as<std::string>());

    lio_.config(yaml["lio_yaml"].as<std::string>());

    lidar_topic_ = yaml["common"]["lid_topic"].as<std::string>();
    imu_topic_ = yaml["common"]["imu_topic"].as<std::string>();
    gnss_topic_ = yaml["common"]["gnss_topic"].as<std::string>();
    kf_path_ = yaml["kf_path"].as<std::string>();

  } catch (...) {
    LOG(ERROR) << "Unable to read config file.";
    return false;
  }

  const std::string scan_path = kf_path_ + "pcd/";
  try {
    if (!std::filesystem::exists(kf_path_)) {
      std::filesystem::create_directories(kf_path_);
    }
    if (!std::filesystem::exists(scan_path)) {
      std::filesystem::create_directories(scan_path);
    }
  } catch (const std::filesystem::filesystem_error &e) {
    LOG(ERROR) << "Unable to create directory" << e.what();
  }

  return true;
}

void FrontEnd::run() {
  if (!bag_player_) {
    return;
  }

  LOG(INFO) << "Reading GNSS data.";
  bag_player_
      ->AddGNSSHandle(
          gnss_topic_,
          [this](std::unique_ptr<sensor_msgs::msg::NavSatFix> gnss_msg) {
            gnss_data_.emplace_back(*gnss_msg);
            return true;
          })
      .Process();

  bag_player_->clearHandles();

  LOG(INFO) << "Setting origin of GNSS data.";
  set_origin();

  LOG(INFO) << "Creating Keyframes from LIO.";
  bag_player_
      ->AddPointCloudHandle(
          lidar_topic_,
          [this](std::unique_ptr<sensor_msgs::msg::PointCloud2> cloud_msg) {
            lio_.add_scan(std::move(cloud_msg));

            if (lio_.keyframe_num() > keyframes_.size()) {
              keyframes_.emplace_back(std::make_unique<KeyFrame>(
                  lio_.keyframe_timestamp(), lio_.keyframe_num() - 1,
                  lio_.keyframe_pose(), lio_.keyframe_scan()));

              auto kf = keyframes_.back().get();
              if (kf->scan->empty()) {
                LOG(WARNING) << "empty scan.";
              }
              kf->save_scan(kf_path_ + "pcd/" + std::to_string(kf->id) +
                            ".pcd");

              kf->scan = nullptr;
              LOG(INFO)
                  << "Creating Keyframe: " << keyframes_.back()->id << " at "
                  << keyframes_.back()->pose_lio.translation().transpose();
            }
            return true;
          })
      .AddIMUHandle(imu_topic_,
                    [this](std::unique_ptr<sensor_msgs::msg::Imu> imu_msg) {
                      lio_.add_imu(std::move(imu_msg));
                      return true;
                    })
      .Process();

  align_kf_gnss();
  LOG(INFO) << "Saving keyframe info at" << kf_path_ + "kf_info.txt";
  mapping::save_keyframes(kf_path_ + "kf_info.txt", keyframes_);
}

bool FrontEnd::set_origin() {
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  bool find_origin = false;
  for (const auto &gnss : gnss_data_) {
    if (gnss.status() == GNSS::Status::FIX) {
      origin = gnss.utm();
      find_origin = true;
      break;
    }
  }
  if (!find_origin) {
    return false;
  }
  for (auto &gnss : gnss_data_) {
    gnss.set_pos(gnss.utm() - origin);
  }
  auto yaml = YAML::LoadFile(config_file_);
  std::vector<double> ori = {origin.x(), origin.y(), origin.z()};
  yaml["origin"] = ori;

  std::ofstream ofs(config_file_);
  ofs << yaml;

  LOG(INFO) << "Setting GNSS to origin " << origin.transpose();

  return true;
}

bool FrontEnd::align_kf_gnss() {
  if (gnss_data_.empty() || keyframes_.empty()) {
    return false;
  }

  std::sort(keyframes_.begin(), keyframes_.end(),
            [](const std::unique_ptr<KeyFrame> &a,
               const std::unique_ptr<KeyFrame> &b) {
              return a->timestamp < b->timestamp;
            });

  std::sort(gnss_data_.begin(), gnss_data_.end(),
            [](const GNSS &a, const GNSS &b) {
              return a.timestamp() < b.timestamp();
            });

  size_t gid_end = 0;
  const double last_gnss_time = gnss_data_.back().timestamp();
  const double first_gnss_time = gnss_data_.begin()->timestamp();

  for (const auto &kf : keyframes_) {
    if (kf->timestamp > last_gnss_time) {
      LOG(WARNING)
          << "All GNSS data timestamp are earlier than keyframe timestamp. "
             "Stop alignment";
      break;
    }
    if (kf->timestamp < first_gnss_time) {
      LOG(WARNING) << "All GNSS data timestamp are later than keyframe "
                      "timestamp. Skip current keyframe.";
      continue;
    }
    while (gnss_data_[gid_end].timestamp() < kf->timestamp) {
      gid_end++;
    }

    const Eigen::Vector3d &gnss_pos_begin = gnss_data_[gid_end - 1].pos();
    const Eigen::Vector3d &gnss_pos_end = gnss_data_[gid_end].pos();
    const double ratio =
        (kf->timestamp - gnss_data_[gid_end - 1].timestamp()) /
        (gnss_data_[gid_end].timestamp() - gnss_data_[gid_end - 1].timestamp());
    kf->pos_gnss = (1 - ratio) * gnss_pos_begin + ratio * gnss_pos_end;
    kf->valid_gnss = true;
  }

  return true;
}