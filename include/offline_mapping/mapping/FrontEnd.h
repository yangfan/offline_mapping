#pragma once

#include "lio/LioIeskf.h"
#include "mapping/KeyFrame.h"
#include "sensors/BagIO.h"
#include "sensors/gnss.h"

#include <Eigen/Core>
#include <sophus/se3.hpp>

class FrontEnd {
public:
  explicit FrontEnd(const std::string yaml_file) : config_file_(yaml_file) {}

  bool config();
  void run();

private:
  std::unique_ptr<BagIO> bag_player_ = nullptr;
  LioIeskf lio_;
  std::vector<GNSS> gnss_data_;
  std::vector<std::unique_ptr<KeyFrame>> keyframes_;

  std::string lidar_topic_;
  std::string imu_topic_;
  std::string gnss_topic_;

  std::string config_file_;
  std::string kf_path_;

  bool set_origin();

  bool align_kf_gnss();
};