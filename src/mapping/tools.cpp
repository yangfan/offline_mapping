#include "mapping/tools.h"
#include "sensors/LidarPointType.h"

#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <fstream>
#include <sstream>

namespace mapping {

void merge_keyframes(const std::string &kf_path, const std::string &info_path,
                     const double filter_sz, const POSE_TYPE pose_type) {
  std::vector<std::unique_ptr<KeyFrame>> kfs;
  load_keyframes(info_path, kfs);

  pcl::VoxelGrid<PointType> grid_filter;
  grid_filter.setLeafSize(filter_sz, filter_sz, filter_sz);

  PointCloudPtr map(new PointCloudType);
  for (const auto &kf : kfs) {
    kf->load_scan(kf_path + std::to_string(kf->id) + ".pcd");
    if (kf->scan->empty()) {
      LOG(WARNING) << "Empty scan in Keyframe " << kf->id;
    }
    PointCloudPtr scan_w(new PointCloudType);
    Sophus::SE3d pose;
    switch (pose_type) {
    case POSE_TYPE::lio:
      pose = kf->pose_lio;
      break;
    case POSE_TYPE::opt1:
      pose = kf->pose_opt1;
      break;
    case POSE_TYPE::opt2:
      pose = kf->pose_opt2;
      break;
    default:
      pose = kf->pose_lio;
      break;
    }
    pcl::transformPointCloud(*kf->scan, *scan_w, pose.matrix().cast<float>());

    grid_filter.setInputCloud(scan_w);
    PointCloudPtr scan_map(new PointCloudType);
    grid_filter.filter(*scan_map);

    *map += *scan_map;
  }
  map->height = 1;
  map->width = map->size();
  pcl::io::savePCDFile(kf_path + "map.pcd", *map, true);
  LOG(INFO) << "merged map saved at " << kf_path + "map.pcd";
}

bool save_keyframes(const std::string &kf_path,
                    const std::vector<std::unique_ptr<KeyFrame>> &kfs) {
  std::ofstream ofs(kf_path);
  if (!ofs.is_open()) {
    return false;
  }
  ofs << kfs.size() << std::endl;
  for (const auto &kf : kfs) {
    kf->save(ofs);
  }
  return true;
}

bool load_keyframes(const std::string &kf_path,
                    std::vector<std::unique_ptr<KeyFrame>> &kfs) {
  LOG(INFO) << "loading kfs at " << kf_path;
  std::ifstream ifs(kf_path);
  if (!ifs.is_open()) {
    return false;
  }
  std::string line;
  std::getline(ifs, line);
  const int num = std::stoi(line);
  line.clear();
  kfs.clear();
  kfs.reserve(num);
  LOG(INFO) << "Number of keyframes: " << num;
  std::stringstream ss;
  while (std::getline(ifs, line)) {
    if (line.empty()) {
      continue;
    }
    ss << line;
    kfs.emplace_back(std::make_unique<KeyFrame>());
    kfs.back()->load(ss);
    ss.clear();
  }
  return true;
}

} // namespace mapping