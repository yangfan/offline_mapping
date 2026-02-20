#include "mapping/tools.h"
#include "sensors/LidarPointType.h"

#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <fstream>
#include <sstream>
#include <unordered_map>

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
      continue;
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

bool load_loops(const std::string &loop_path,
                std::vector<LoopClosure::Candidate> &loops) {
  LOG(INFO) << "loading loop candidates.";
  std::ifstream ifs(loop_path);
  if (!ifs.is_open()) {
    LOG(INFO) << "Unable to read loop file at " << loop_path;
    return false;
  }
  auto read_pose = [](std::stringstream &ss) {
    double data[7];
    for (size_t i = 0; i < 7; ++i) {
      ss >> data[i];
    }
    return Sophus::SE3d(Eigen::Quaterniond(data[3], data[0], data[1], data[2]),
                        Eigen::Vector3d(data[4], data[5], data[6]));
  };
  std::string line;
  std::getline(ifs, line);
  int num_loop = std::stoi(line);
  line.clear();

  loops.clear();
  loops.reserve(num_loop);

  std::stringstream ss;

  while (std::getline(ifs, line)) {
    if (line.empty()) {
      continue;
    }

    int idi = 0, idj = 0;
    double score = 0.0;
    ss << line;
    ss >> idi >> idj >> score;
    const Sophus::SE3d T_i_j = read_pose(ss);

    LoopClosure::Candidate candidate(idi, idj, T_i_j);
    candidate.ndt_score = score;
    loops.emplace_back(candidate);

    ss.clear();
    line.clear();
  }

  return true;
}

void partition_map(const std::string &kf_path, const std::string &info_path,
                   const double filter_sz, const double resolution) {

  std::vector<std::unique_ptr<KeyFrame>> kfs;
  load_keyframes(info_path, kfs);

  pcl::VoxelGrid<PointType> voxel_filter;
  voxel_filter.setLeafSize(filter_sz, filter_sz, filter_sz);

  std::unordered_map<Eigen::Vector2i, PointCloudPtr, hash_pt2> submaps;

  auto sub_id = [&resolution](const double x, const double y) {
    return Eigen::Vector2i(int(std::floor((x - 50.0) / resolution)),
                           int(std::floor((y - 50.0) / resolution)));
  };

  for (auto &kf : kfs) {
    kf->load_scan(kf_path + "pcd/" + std::to_string(kf->id) + ".pcd");
    if (kf->scan->empty()) {
      LOG(WARNING) << "Empty scan in Keyframe " << kf->id;
      continue;
    }
    PointCloudPtr scan_w(new PointCloudType);
    pcl::transformPointCloud(*kf->scan, *scan_w, kf->pose_opt2.matrix());
    voxel_filter.setInputCloud(scan_w);
    PointCloudPtr scan(new PointCloudType);
    voxel_filter.filter(*scan);

    for (const auto &pt : scan_w->points) {
      const Eigen::Vector2i sid = sub_id(pt.x, pt.y);
      auto it = submaps.find(sid);
      if (it == submaps.end()) {
        PointCloudPtr submap(new PointCloudType);
        submap->points.emplace_back(pt);
        submap->is_dense = false;
        submap->height = 1;
        submaps.insert({sid, submap});
      } else {
        submaps[sid]->points.emplace_back(pt);
      }
    }
  }

  std::ofstream ofs(kf_path + "submaps/submaps_info.txt");
  ofs << submaps.size() << std::endl;

  for (const auto &[sid, submap] : submaps) {
    ofs << sid.x() << " " << sid.y() << std::endl;

    voxel_filter.setInputCloud(submap);
    PointCloudPtr cloud(new PointCloudType);
    voxel_filter.filter(*cloud);

    cloud->height = 1;
    cloud->width = cloud->size();
    pcl::io::savePCDFile(kf_path + "submaps/" + std::to_string(sid.x()) + '_' +
                             std::to_string(sid.y()) + ".pcd",
                         *cloud, true);
  }
}

} // namespace mapping