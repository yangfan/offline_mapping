#include "mapping/LoopClosure.h"
// #include "mapping/ndt.h"
#include "mapping/tools.h"

#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <yaml-cpp/yaml.h>

#include <execution>
#include <fstream>

bool LoopClosure::config() {

  try {
    auto yaml = YAML::LoadFile(config_file_);

    kf_path_ = yaml["kf_path"].as<std::string>();
    params_.loop_dist_ = yaml["loop_closure"]["loop_dist"].as<double>();
    params_.loop_id_dist_ = yaml["loop_closure"]["loop_id_dist"].as<int>();
    params_.candidate_id_dist =
        yaml["loop_closure"]["candidate_id_dist"].as<int>();
    params_.ndt_min_score = yaml["loop_closure"]["ndt_min_score"].as<double>();

  } catch (...) {
    LOG(ERROR) << "Unable to read config file at " << config_file_;
    return false;
  }

  LOG(INFO) << "loop dist: " << params_.loop_dist_
            << ", loop id dist: " << params_.loop_id_dist_
            << ", candidate id dist: " << params_.candidate_id_dist
            << ", ndt min score: " << params_.ndt_min_score;

  return true;
}

void LoopClosure::run() {

  mapping::load_keyframes(kf_path_ + "kf_info.txt", keyframes_);

  detect_candidates();

  match_candidates();

  save_candidates();
}

bool LoopClosure::detect_candidates() {
  if (keyframes_.empty()) {
    LOG(WARNING) << "Empty keyframes. Stop detecting loop closure.";
    return false;
  }

  candidates_.clear();
  candidates_.reserve(keyframes_.size());

  KeyFrame *last_cand_i = nullptr;
  KeyFrame *last_cand_j = nullptr;

  for (size_t i = 0; i < keyframes_.size(); ++i) {
    KeyFrame *kf_i = keyframes_[i].get();
    if (last_cand_i && std::abs(int(kf_i->id) - int(last_cand_i->id)) <=
                           params_.candidate_id_dist) {
      continue;
    }
    for (size_t j = i + 1; j < keyframes_.size(); ++j) {
      KeyFrame *kf_j = keyframes_[j].get();
      if (last_cand_j && std::abs(int(kf_j->id) - int(last_cand_j->id)) <=
                             params_.candidate_id_dist) {
        continue;
      }
      if (std::abs(int(kf_i->id) - int(kf_j->id)) <=
          int(params_.loop_id_dist_)) {
        continue;
      }
      if ((kf_i->pose_opt1.translation() - kf_j->pose_opt1.translation())
              .norm() < params_.loop_dist_) {
        candidates_.emplace_back(kf_i->id, kf_j->id,
                                 kf_i->pose_opt1.inverse() * kf_j->pose_opt1);
        last_cand_i = kf_i;
        last_cand_j = kf_j;
      }
    }
  }
  LOG(INFO) << "Detect " << candidates_.size() << " loop candidates.";

  return !candidates_.empty();
}

void LoopClosure::match_candidates() {
  if (candidates_.empty()) {
    LOG(INFO) << "No candidate detected. Stop matching.";
    return;
  }
  std::for_each(std::execution::par_unseq, candidates_.begin(),
                candidates_.end(),
                [this](Candidate &candidate) { grade_candidate(candidate); });

  std::vector<Candidate> loop_matches;
  loop_matches.reserve(candidates_.size());
  for (const auto &candidate : candidates_) {
    if (candidate.ndt_score > params_.ndt_min_score) {
      loop_matches.emplace_back(candidate);
    }
  }
  candidates_.swap(loop_matches);
  return;
}

void LoopClosure::grade_candidate(Candidate &candidate) {

  const int range = 40;
  const KeyFrame *kf_i = keyframes_[candidate.idi].get();
  PointCloudPtr submap(new PointCloudType);

  for (int offset = -range; offset < range; offset += 4) {
    const int kf_id = kf_i->id + offset;
    if (kf_id < 0 || kf_id > int(keyframes_.size())) {
      continue;
    }
    const KeyFrame &kf = *keyframes_[kf_id];
    PointCloudPtr scan(new PointCloudType);
    pcl::io::loadPCDFile(kf_path_ + "pcd/" + std::to_string(kf_id) + ".pcd",
                         *scan);
    RemoveGround(scan, 0.1);
    if (scan->empty()) {
      return;
    }
    PointCloudPtr scan_w(new PointCloudType);
    pcl::transformPointCloud(*scan, *scan_w, kf.pose_opt1.matrix());
    *submap += *scan_w;
  }

  const KeyFrame *kf_j = keyframes_[candidate.idj].get();
  PointCloudPtr cur_scan(new PointCloudType);
  pcl::io::loadPCDFile(kf_path_ + "pcd/" + std::to_string(kf_j->id) + ".pcd",
                       *cur_scan);

  if (submap->empty() || cur_scan->empty()) {
    return;
  }

  pcl::NormalDistributionsTransform<PointType, PointType> ndt;

  ndt.setTransformationEpsilon(0.05);
  ndt.setStepSize(0.7);
  ndt.setMaximumIterations(40);

  Eigen::Matrix<float, 4, 4> T_w_s = kf_j->pose_opt1.matrix().cast<float>();
  const std::vector<double> resolutions = {10.0, 5.0, 4.0, 3.0}; // m / voxel
  PointCloudPtr output(new PointCloudType);

  for (const double res : resolutions) {
    ndt.setResolution(res);
    auto target = VoxelGridFilter(submap, res * 0.1);
    ndt.setInputTarget(target);

    auto source = VoxelGridFilter(cur_scan, res * 0.1);
    ndt.setInputSource(source);

    ndt.align(*output, T_w_s);
    T_w_s = ndt.getFinalTransformation();
  }
  Eigen::Quaterniond quat(T_w_s.block<3, 3>(0, 0).cast<double>());
  quat.normalize();
  const Eigen::Vector3d trans(T_w_s.block<3, 1>(0, 3).cast<double>());

  candidate.Tij = kf_i->pose_opt1.inverse() * Sophus::SE3d(quat, trans);
  candidate.ndt_score = ndt.getTransformationLikelihood();
  LOG(INFO) << "loop " << candidate.idi << ", " << candidate.idj
            << ", score: " << candidate.ndt_score;

  return;
}

PointCloudPtr LoopClosure::VoxelGridFilter(PointCloudPtr input,
                                           const double voxel_size) {
  pcl::VoxelGrid<PointType> voxel;
  voxel.setLeafSize(voxel_size, voxel_size, voxel_size);
  voxel.setInputCloud(input);

  PointCloudPtr output(new PointCloudType);
  voxel.filter(*output);
  return output;
}

void LoopClosure::RemoveGround(PointCloudPtr input, const double height) {
  PointCloudPtr output(new PointCloudType);
  output->reserve(input->size());

  for (const auto &pt : input->points) {
    if (pt.z > height) {
      output->points.emplace_back(pt);
    }
  }
  output->height = 1;
  output->is_dense = false;
  output->width = output->points.size();
  input->swap(*output);
}

void LoopClosure::save_candidates() {

  std::ofstream ofs(kf_path_ + "loop.txt");
  if (!ofs.is_open()) {
    return;
  }
  auto save_pose = [](std::ostream &os, const Sophus::SE3d &pose) {
    const Eigen::Quaterniond quat = pose.so3().unit_quaternion();
    const Eigen::Vector3d translation = pose.translation();
    os << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w()
       << " " << translation.x() << " " << translation.y() << " "
       << translation.z() << " ";
  };
  LOG(INFO) << "number of loop: " << candidates_.size();
  ofs << candidates_.size() << std::endl;
  for (const auto &candidate : candidates_) {
    ofs << candidate.idi << " " << candidate.idj << " " << candidate.ndt_score
        << " ";
    save_pose(ofs, candidate.Tij);
    ofs << std::endl;
  }
}