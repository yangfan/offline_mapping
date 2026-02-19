#pragma once

#include "mapping/KeyFrame.h"
#include "sensors/LidarPointType.h"

#include <Eigen/Core>
#include <sophus/se3.hpp>

#include <string>

class LoopClosure {
public:
  struct Param {
    double loop_dist_ = 30.0;
    int loop_id_dist_ = 50;
    int candidate_id_dist = 5;
    double ndt_min_score = 2.5;
    size_t ndt_submap_radius = 40;
  };
  struct Candidate {
    Candidate() = default;
    Candidate(const unsigned long i, const unsigned long j,
              const Sophus::SE3d &T_ij)
        : idi(i), idj(j), Tij(T_ij) {}
    unsigned long idi = 0;
    unsigned long idj = 0;
    Sophus::SE3d Tij;
    double ndt_score = 0.0;
  };
  explicit LoopClosure(const std::string &config) : config_file_(config) {}

  bool config();
  void run();

private:
  Param params_;
  std::vector<std::unique_ptr<KeyFrame>> keyframes_;
  std::vector<Candidate> candidates_;

  std::string config_file_;
  std::string kf_path_;

  bool detect_candidates();
  void match_candidates();
  void grade_candidate(Candidate &candidate);
  void save_candidates();

  PointCloudPtr VoxelGridFilter(PointCloudPtr input, const double voxel_size);
  void RemoveGround(PointCloudPtr input, const double height);
};