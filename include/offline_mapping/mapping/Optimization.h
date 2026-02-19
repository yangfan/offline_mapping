#pragma once

#include "g2o_types/types.h"
#include "mapping/KeyFrame.h"
#include "mapping/LoopClosure.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <sophus/se3.hpp>

#include <string>

class Optimization {
public:
  struct Param {
    double gnss_outlier_th = 1.0;
    double loop_outlier_th = 5.2;
    double noise_gnss_pos = 2.0;
    double noise_gnss_deg = 5.0;
    double noise_gnss_height = 20.0;
    double noise_lio_pos = 0.01;
    double noise_lio_deg = 0.1;
    double noise_loop_pos = 0.1;
    double noise_loop_deg = 0.5;
    int edge_lio_num = 5;
    int max_iterations = 100;
    int opt_stage = 1;
  };

  explicit Optimization(const std::string &config_file)
      : config_file_(config_file) {}

  bool config(const int stage);
  void run();

  bool save_pose_graph(const std::string &g2o_path) const;
  bool load_pose_graph(const std::string &g2o_path);

private:
  Param params_;

  Sophus::SE3d T_B_G_;
  std::vector<std::unique_ptr<KeyFrame>> keyframes_;
  std::vector<LoopClosure::Candidate> loop_candidates_;

  g2o::SparseOptimizer optimizer_;
  std::vector<VertexSE3 *> vertices_;
  std::vector<EdgeRelSE3 *> edges_lio_;
  std::vector<EdgeGNSSPos *> edges_gnss_;
  std::vector<EdgeRelSE3 *> edges_loop_;

  std::string config_file_;
  std::string kf_path_;

  void icp_align();

  void build_pose_graph();
  void build_vertices();
  void build_lio_edges();
  void build_gnss_edges();
  void build_loop_edges();

  void optimize();
  void update_kfs();

  void remove_outliers();
};