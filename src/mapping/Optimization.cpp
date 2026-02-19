#include "mapping/Optimization.h"
#include "mapping/tools.h"

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <fstream>

bool Optimization::config(const int stage) {

  params_.opt_stage = stage;
  try {
    auto yaml = YAML::LoadFile(config_file_);

    params_.gnss_outlier_th = yaml["gnss_outlier_th"].as<double>();
    params_.noise_gnss_pos = yaml["gnss_pos_noise"].as<double>();
    params_.noise_gnss_deg = yaml["gnss_ang_noise"].as<double>();
    params_.noise_gnss_height =
        yaml["gnss_height_noise_ratio"].as<double>() * params_.noise_gnss_pos;
    params_.edge_lio_num = yaml["lidar_continuous_num"].as<int>();
    params_.max_iterations = yaml["max_iterations"].as<int>();
    params_.loop_outlier_th = yaml["loop_outlier_th"].as<double>();
    kf_path_ = yaml["kf_path"].as<std::string>();

    const std::vector<double> gnss_translate =
        yaml["gnss_ext"]["t"].as<std::vector<double>>();
    T_B_G_ =
        Sophus::SE3d(Sophus::SO3d(),
                     Eigen::Map<const Eigen::Vector3d>(gnss_translate.data()));

  } catch (...) {
    LOG(ERROR) << "Unable to read config file.";
    return false;
  }

  return true;
}

void Optimization::run() {
  if (!mapping::load_keyframes(kf_path_ + "kf_info.txt", keyframes_)) {
    LOG(ERROR) << "Unable to load keyframes.";
    return;
  }
  if (params_.opt_stage == 1) {
    icp_align();
  }

  build_pose_graph();

  optimize();

  remove_outliers();

  optimize();

  LOG(INFO) << "after opt:";
  LOG(INFO) << "lio error: " << mapping::print_info(edges_lio_, 0);
  LOG(INFO) << "gnss error: "
            << mapping::print_info(edges_gnss_, params_.gnss_outlier_th);
  if (params_.opt_stage == 2) {
    LOG(INFO) << "loop closure error: "
              << mapping::print_info(edges_loop_, params_.loop_outlier_th);
  }

  update_kfs();

  mapping::save_keyframes(kf_path_ + "kf_info.txt", keyframes_);
  save_pose_graph(kf_path_ + "pose_graph.g2o");
  // load_pose_graph(kf_path_ + "g2o_opt1.g2o");
}

// goal: pos_G = T_G_Lio * pos_Lio
void Optimization::icp_align() {
  std::vector<Eigen::Vector3d> gpts;
  std::vector<Eigen::Vector3d> lpts;
  gpts.reserve(keyframes_.size());
  lpts.reserve(keyframes_.size());

  for (const auto &kf : keyframes_) {
    if (kf->valid_gnss && std::abs(kf->pos_gnss.z()) < 20.0) {
      gpts.emplace_back(kf->pos_gnss);
      lpts.emplace_back((kf->pose_lio).translation());
    }
  }
  const Eigen::Vector3d mean_gnss =
      std::accumulate(gpts.begin(), gpts.end(),
                      Eigen::Vector3d::Zero().eval()) /
      gpts.size();
  const Eigen::Vector3d mean_lio =
      std::accumulate(lpts.begin(), lpts.end(),
                      Eigen::Vector3d::Zero().eval()) /
      lpts.size();

  Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
  for (size_t i = 0; i < gpts.size(); ++i) {
    const Eigen::Vector3d gpt = gpts[i] - mean_gnss;
    const Eigen::Vector3d lpt = lpts[i] - mean_lio;
    W += gpt * lpt.transpose();
  }
  Eigen::JacobiSVD svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();
  if (R.determinant() < 0) {
    R = -R;
  }
  Eigen::Vector3d t = mean_gnss - R * mean_lio;
  const Sophus::SE3d T_G_L(R, t);
  std::for_each(keyframes_.begin(), keyframes_.end(),
                [&T_G_L](std::unique_ptr<KeyFrame> &kf) {
                  kf->pose_lio = T_G_L * kf->pose_lio;
                });
}

void Optimization::build_pose_graph() {
  if (keyframes_.empty()) {
    LOG(WARNING) << "No keyframes.";
    return;
  }
  using BlockSolverType = g2o::BlockSolverX;
  using LinearSolverType =
      g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;

  auto solver = new g2o::OptimizationAlgorithmLevenberg(
      std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));

  optimizer_.setAlgorithm(solver);

  build_vertices();
  build_lio_edges();
  build_gnss_edges();

  if (params_.opt_stage == 2 &&
      mapping::load_loops(kf_path_ + "loop.txt", loop_candidates_)) {
    build_loop_edges();
  }
}

void Optimization::build_vertices() {
  vertices_.clear();
  vertices_.reserve(keyframes_.size());
  for (const auto &kf : keyframes_) {
    auto vertex = new VertexSE3();
    vertex->setId(kf->id);
    if (params_.opt_stage == 1) {
      vertex->setEstimate(kf->pose_lio);
    } else if (params_.opt_stage == 2) {
      vertex->setEstimate(kf->pose_opt1);
    }
    vertices_.emplace_back(vertex);
    optimizer_.addVertex(vertex);
  }
}

void Optimization::build_lio_edges() {
  const double deg2rad = M_PI / 180.0;
  Eigen::Matrix<double, 6, 6> info = Eigen::Matrix<double, 6, 6>::Zero();
  const double var_pos_inv =
      1.0 / (params_.noise_lio_pos * params_.noise_lio_pos);
  const double var_rad_inv =
      1.0 / (params_.noise_lio_deg * deg2rad * params_.noise_lio_deg * deg2rad);
  info.diagonal() << var_pos_inv, var_pos_inv, var_pos_inv, var_rad_inv,
      var_rad_inv, var_rad_inv;

  edges_lio_.clear();
  edges_lio_.reserve(keyframes_.size() * params_.edge_lio_num);
  for (size_t vid = 0; vid < vertices_.size(); ++vid) {
    auto vertex_i = vertices_[vid];
    for (int inc = 1; inc <= params_.edge_lio_num; ++inc) {
      const size_t vj = vid + inc;
      if (vj >= vertices_.size()) {
        break;
      }
      auto vertex_j = vertices_[vj];
      auto edge = new EdgeRelSE3();
      edge->setId(edges_lio_.size());
      edge->setVertex(0, vertex_i);
      edge->setVertex(1, vertex_j);
      edge->setMeasurement(vertex_i->estimate().inverse() *
                           vertex_j->estimate());
      edge->setInformation(info);
      edges_lio_.emplace_back(edge);
      optimizer_.addEdge(edge);
    }
  }
}

void Optimization::build_gnss_edges() {
  Eigen::Matrix3d info = Eigen::Matrix3d::Zero();
  const double var_pos_inv =
      1.0 / (params_.noise_gnss_pos * params_.noise_gnss_pos);
  const double var_ht_inv =
      1.0 / (params_.noise_gnss_height * params_.noise_gnss_height);
  info.diagonal() << var_pos_inv, var_pos_inv, var_ht_inv;
  if (params_.opt_stage == 2) {
    info *= 0.01;
  }

  edges_gnss_.clear();
  edges_gnss_.reserve(keyframes_.size());
  for (const auto &kf : keyframes_) {
    if (kf->valid_gnss) {
      auto edge = new EdgeGNSSPos(T_B_G_);
      edge->setId(edges_lio_.size() + edges_gnss_.size());
      edge->setVertex(0, vertices_[kf->id]);
      edge->setMeasurement(kf->pos_gnss);
      edge->setInformation(info);
      auto rk = new g2o::RobustKernelCauchy();
      rk->setDelta(params_.gnss_outlier_th);
      edge->setRobustKernel(rk);
      edges_gnss_.emplace_back(edge);
      optimizer_.addEdge(edge);
    }
  }
}

void Optimization::build_loop_edges() {
  if (loop_candidates_.empty() || vertices_.empty()) {
    LOG(WARNING) << "Empty loop candidate.";
    return;
  }
  const double deg2rad = M_PI / 180.0;
  Eigen::Matrix<double, 6, 6> info = Eigen::Matrix<double, 6, 6>::Zero();
  const double var_pos_inv =
      1.0 / (params_.noise_loop_pos * params_.noise_loop_pos);
  const double var_rad_inv = 1.0 / (params_.noise_loop_deg * deg2rad *
                                    params_.noise_loop_deg * deg2rad);
  info.diagonal() << var_pos_inv, var_pos_inv, var_pos_inv, var_rad_inv,
      var_rad_inv, var_rad_inv;

  edges_loop_.clear();
  edges_loop_.reserve(loop_candidates_.size());
  size_t eid = edges_lio_.size() + edges_gnss_.size();
  for (const auto &candidate : loop_candidates_) {
    EdgeRelSE3 *edge = new EdgeRelSE3();
    VertexSE3 *v0 = vertices_[candidate.idi];
    VertexSE3 *v1 = vertices_[candidate.idj];
    edge->setVertex(0, v0);
    edge->setVertex(1, v1);
    edge->setId(eid++);
    edge->setMeasurement(candidate.Tij);
    edge->setInformation(info);
    auto rk = new g2o::RobustKernelCauchy();
    rk->setDelta(params_.loop_outlier_th);
    edge->setRobustKernel(rk);
    edges_loop_.emplace_back(edge);
    optimizer_.addEdge(edge);
  }
}

// pose graph for g2o viewer only
bool Optimization::save_pose_graph(const std::string &g2o_path) const {
  std::ofstream ofs(g2o_path);
  if (!ofs.is_open()) {
    LOG(WARNING) << "g2o file path " << g2o_path << " is not valid.";
    return false;
  }
  LOG(INFO) << "Saving pose graph.";
  LOG(INFO) << "Number of vertices: " << optimizer_.vertices().size()
            << ", number of edges: " << optimizer_.edges().size();
  for (const auto v : vertices_) {
    v->write(ofs);
  }
  for (const auto &e : edges_lio_) {
    e->write(ofs);
  }
  for (const auto &e : edges_loop_) {
    if (e->level() == 0) {
      e->write(ofs);
    }
  }
  // for (const auto &e : edges_gnss_) {
  //   e->write(ofs);
  // }
  return true;
}

bool Optimization::load_pose_graph(const std::string &g2o_path) {
  std::ifstream ifs(g2o_path);
  if (!ifs.is_open()) {
    LOG(WARNING) << "g2o file path " << g2o_path << " is not valid.";
    return false;
  }
  optimizer_.clear();
  LOG(INFO) << "Loading pose graph.";
  optimizer_.load(ifs);
  LOG(INFO) << "Number of vertices: " << optimizer_.vertices().size()
            << ", number of edges: " << optimizer_.edges().size();
  return true;
}

void Optimization::optimize() {
  optimizer_.setVerbose(true);
  optimizer_.initializeOptimization();
  optimizer_.optimize(params_.max_iterations);
}

void Optimization::remove_outliers() {
  size_t num_outliers = 0;
  auto remove_outlier = [&num_outliers](g2o::OptimizableGraph::Edge *edge) {
    if (edge->chi2() > edge->robustKernel()->delta()) {
      edge->setLevel(1);
      num_outliers++;
    } else {
      edge->setRobustKernel(nullptr);
    }
  };
  std::for_each(edges_gnss_.begin(), edges_gnss_.end(), remove_outlier);
  LOG(INFO) << "Number of gnss outliers: " << num_outliers << " / "
            << edges_gnss_.size();

  num_outliers = 0;
  std::for_each(edges_loop_.begin(), edges_loop_.end(), remove_outlier);
  LOG(INFO) << "Number of loop closure outliers: " << num_outliers << " / "
            << edges_loop_.size();
}

void Optimization::update_kfs() {
  for (auto &kf : keyframes_) {
    // T_W_G * T_G_B = T_W_B
    if (params_.opt_stage == 1) {
      kf->pose_opt1 = vertices_[kf->id]->estimate();
    } else {
      kf->pose_opt2 = vertices_[kf->id]->estimate();
    }
  }
}