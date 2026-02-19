#pragma once

#include "mapping/KeyFrame.h"
#include "mapping/LoopClosure.h"

#include <string>

#include <boost/format.hpp>

namespace mapping {

enum class POSE_TYPE { lio, opt1, opt2 };

void merge_keyframes(const std::string &kf_path, const std::string &info_path,
                     const double filter_sz, const POSE_TYPE pose_type);

bool save_keyframes(const std::string &kf_path,
                    const std::vector<std::unique_ptr<KeyFrame>> &kfs);

bool load_keyframes(const std::string &kf_path,
                    std::vector<std::unique_ptr<KeyFrame>> &kfs);

bool load_loops(const std::string &loop_path,
                std::vector<LoopClosure::Candidate> &loops);

template <typename T>
std::string print_info(const std::vector<T> &edges, double th) {
  std::vector<double> chi2;
  for (auto &edge : edges) {
    if (edge->level() == 0) {
      edge->computeError();
      chi2.push_back(edge->chi2());
    }
  }

  std::sort(chi2.begin(), chi2.end());
  double ave_chi2 =
      std::accumulate(chi2.begin(), chi2.end(), 0.0) / chi2.size();
  boost::format fmt(
      "Num: %d, Mean: %f, Median: %f, 0.1 Quartile: %f, 0.9 Quartile: %f, "
      "0.95 Quartile: %f, Max: %f, Threshold: %f\n");
  if (!chi2.empty()) {
    std::string str =
        (fmt % chi2.size() % ave_chi2 % chi2[chi2.size() / 2] %
         chi2[int(chi2.size() * 0.1)] % chi2[int(chi2.size() * 0.9)] %
         chi2[int(chi2.size() * 0.95)] % chi2.back() % th)
            .str();
    return str;
  }
  return std::string();
}

} // namespace mapping