#pragma once

#include "mapping/KeyFrame.h"

#include <string>

namespace mapping {

enum class POSE_TYPE { lio, opt1, opt2 };

void merge_keyframes(const std::string &kf_path, const double filter_sz,
                     const POSE_TYPE pose_type);

bool save_keyframes(const std::string &kf_path,
                    const std::vector<std::unique_ptr<KeyFrame>> &kfs);

bool load_keyframes(const std::string &kf_path,
                    std::vector<std::unique_ptr<KeyFrame>> &kfs);

} // namespace mapping