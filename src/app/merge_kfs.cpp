#include "mapping/tools.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_string(
    data_path,
    "/home/fan/ssd/Projects/ros2_ws/src/offline_mapping/data/output/keyframes/",
    "Path to save map file");
DEFINE_string(info_file, "kf_info.txt", "keyframe info file");
DEFINE_string(pose_type, "lio", "pose type: lio, opt1, opt2");

int main(int argc, char **argv) {

  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);

  google::ParseCommandLineFlags(&argc, &argv, true);

  mapping::POSE_TYPE pose_type = mapping::POSE_TYPE::lio;
  if (FLAGS_pose_type == "opt1") {
    pose_type = mapping::POSE_TYPE::opt1;
  } else if (FLAGS_pose_type == "opt2") {
    pose_type = mapping::POSE_TYPE::opt2;
  }

  mapping::merge_keyframes(FLAGS_data_path + "pcd/",
                           FLAGS_data_path + FLAGS_info_file, 0.1, pose_type);

  return 0;
}