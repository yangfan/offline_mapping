#include "mapping/tools.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <filesystem>

DEFINE_string(
    data_path,
    "/home/fan/ssd/Projects/ros2_ws/src/offline_mapping/data/output/keyframes/",
    "Path to save map file");
DEFINE_string(info_file, "kf_info.txt", "keyframe info file");
DEFINE_double(resolution, 100.0, "submap size");
DEFINE_double(grid_origin, 50.0, "submap size");

int main(int argc, char **argv) {

  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);

  google::ParseCommandLineFlags(&argc, &argv, true);

  const std::string submap_path = FLAGS_data_path + "submaps/";
  try {
    if (!std::filesystem::exists(submap_path)) {
      std::filesystem::create_directories(submap_path);
    }
  } catch (const std::filesystem::filesystem_error &e) {
    LOG(ERROR) << "Unable to create directory" << e.what();
  }

  mapping::partition_map(FLAGS_data_path, FLAGS_data_path + FLAGS_info_file,
                         0.1, FLAGS_resolution,
                         Eigen::Vector2d(FLAGS_grid_origin, FLAGS_grid_origin));

  return 0;
}