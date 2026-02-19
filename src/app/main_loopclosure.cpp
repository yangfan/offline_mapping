#include "mapping/LoopClosure.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_string(
    config_file,
    "/home/fan/ssd/Projects/ros2_ws/src/offline_mapping/config/mapping.yaml",
    "path of configuration file");

int main(int argc, char **argv) {

  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);

  google::ParseCommandLineFlags(&argc, &argv, true);

  LoopClosure loopclosure(FLAGS_config_file);
  loopclosure.config();
  loopclosure.run();

  return 0;
}