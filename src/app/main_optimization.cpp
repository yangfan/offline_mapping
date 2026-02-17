
#include "mapping/Optimization.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_string(
    config_file,
    "/home/fan/ssd/Projects/ros2_ws/src/offline_mapping/config/mapping.yaml",
    "path of configuration file");
DEFINE_int32(opt_stage, 1, "optimization stage: 1, 2");

int main(int argc, char **argv) {

  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);

  google::ParseCommandLineFlags(&argc, &argv, true);

  Optimization opt(FLAGS_config_file);
  opt.config(FLAGS_opt_stage);
  opt.run();

  return 0;
}