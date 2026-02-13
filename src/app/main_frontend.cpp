#include "mapping/FrontEnd.h"

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

  FrontEnd frontend(FLAGS_config_file);
  frontend.config();

  frontend.run();

  return 0;
}