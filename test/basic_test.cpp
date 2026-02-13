#include "mapping/tools.h"
#include "sensors/BagIO.h"
#include "sensors/gnss.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

DEFINE_string(bag_file,
              "/home/fan/ssd/Projects/ros2_ws/data/bags/NCLT/ros2_0110",
              "path to ros2bag file");
// DEFINE_string(
//     config_file,
//     "/home/fan/ssd/Projects/ros2_ws/src/lio_preint/config/velodyne_nclt.yaml",
//     "path of configuration file");

DEFINE_string(cloud_topic_name, "points_raw", "topic name of pointcloud data ");
DEFINE_string(imu_topic_name, "imu_raw", "topic name of imu data");
DEFINE_string(gnss_topic_name, "gps_rtk_fix", "topic name of imu data");

DEFINE_string(
    data_path,
    "/home/fan/ssd/Projects/ros2_ws/src/offline_mapping/data/output/keyframes/",
    "Path to save map file");

// TEST(BASIC, IO) {
//   BagIO bag_io(FLAGS_bag_file);

//   Eigen::Vector3d origin = Eigen::Vector3d::Zero();
//   bool set_origin = false;

//   bag_io
//       .AddGNSSHandle(FLAGS_gnss_topic_name,
//                      [&origin, &set_origin](
//                          std::unique_ptr<sensor_msgs::msg::NavSatFix> msg) {
//                        GNSS data(*msg);
//                        EXPECT_TRUE(data.convert_utm());

//                        if (!set_origin && data.status() == GNSS::Status::FIX)
//                        {
//                          origin = data.utm();
//                          set_origin = true;
//                          LOG(INFO) << "origin: " << origin.transpose();
//                        }

//                        //  if (data.status() == GNSS::Status::FIX) {
//                        //    LOG(INFO)
//                        //        << "Pos: " << (data.utm() -
//                        //        origin).transpose();
//                        //  } else {
//                        //    LOG(INFO) << "invalid data";
//                        //  }
//                        //  if (data.status() == GNSS::Status::NO_FIX) {
//                        //    LOG(WARNING) << "no fix.";
//                        //  }

//                        return true;
//                      })
//       .Process();
//   LOG(INFO) << "origin: " << origin.transpose();
// }

TEST(BACIS, Map) {
  mapping::merge_keyframes(FLAGS_data_path, 0.1, mapping::POSE_TYPE::lio);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);

  FLAGS_stderrthreshold = google::INFO;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);

  google::ParseCommandLineFlags(&argc, &argv, true);

  return RUN_ALL_TESTS();
}