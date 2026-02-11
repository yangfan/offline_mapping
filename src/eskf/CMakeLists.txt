
add_library(ieskf LioIeskf.cpp ieskf.cpp)
target_link_libraries(ieskf tools NDT_INC yaml-cpp::yaml-cpp Eigen3::Eigen Sophus::Sophus ${PCL_LIBRARIES} rclcpp::rclcpp ${sensor_msgs_TARGETS} pcl_conversions::pcl_conversions)
target_include_directories(ieskf PRIVATE ${CMAKE_SOURCE_DIR}/include/offline_mapping)