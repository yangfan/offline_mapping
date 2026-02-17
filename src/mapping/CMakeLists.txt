add_library(mapping FrontEnd.cpp KeyFrame.cpp tools.cpp Optimization.cpp)
target_link_libraries(mapping sensors g2o_types lio yaml-cpp::yaml-cpp Eigen3::Eigen Sophus::Sophus rclcpp::rclcpp ${sensor_msgs_TARGETS})
target_include_directories(mapping PUBLIC ${CMAKE_SOURCE_DIR}/include/offline_mapping)
