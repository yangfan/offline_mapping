add_library(frontend FrontEnd.cpp KeyFrame.cpp tools.cpp)
target_link_libraries(frontend sensors lio yaml-cpp::yaml-cpp Eigen3::Eigen Sophus::Sophus rclcpp::rclcpp ${sensor_msgs_TARGETS})
target_include_directories(frontend PUBLIC ${CMAKE_SOURCE_DIR}/include/offline_mapping)
