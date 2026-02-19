add_library(mapping FrontEnd.cpp KeyFrame.cpp tools.cpp Optimization.cpp LoopClosure.cpp)
target_link_libraries(mapping sensors g2o_types lio TBB::tbb yaml-cpp::yaml-cpp Eigen3::Eigen Sophus::Sophus rclcpp::rclcpp ${sensor_msgs_TARGETS} ${PCL_LIBRARIES})
target_include_directories(mapping PUBLIC ${CMAKE_SOURCE_DIR}/include/offline_mapping)
