add_executable(main_frontend main_frontend.cpp)
target_link_libraries(main_frontend frontend Eigen3::Eigen Sophus::Sophus glog::glog ${GFLAGS_LIBRARIES})
target_include_directories(main_frontend PUBLIC ${CMAKE_SOURCE_DIR}/include/offline_mapping)