add_executable(main_frontend main_frontend.cpp)
target_link_libraries(main_frontend mapping Eigen3::Eigen Sophus::Sophus glog::glog ${GFLAGS_LIBRARIES})
target_include_directories(main_frontend PUBLIC ${CMAKE_SOURCE_DIR}/include/offline_mapping)

add_executable(main_opt main_optimization.cpp)
target_link_libraries(main_opt mapping Eigen3::Eigen Sophus::Sophus glog::glog ${GFLAGS_LIBRARIES})
target_include_directories(main_opt PUBLIC ${CMAKE_SOURCE_DIR}/include/offline_mapping)

add_executable(merge_kfs merge_kfs.cpp)
target_link_libraries(merge_kfs mapping glog::glog ${GFLAGS_LIBRARIES})
target_include_directories(merge_kfs PUBLIC ${CMAKE_SOURCE_DIR}/include/offline_mapping)

add_executable(main_loop main_loopclosure.cpp)
target_link_libraries(main_loop mapping Eigen3::Eigen Sophus::Sophus glog::glog ${GFLAGS_LIBRARIES})
target_include_directories(main_loop PUBLIC ${CMAKE_SOURCE_DIR}/include/offline_mapping)