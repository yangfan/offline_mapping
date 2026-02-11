
add_library(NDT NDT.cpp)
target_link_libraries(NDT PUBLIC Eigen3::Eigen Sophus::Sophus ${PCL_LIBRARIES} TBB::tbb)
target_include_directories(NDT PUBLIC ${CMAKE_SOURCE_DIR}/include/offline_mapping/matching)

add_library(NDT_INC NDT_INC.cpp)
target_link_libraries(NDT_INC PUBLIC Eigen3::Eigen Sophus::Sophus ${PCL_LIBRARIES} TBB::tbb)
target_include_directories(NDT_INC PUBLIC ${CMAKE_SOURCE_DIR}/include/offline_mapping/matching)