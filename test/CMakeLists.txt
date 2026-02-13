
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG main
)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(basic_test basic_test.cpp)
target_link_libraries(basic_test frontend sensors gtest glog::glog ${GFLAGS_LIBRARIES})
target_include_directories(basic_test PUBLIC ${CMAKE_SOURCE_DIR}/include/offline_mapping)

include(GoogleTest)
gtest_discover_tests(basic_test)