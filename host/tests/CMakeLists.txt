include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG v1.15.2
)
# Keep gtest's own runtime settings from fighting the parent project's.
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(imu_host_tests
  test_crc16.cpp
  test_frame_parser.cpp
  test_fusion.cpp
  test_latency_tracker.cpp
  ../src/fusion/fusion.cpp
  ../src/network/frame_parser.cpp
)
target_include_directories(imu_host_tests PRIVATE ../include)
target_link_libraries(imu_host_tests PRIVATE gtest_main)

include(GoogleTest)
gtest_discover_tests(imu_host_tests)
