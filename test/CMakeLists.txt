cmake_minimum_required(VERSION 3.0.0)
project(test)

find_package(GTest REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})

add_executable(tests 
    ${PROJECT_SOURCE_DIR}/performance_measurement_test.cc
    ${PROJECT_SOURCE_DIR}/rate_test.cc
    ${PROJECT_SOURCE_DIR}/timer_test.cc
    ${PROJECT_SOURCE_DIR}/lowpass_filter_test.cc
    ${PROJECT_SOURCE_DIR}/kalman_filter_test.cc
    ${PROJECT_SOURCE_DIR}/quaternion_estimator_test.cc
)

target_link_libraries(tests ${GTEST_LIBRARIES} gtest_main pthread perf timer filter fusion)

add_test(NAME tests COMMAND tests)