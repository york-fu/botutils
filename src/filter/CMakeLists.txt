cmake_minimum_required(VERSION 3.0.0)

find_package(Eigen3 REQUIRED)

include_directories(./)
set(SRCS
  ./lowpass_filter.cc
  ./kalman_filter.cc
)
add_library(filter ${SRCS})
target_include_directories(filter PUBLIC ./ ${EIGEN3_INCLUDE_DIR})
