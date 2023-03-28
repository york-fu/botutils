cmake_minimum_required(VERSION 3.0.0)

find_package(Eigen3 REQUIRED)

include_directories(./)
set(SRCS
  ./quaternion_estimator.cc
)
add_library(fusion ${SRCS})
target_include_directories(fusion PUBLIC ./ ${EIGEN3_INCLUDE_DIR})
