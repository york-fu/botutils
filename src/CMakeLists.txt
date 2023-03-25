cmake_minimum_required(VERSION 3.0.0)

include_directories(./)
set(SRCS
  ./timer.cc
)
add_library(timer ${SRCS})
target_include_directories(timer PUBLIC ./)

include_directories(./)
set(SRCS
  ./performance_measurement.cc
)
add_library(perf ${SRCS})
target_include_directories(perf PUBLIC ./)
