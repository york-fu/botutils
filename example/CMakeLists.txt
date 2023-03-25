cmake_minimum_required(VERSION 3.0.0)
project(example VERSION 0.1.0)

set(SRCS 
  ./example_timer.cpp
)
add_executable(${PROJECT_NAME}_timer ${SRCS})
target_link_libraries(${PROJECT_NAME}_timer timer perf pthread)

set(SRCS 
  ./example_rate.cpp
)
add_executable(${PROJECT_NAME}_rate ${SRCS})
target_link_libraries(${PROJECT_NAME}_rate timer perf pthread)
