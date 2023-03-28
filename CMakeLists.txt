cmake_minimum_required(VERSION 3.0.0)
project(botkit VERSION 0.1.0)

option(EXAMPLE "build example" OFF)

include(CTest)
enable_testing()

add_subdirectory(src)
add_subdirectory(src/filter)
add_subdirectory(src/quaternion_estimator)
add_subdirectory(test)

if(EXAMPLE)
  add_subdirectory(example)
endif(EXAMPLE)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
