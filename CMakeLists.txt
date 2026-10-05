cmake_minimum_required(VERSION 3.15)
project(mppi_ga)

find_package(Eigen3)
#set(CMAKE_CXX_STANDARD 20)

set(INCLUDE_DEPENDS ${EIGEN3_INCLUDE_DIR})

add_library(mppi_ga SHARED src/mppi.cpp
  include/mppi_ga/ThreadPool.h
  include/mppi_ga/ThreadPool2.h
  include/mppi_ga/states.h
)
target_include_directories(mppi_ga PUBLIC include ${INCLUDE_DEPENDS})
target_link_libraries(mppi_ga tbb)

# allow finding this library
set(INCLUDE_DEPENDS ${CMAKE_INSTALL_PREFIX}/include ${INCLUDE_DEPENDS})
set(LIB_DEPENDS ${CMAKE_INSTALL_PREFIX}/lib/libmppi_ga.so ${LIB_DEPENDS})

add_subdirectory(demo)
