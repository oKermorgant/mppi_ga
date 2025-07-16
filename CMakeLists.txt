cmake_minimum_required(VERSION 3.5)
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
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/mppi_gaConfig.cmake.in
               "${PROJECT_BINARY_DIR}/mppi_gaConfig.cmake")

add_subdirectory(demo)

install (FILES
    ${CMAKE_CURRENT_BINARY_DIR}/mppi_gaConfig.cmake
    DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/mppi_ga")

install(TARGETS mppi_ga EXPORT mppi_gaConfig
    ARCHIVE  DESTINATION lib
    LIBRARY  DESTINATION lib
    RUNTIME  DESTINATION bin)
install(DIRECTORY include/mppi_ga DESTINATION include)
