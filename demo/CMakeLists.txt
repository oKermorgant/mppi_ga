
find_package(log2plot REQUIRED)
find_package(VISP REQUIRED COMPONENTS core)
add_definitions(-DMPPI_GA_RESULT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/results/")
#set(CMAKE_CXX_STANDARD 20)

set(includes ${EIGEN3_INCLUDE_DIR} ${LOG2PLOT_INCLUDE_DIRS})

add_executable(bicycle bicycle/main.cpp)
target_include_directories(bicycle PUBLIC ${includes})
target_link_libraries(bicycle mppi_ga ${LOG2PLOT_LIBRARIES} ${VISP_LIBRARIES})

add_executable(rov rov/rov.cpp rov/main.cpp)
target_include_directories(rov PUBLIC ${includes})
target_link_libraries(rov mppi_ga ${LOG2PLOT_LIBRARIES} ${VISP_LIBRARIES})

add_executable(rov_test rov/rov.cpp rov/test.cpp)
target_include_directories(rov_test PUBLIC ${includes})
target_link_libraries(rov_test mppi_ga ${LOG2PLOT_LIBRARIES} ${VISP_LIBRARIES})

add_executable(pend pendulum/pendulum.cpp pendulum/main.cpp)
target_include_directories(pend PUBLIC ${includes})
target_link_libraries(pend mppi_ga ${LOG2PLOT_LIBRARIES} ${VISP_LIBRARIES})

add_executable(boat boat/boat.cpp boat/main.cpp)
target_include_directories(boat PUBLIC ${includes})
target_link_libraries(boat mppi_ga ${LOG2PLOT_LIBRARIES} ${VISP_LIBRARIES})
