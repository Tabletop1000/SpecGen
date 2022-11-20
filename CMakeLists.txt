cmake_minimum_required(VERSION 3.0.0)
project(ReportGen VERSION 0.1.0)

include(CTest)
enable_testing()

add_subdirectory(${CMAKE_SOURCE_DIR}/include/matplotplusplus)

add_executable(ReportGen 
    ${CMAKE_SOURCE_DIR}/main.cpp
    ${CMAKE_SOURCE_DIR}/src/actuators.cpp
    )

target_include_directories(ReportGen PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/include
    )

target_link_libraries(ReportGen PUBLIC matplot)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

