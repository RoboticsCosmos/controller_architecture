cmake_minimum_required(VERSION 3.5)

project(controllerArchitecture)

add_compile_definitions(_OS_UNIX)

find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(kinova_mediator REQUIRED HINTS ${CMAKE_CURRENT_SOURCE_DIR}/kinova_mediator/build)

include_directories(
    ${orocos_kdl_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/kinova_mediator/include
    # ${CMAKE_CURRENT_SOURCE_DIR}/kinova_mediator/kinova_api/lib/release
    ${CMAKE_CURRENT_SOURCE_DIR}/kinova_mediator/kinova_api/include/client
    ${CMAKE_CURRENT_SOURCE_DIR}/kinova_mediator/kinova_api/include/client_stubs
    ${CMAKE_CURRENT_SOURCE_DIR}/kinova_mediator/kinova_api/include/common
    ${CMAKE_CURRENT_SOURCE_DIR}/kinova_mediator/kinova_api/include/google
    ${CMAKE_CURRENT_SOURCE_DIR}/kinova_mediator/kinova_api/include
    ${CMAKE_CURRENT_SOURCE_DIR}/kinova_mediator/kinova_api/include/messages
)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/matplotplusplus)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/gen)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/utils/src)