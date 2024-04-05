cmake_minimum_required(VERSION 3.5)

project(pidController)

find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

include_directories(
    ${orocos_kdl_INCLUDE_DIRS}
)

add_subdirectory(matplotplusplus)

# specifying name of the executable as the same as project name
# add_executable(${PROJECT_NAME} src/adaptive_control.cpp)
add_executable(${PROJECT_NAME} src/simple_adaptive_pid.cpp)

# PUBLIC keyword is used to specify thet other projects that depend on the current one should also be linked to the executable
target_link_libraries(
    ${PROJECT_NAME} PUBLIC matplot
    orocos-kdl
    )

# set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "output_name")