add_library(control_blocks_binary SHARED control_blocks.c) # this will get included into the CMakeLists where this is added as a sub_directory
add_library(utilities SHARED utilities.c) # this will get included into the CMakeLists where this is added as a sub_directory
add_library(monitors_binary SHARED monitors.c) # this will get included into the CMakeLists where this is added as a sub_directory
add_library(log_data SHARED log_data.cpp) # this will get included into the CMakeLists where this is added as a sub_directory

# add where the headers are located; 99% of the time this should be PUBLIC
# it is a good pactice to add /.. after specifying the target include directories so that the headers are included from the parent directory for better organization
target_include_directories(control_blocks_binary PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../include) # using PUBLIC here means that any target that links to this target will also have access to the headers
target_include_directories(monitors_binary PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../include) # using PUBLIC here means that any target that links to this target will also have access to the headers
target_include_directories(log_data PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../include) # using PUBLIC here means that any target that links to this target will also have access to the headers
target_include_directories(utilities PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../include) # using PUBLIC here means that any target that links to this target will also have access to the headers

# as monitors_binary is dependent on utilities, we need to link the utilities library to the monitors_binary library
target_link_libraries(monitors_binary PUBLIC utilities) # this will link the utilities library to the control_blocks_binary library

# specify which version of compiler to use
# target_compile_features(monitors_binary PRIVATE cxx_std_17) # this will use C++17; by using PRIVATE here, (autocompleted: it means that only this target will use C++17), (in lecture: external people cannot know what version of C++ is being used, so it is private)
target_compile_features(control_blocks_binary PRIVATE cxx_std_17) # this will use C++17; by using PRIVATE here, (autocompleted: it means that only this target will use C++17), (in lecture: external people cannot know what version of C++ is being used, so it is private)

