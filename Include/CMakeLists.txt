cmake_minimum_required(VERSION 3.6)   # CMake version check
set(CMAKE_CXX_STANDARD 14)            # Enable c++11 standard

include_directories("boost/")

add_library(grapholon common.hpp GrapholonTypes.hpp SkeletalGraph.hpp VoxelSkeleton.hpp)       # Add executable target with source files listed in SOURCE_FILES variable
set_target_properties(grapholon PROPERTIES LINKER_LANGUAGE CXX)
