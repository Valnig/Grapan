cmake_minimum_required(VERSION 3.6)   # CMake version check
project(Grapholon_cli)               # Create project "simple_example"
set(CMAKE_CXX_STANDARD 14)            # Enable c++11 standard

subdirs("Include/")

include_directories("Include/")

set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED OFF)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost) 

if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS}) 
	add_executable(grapholon_cli grapholon_cli.cpp)       # Add executable target with source files listed in SOURCE_FILES variable
	target_link_libraries(grapholon_cli grapholon ${Boost_LIBRARIES})
endif()
