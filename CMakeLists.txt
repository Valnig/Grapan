cmake_minimum_required(VERSION 2.6)

project(Grapholon)

add_library(
    Grapholon
        "src/Grapholon.cpp" 
        "inc/Grapholon.hpp"
)

target_include_directories(
    Grapholon 
        PUBLIC "inc"
)

