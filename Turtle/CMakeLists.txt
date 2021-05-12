cmake_minimum_required(VERSION 3.17)
project(Turtle)

set(CMAKE_CXX_STANDARD 14)

add_executable(Turtle main.cpp)

target_include_directories(Turtle PRIVATE 3rd_party)

find_package(Boost REQUIRED)

include_directories(${Boost_INCLUDE_DIR})