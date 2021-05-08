cmake_minimum_required(VERSION 3.17)
project(Turtle)

set(CMAKE_CXX_STANDARD 14)

add_executable(Turtle main.cpp)

target_include_directories(Turtle PRIVATE 3rd_party)
