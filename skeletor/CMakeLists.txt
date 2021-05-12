  
cmake_minimum_required(VERSION 3.16)
project(tree_json)
find_package(nlohmann_json)

add_executable(conformingterrain main.cpp)
target_link_libraries(conformingterrain nlohmann_json::nlohmann_json)
set_target_properties(conformingterrain PROPERTIES CXX_STANDARD 17)

# Make the executable a default target to build & run in Visual Studio
set_property(DIRECTORY ${PROJECT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ../conformingterrain)