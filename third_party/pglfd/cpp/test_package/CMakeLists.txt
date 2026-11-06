cmake_minimum_required(VERSION 3.15)
project(pglfdPackageTest CXX)

set(CMAKE_CXX_STANDARD 20)

find_package(pglfd CONFIG REQUIRED)

add_executable(example src/example.cpp)
target_link_libraries(example pglfd::pglfd)
