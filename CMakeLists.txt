cmake_minimum_required(VERSION 3.1)
project(GeometryLab)

set(ENGINE_DIR ${CMAKE_SOURCE_DIR}/engine)
set(EIGEN_DIR "${ENGINE_DIR}/eigen")

add_subdirectory(src)
#add_subdirectory("demo/view2d")
add_subdirectory("demo/tetraDemo")
#add_subdirectory("demo/torus")
#add_subdirectory("src/visual_debugger")
#add_subdirectory("src/discreteDomain")