cmake_minimum_required(VERSION 3.1)
project(research)

set(ENGINE_DIR "D:/research/engine")
message(status ${ENGINE_DIR})
set(EIGEN_DIR "${ENGINE_DIR}/eigen")
add_subdirectory("src/visual_debugger")

#add_subdirectory("src/vector_heat")

#add_subdirectory("src/flip_geodesic")