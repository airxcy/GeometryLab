cmake_minimum_required(VERSION 3.1)
project(visual_debugger)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(CMAKE_BUILD_TYPE Release)

# Add dir path of a independent projects with a CMakeList.txt





set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)


add_subdirectory("${ENGINE_DIR}/glfw" glfw)
add_subdirectory("${ENGINE_DIR}/glad" glad)
add_subdirectory("${ENGINE_DIR}/imgui" imgui)
set(GC_EIGEN_LOCATION ${EIGEN_DIR})
add_subdirectory(${ENGINE_DIR}/geometrycentral GEOMETRYCENTRAL)
add_subdirectory(${ENGINE_DIR}/stb STB)
add_subdirectory(${ENGINE_DIR}/polyscope POLYSCOPE)
# set 3party path directories




#add_library(imguizmo engine/imguizmo/ImGuizmo.cpp engine/imguizmo/ImGuizmo.h)
file(GLOB IMGUIZMO_SOURCES ${ENGINE_DIR}/ImGuizmo/*.cpp engine/ImGuizmo/*.h)
#add_library(imguizmo ${IMGUIZMO_SOURCES})
#target_link_libraries(imguizmo PUBLIC imgui)

file(GLOB FILE_SRC *.cpp)
file(GLOB FILE_H *.h)


add_executable(visual_debugger  visual_debugger.cpp EigenMeshD.cpp visual_debugger.h ${IMGUIZMO_SOURCES})
target_include_directories(visual_debugger PUBLIC ${ENGINE_DIR} ${EIGEN_DIR} "${ENGINE_DIR}/ImGuizmo" ${ENGINE_DIR}/polyscope ${ENGINE_DIR}/geometrycentral/include)
target_link_libraries(visual_debugger polyscope geometry-central)

