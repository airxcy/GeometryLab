cmake_minimum_required(VERSION 3.1)
project(visual)

#engine core
set(CORE_DIR "${ENGINE_DIR}/visual")


set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)

#add_subdirectory("engine/cork")
#add_subdirectory("engine")
add_subdirectory("${ENGINE_DIR}/glfw" gflw)
# set 3party path directories


#manually add glad
set(GLAD_DIR "${ENGINE_DIR}/glad")
file(GLOB GLAD_SOURCES ${GLAD_DIR}/src/*.c)
file(GLOB_RECURSE GLAD_INCLUDE ${GLAD_DIR}/include/*.h)
add_library(glad ${GLAD_SOURCES} ${GLAD_INCLUDE})
target_include_directories(glad PUBLIC ${GLAD_DIR}/include)


#imgui projects with no CMakeList.txt
set(IMGUI_DIR "${ENGINE_DIR}/imgui")
file(GLOB IMGUI_SOURCES  ${IMGUI_DIR}/*.cpp ${IMGUI_DIR}/platform/*.cpp ${IMGUI_DIR}/*.h ${IMGUI_DIR}/platform/*.h)
add_library(imgui ${IMGUI_SOURCES})
target_compile_definitions(imgui PUBLIC -DIMGUI_IMPL_OPENGL_LOADER_GLAD)
target_include_directories(imgui PUBLIC ${IMGUI_DIR} ${IMGUI_DIR}/platform)
target_link_libraries(imgui PRIVATE glad glfw)


#add_library(imguizmo engine/imguizmo/ImGuizmo.cpp engine/imguizmo/ImGuizmo.h)
file(GLOB IMGUIZMO_SOURCES ${ENGINE_DIR}/ImGuizmo/*.cpp engine/ImGuizmo/*.h)
#add_library(imguizmo ${IMGUIZMO_SOURCES})
#target_link_libraries(imguizmo PUBLIC imgui)

file(GLOB IMGUINODE_SOURCES ${ENGINE_DIR}/imguinode/*.cpp engine/imguinode/*.h )


file(GLOB CORE_SRC  src/*.cpp)
file(GLOB CORE_H  include/*.h)
add_library(${PROJECT_NAME} ${CORE_SRC} ${CORE_H} ${IMGUIZMO_SOURCES} ${IMGUI_SOURCES} ${IMGUINODE_SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC "${ENGINE_DIR}" ${CORE_DIR}/include ${EIGEN_DIR} "${ENGINE_DIR}/ImGuizmo" ${IMGUI_DIR})
target_link_libraries(${PROJECT_NAME} glad glfw ${GLFW_LIBRARIES} )