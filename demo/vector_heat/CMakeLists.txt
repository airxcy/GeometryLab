cmake_minimum_required(VERSION 3.8.0)

project(vector_heat)

set(TargetName ${PROJECT_NAME})


set(GC_EIGEN_LOCATION "${ENGINE_DIR}/eigen")
add_subdirectory("${ENGINE_DIR}/geometrycentral" geometrycentral)
add_subdirectory("${ENGINE_DIR}/polyscope" polyscope)
add_subdirectory("${ENGINE_DIR}/args" args)
add_subdirectory("${ENGINE_DIR}/glm" glm)
add_subdirectory("${ENGINE_DIR}/stb" stb)


file(GLOB FILE_SRC *.cpp)
file(GLOB FILE_H  *.h)

add_library(${TargetName} ${FILE_SRC} ${FILE_H})
message(STATUS "engine_dir: ${ENGINE_DIR}")
target_include_directories(${TargetName} PUBLIC ${ENGINE_DIR}/geometrycentral)
target_include_directories(${TargetName} PUBLIC ${ENGINE_DIR}/polyscope/include)
target_include_directories(${TargetName} PUBLIC ${ENGINE_DIR}/glm)
target_include_directories(${TargetName} PUBLIC ${ENGINE_DIR}/args)
target_link_libraries(${TargetName}  PUBLIC visual_debugger visual geometry-central polyscope)