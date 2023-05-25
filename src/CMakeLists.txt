cmake_minimum_required(VERSION 3.1)
project(geometrylab)
set(GC_EIGEN_LOCATION ${EIGEN_DIR})
add_subdirectory(${ENGINE_DIR}/geometrycentral GEOMETRYCENTRAL)

file(GLOB TRIANGLE_SOURCES ${ENGINE_DIR}/triangle/triangle.c ${ENGINE_DIR}/triangle/triangle.h)
file(GLOB SRC *.cpp)
file(GLOB INCLUDES *.h)


add_library(${PROJECT_NAME}  ${SRC} ${INCLUDES} ${TRIANGLE_SOURCES})
target_compile_definitions(${PROJECT_NAME} PRIVATE -DTRILIBRARY -DANSI_DECLARATORS  -DNO_TIMER)
target_include_directories(${PROJECT_NAME} PUBLIC ${ENGINE_DIR} ${EIGEN_DIR} 
  ${ENGINE_DIR}/geometrycentral/include ${ENGINE_DIR}/triangle)