cmake_minimum_required(VERSION 3.1)
project(geometrylab)
set(CMAKE_CXX_STANDARD 17)

set(GC_EIGEN_LOCATION ${EIGEN_DIR})
add_subdirectory(${ENGINE_DIR}/geometrycentral GEOMETRYCENTRAL)
add_subdirectory(${ENGINE_DIR}/tetgen LIBTETGEN)


set(CMAKE_PREFIX_PATH ${ENGINE_DIR}/occ/cmake)
find_package (OpenCASCADE REQUIRED)
if (NOT OpenCASCADE_FOUND)
  message (FATAL_ERROR "coult not find OpenCASCADE, please set OpenCASCADE_DIR variable" )
else()
  message (STATUS "Using OpenCASCADE from \"${OpenCASCADE_INSTALL_PREFIX}\"" )
  message (STATUS "OpenCASCADE_INCLUDE_DIR=${OpenCASCADE_INCLUDE_DIR}")
  message (STATUS "OpenCASCADE_LIBRARY_DIR=${OpenCASCADE_LIBRARY_DIR}")
  INCLUDE_DIRECTORIES(${OpenCASCADE_INCLUDE_DIR})
endif()
SET(OpenCASCADE_LIBS 
  TKernel
  TKMath
  TKMesh
  TKBRep
  TKXSBase
  TKService
  TKV3d
  TKOpenGl
  TKGeomBase
  TKGeomAlgo
  TKG3d
  TKG2d
  TKTopAlgo
  TKPrim
  TKIGES
  TKSTEP
  TKVRML
  TKLCAF
)

set(ZLIB_DIR ${CMAKE_CURRENT_BINARY_DIR}/NETGENLIB/dependencies/zlib)
add_subdirectory(${ENGINE_DIR}/netgen NETGENLIB)
file(GLOB occdll "${ENGINE_DIR}/OCC/win64/vc14/bin/*.dll")
add_custom_target(copy_dll ALL
    COMMAND ${CMAKE_COMMAND} -E copy
        ${ENGINE_DIR}/freetype/bin/freetype.dll
        $<CONFIG>/freetype.dll
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${ZLIB_DIR}/lib/
        $<CONFIG>/
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        NETGENLIB/$<CONFIG>/
        $<CONFIG>/
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        NETGENLIB/libsrc/core/$<CONFIG>/
        $<CONFIG>/
    COMMAND ${CMAKE_COMMAND} -E copy
        ${occdll}
        $<CONFIG>/
    COMMENT "Copying"
)
message("------------" ${ZLIB_DIR}/lib)




file(GLOB TRIANGLE_SOURCES ${ENGINE_DIR}/triangle/triangle.c ${ENGINE_DIR}/triangle/triangle.h)
file(GLOB SRC *.cpp)
file(GLOB INCLUDES *.h)

link_directories(${ZLIB_DIR}/lib)
add_library(${PROJECT_NAME}  ${SRC} ${INCLUDES} ${TRIANGLE_SOURCES})
target_compile_definitions(${PROJECT_NAME} PRIVATE -DTRILIBRARY -DANSI_DECLARATORS  -DNO_TIMER)
target_include_directories(${PROJECT_NAME} PUBLIC ${ENGINE_DIR} ${EIGEN_DIR} 
  ${ENGINE_DIR}/geometrycentral/include ${ENGINE_DIR}/triangle
  ${OpenCASCADE_INCLUDE_DIR}
  ${ENGINE_DIR}/netgen/libsrc ${ENGINE_DIR}/netgen/libsrc/include ${ENGINE_DIR}/netgen/nglib)
target_link_libraries(${PROJECT_NAME} geometry-central tet ${OpenCASCADE_LIBS} ngcore nglib)
add_dependencies(${PROJECT_NAME} copy_dll)
