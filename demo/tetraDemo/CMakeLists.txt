cmake_minimum_required(VERSION 3.1)
project(tetraDemo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
message("CMAKE_MODULE_PATH" ${CMAKE_MODULE_PATH})
set(CMAKE_BUILD_TYPE Release)
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

# execute_process(COMMAND "cd ../engine/OCCT/build")
# execute_process(COMMAND "cmake .. --build --install")
# execute_process(COMMAND "cmake  --build --install")
# execute_process(COMMAND "cd D:/projects/GeometryLab/build")
# Open CASCADE Technology




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
add_subdirectory(${ENGINE_DIR}/tetgen LIBTETGEN)
message("------------" ${ZLIB_DIR}/lib)

file(GLOB SRC ${CMAKE_SOURCE_DIR}/src/*.cpp)
file(GLOB INCLUDES ${CMAKE_SOURCE_DIR}/src/*.h)
file(GLOB DEMO_SRC *.cpp)
file(GLOB DEMO_INCLUDES *.h)
link_directories(${ZLIB_DIR}/lib)
file(GLOB IMGUIZMO_SOURCES ${ENGINE_DIR}/ImGuizmo/*.cpp engine/ImGuizmo/*.h)
add_executable(${PROJECT_NAME}  ${DEMO_SRC} ${DEMO_INCLUDES} ${SRC} ${INCLUDES} ${IMGUIZMO_SOURCES} )
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/src ${ENGINE_DIR} ${EIGEN_DIR} 
  ${ENGINE_DIR}/polyscope ${ENGINE_DIR}/geometrycentral/include 
  ${ENGINE_DIR}/netgen/libsrc ${ENGINE_DIR}/netgen/libsrc/include ${ENGINE_DIR}/netgen/nglib)
target_link_libraries(${PROJECT_NAME} polyscope geometry-central ${OpenCASCADE_LIBS} ${OPENGL_LIBRARIES} ngcore nglib tet)
add_dependencies(${PROJECT_NAME} copy_dll)
