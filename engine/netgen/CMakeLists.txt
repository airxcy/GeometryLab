project(NetGen)

cmake_minimum_required(VERSION 3.13)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_DEBUG_POSTFIX d)
set(NETGEN_VERSION_GIT v6.2.2302-21-g9180f9b9)
message("CMAKE_SOURCE_DIR:" ${CMAKE_SOURCE_DIR} )
set(ENGINE_DIR D:/projects/GeometryLab/engine)
set(INSTALL_DIR ${ENGINE_DIR}/netgen)
set(CMAKE_INSTALL_PREFIX ${ENGINE_DIR}/netgen)
# add_definitions(-DCMAKE_GENERATOR_PLATFORM=x64)
 set(CMAKE_ARGS " -A x64 ") 
include(ExternalProject)
include (CMakeDependentOption)
option( USE_OCC     "build with OpenCascade geometry kernel interface" ON)
set(NG_COMPILE_FLAGS "" CACHE STRING "Additional compile flags")
set(NGLIB_LIBRARY_TYPE STATIC CACHE STRING "nglib library type")
set(NGCORE_LIBRARY_TYPE STATIC CACHE STRING "ngcore library type")

#set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_modules")



#######################################################################
# build options
include_directories ("${PROJECT_SOURCE_DIR}/include")
include_directories ("${PROJECT_SOURCE_DIR}/libsrc")
include_directories ("${PROJECT_SOURCE_DIR}/libsrc/include")
include_directories ("${PROJECT_BINARY_DIR}")

set(CMAKE_INCLUDE_CURRENT_DIR ON)



set(NG_INSTALL_DIR D:/projects/GeometryLab/engine/netgen)
add_library(nglib ${NGLIB_LIBRARY_TYPE})

if (USE_OCC)
    find_package(OpenCascade NAMES OpenCASCADE opencascade REQUIRED CMAKE_FIND_ROOT_PATH_BOTH)
    add_definitions(-DOCCGEOMETRY)
    set(OCC_LIBRARIES
      TKBO
      TKBRep
      TKBool
      TKCAF
      TKCDF
      TKFillet
      TKG2d
      TKG3d
      TKGeomAlgo
      TKGeomBase
      TKHLR
      TKIGES
      TKLCAF
      TKMath
      TKMesh
      TKOffset
      TKPrim
      TKSTEP
      TKSTEP209
      TKSTEPAttr
      TKSTEPBase
      TKSTL
      TKService
      TKShHealing
      TKTopAlgo
      TKV3d
      TKVCAF
      TKXCAF
      TKXDEIGES
      TKXDESTEP
      TKXSBase
      TKernel
    )
    include_directories(${OpenCASCADE_INCLUDE_DIR})
endif (USE_OCC)

# include instead of add_subdirectory to recognize the generated source files properly
include(rules/CMakeLists.txt)
add_subdirectory(libsrc)
add_subdirectory(nglib)