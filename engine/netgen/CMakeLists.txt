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
# add_definitions(-DNG_INSTALL_DIR=${CMAKE_INSTALL_PREFIX})
# add_definitions(-DNG_INSTALL_DIR_BIN=${CMAKE_INSTALL_PREFIX}/bin)
# add_definitions(-DNG_INSTALL_DIR_PYTHON=${CMAKE_INSTALL_PREFIX})
# add_definitions(-DNG_INSTALL_DIR_LIB=${CMAKE_INSTALL_PREFIX}/lib)
# add_definitions(-DNG_INSTALL_DIR_CMAKE=${CMAKE_INSTALL_PREFIX}/cmake)
# add_definitions(-DNG_INSTALL_DIR_INCLUDE=${CMAKE_INSTALL_PREFIX}/include)
#set(CMAKE_PREFIX_PATH ${ENGINE_DIR}/occ/cmake)
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

# set(NG_INSTALL_DIR EXPORT netgen-targets RUNTIME DESTINATION ${NG_INSTALL_DIR_BIN} COMPONENT netgen LIBRARY DESTINATION ${NG_INSTALL_DIR_LIB} COMPONENT netgen_devel ARCHIVE DESTINATION ${NG_INSTALL_DIR_LIB} COMPONENT netgen_devel)
# install(EXPORT netgen-targets DESTINATION ${NG_INSTALL_DIR_CMAKE} COMPONENT netgen_devel)

#######################################################################
add_library(nglib ${NGLIB_LIBRARY_TYPE})

#######################################################################

ExternalProject_Add(project_zlib
  ${SUBPROJECT_ARGS}
  SOURCE_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/external_dependencies/zlib
  CMAKE_ARGS
      -DCMAKE_INSTALL_PREFIX=${ZLIB_DIR}
      ${SUBPROJECT_CMAKE_ARGS}
  UPDATE_COMMAND "" # Disable update
  # BUILD_IN_SOURCE 1
  # INSTALL_COMMAND ${CMAKE_COMMAND} -E copy_directory . ${ZLIB_DIR}
  )

list(APPEND NETGEN_DEPENDENCIES project_zlib)
list(APPEND NETGEN_CMAKE_PREFIX_PATH ${ZLIB_DIR})

if(WIN32)
  # force linking the static library
  set(ZLIB_INCLUDE_DIRS ${ZLIB_DIR}/include)
  set(ZLIB_LIBRARIES  zlibstatic)
endif(WIN32)

link_directories(${ZLIB_DIR}/lib)
target_link_directories(nglib PRIVATE ${ZLIB_DIR}/lib)
target_include_directories(nglib PRIVATE ${ZLIB_INCLUDE_DIRS})
target_link_libraries(nglib PRIVATE ${ZLIB_LIBRARIES})
message("+++++++" ${ZLIB_DIR}/include)


#######################################################################
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


#######################################################################

# include instead of add_subdirectory to recognize the generated source files properly
include(rules/CMakeLists.txt)

add_subdirectory(windows)
add_subdirectory(libsrc)
add_subdirectory(ng)
# add_subdirectory(tutorials)
# add_subdirectory(py_tutorials)
# add_subdirectory(doc)
add_subdirectory(nglib)
# add_subdirectory(tests)



# get_target_property(ALLSRC nglib SOURCES)

# message(${ALLSRC})