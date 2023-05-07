cmake_minimum_required(VERSION 3.1)
project(testNg)

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_PREFIX_PATH ${ENGINE_DIR}/occ/cmake)
find_package (OpenCASCADE REQUIRED)
add_subdirectory(${ENGINE_DIR}/netgen NETGENLIB)


#file(GLOB IMGUIZMO_SOURCES ${ENGINE_DIR}/ImGuizmo/*.cpp engine/ImGuizmo/*.h)


file(GLOB occdll "${ENGINE_DIR}/OCC/win64/vc14/lib/*.lib")
add_custom_target(copy_dll ALL
    COMMAND ${CMAKE_COMMAND} -E copy
        ${ENGINE_DIR}/freetype/bin/freetype.dll
        Release/freetype.dll
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        NETGENLIB/Release/
        Release/
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        NETGENLIB/libsrc/core/Release/
        Release/
    # COMMAND ${CMAKE_COMMAND} -E copy
    #     ${occdll}
    #     Release/
    COMMENT "Copying"
)


#set(CMAKE_EXE_LINKER_FLAGS "-static")
# configure_file(${ENGINE_DIR}/freetype/bin/freetype.dll ${OpenCASCADE_BINARY_DIR}/freetype.dll COPYONLY)
# configure_file(${ENGINE_DIR}/netgen/bin/ngcore.dll D:/projects/GeometryLab/build/src/occSTEP/Release/ngcore.dll COPYONLY)
# configure_file(${ENGINE_DIR}/netgen/bin/nglib.dll D:/projects/GeometryLab/build/src/occSTEP/Release/nglib.dll COPYONLY)
# configure_file(${ENGINE_DIR}/netgen/bin/zlib.dll D:/projects/GeometryLab/build/src/occSTEP/Release/zlib.dll COPYONLY)
# execute_process(COMMAND ${OpenCASCADE_BINARY_DIR} copy "${ENGINE_DIR}/freetype/bin/freetype.dll" ${OpenCASCADE_BINARY_DIR})
file(GLOB SRC *.cpp)
file(GLOB INCLUDES *.h)
link_directories(${ENGINE_DIR}/netgen/lib)
add_executable(${PROJECT_NAME}  ${SRC} ${INCLUDES} )
target_include_directories(${PROJECT_NAME} PUBLIC
  ${ENGINE_DIR}/netgen/libsrc ${ENGINE_DIR}/netgen/libsrc/include ${ENGINE_DIR}/netgen/nglib)
target_link_libraries(${PROJECT_NAME} nglib)
add_dependencies(${PROJECT_NAME} copy_dll)
