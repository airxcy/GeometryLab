cmake_minimum_required(VERSION 3.1)
project(view2d)

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
add_subdirectory(${ENGINE_DIR}/stb STB)
add_subdirectory(${ENGINE_DIR}/polyscope POLYSCOPE)

#file(GLOB SRC ${CMAKE_SOURCE_DIR}/src/*.cpp)
#file(GLOB INCLUDES ${CMAKE_SOURCE_DIR}/src/*.h)
file(GLOB DEMO_SRC *.cpp)
file(GLOB DEMO_INCLUDES *.h)
add_executable(${PROJECT_NAME}  
${DEMO_SRC} ${DEMO_INCLUDES} 
#   ${SRC} ${INCLUDES} 
#${IMGUIZMO_SOURCES} 
)
target_include_directories(${PROJECT_NAME} PUBLIC ${ENGINE_DIR}  ${EIGEN_DIR} 
${ENGINE_DIR}/polyscope/include)
target_link_libraries(${PROJECT_NAME} polyscope ${OPENGL_LIBRARIES})
