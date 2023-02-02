cmake_minimum_required(VERSION 3.1)



#cork
set(CORK_DIR "src")
file(GLOB_RECURSE CORK_SRC ${CORK_DIR}/*.cpp)
file(GLOB_RECURSE CORK_H ${CORK_DIR}/*.h )
FILE(GLOB cork_include  ${CORK_DIR}/*)
message(STATUS ${cork_include}  )
add_library(cork  ${CORK_SRC} ${CORK_H})
target_compile_definitions(cork PRIVATE -D_USE_MATH_DEFINES -DNOMINMAX)
target_include_directories(cork PUBLIC  ${CORK_DIR} "${ENGINE_DIR}/mpir")
target_include_directories(cork  PUBLIC ${cork_include})
target_link_libraries(cork mpir)
