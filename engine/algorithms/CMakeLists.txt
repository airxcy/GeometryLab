#algoritms
set(ALGORITHMS_DIR "engine/algorithms")
file(GLOB ALGORITHMS_SRC *.cpp)
file(GLOB ALGORITHMS_H  *.h)
add_library(algorithms  ${ALGORITHMS_SRC} ${ALGORITHMS_H} )
target_include_directories(algorithms PUBLIC ${ENGINE_DIR} ${EIGEN_DIR} "${ENGINE_DIR}/triangle")
target_link_libraries(algorithms  triangle )
