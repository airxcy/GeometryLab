
add_library(ngcore ${NGCORE_LIBRARY_TYPE}
  archive.cpp
  bitarray.cpp
  exception.cpp
  localheap.cpp
  logging.cpp
  flags.cpp
  paje_trace.cpp
  profiler.cpp
  table.cpp
  taskmanager.cpp
  utils.cpp
  version.cpp
  )


target_compile_definitions(ngcore PRIVATE NGCORE_EXPORTS)
target_include_directories(ngcore INTERFACE $<INSTALL_INTERFACE:${NG_INSTALL_DIR_INCLUDE}> $<INSTALL_INTERFACE:${NG_INSTALL_DIR_INCLUDE}/include>)

if(CHECK_RANGE)
  target_compile_definitions(ngcore PUBLIC NETGEN_ENABLE_CHECK_RANGE)
endif(CHECK_RANGE)



target_link_libraries(ngcore PUBLIC PRIVATE  ${CMAKE_THREAD_LIBS_INIT})

install(FILES ngcore.hpp archive.hpp type_traits.hpp version.hpp ngcore_api.hpp logging.hpp memtracer.hpp
  exception.hpp symboltable.hpp paje_trace.hpp utils.hpp profiler.hpp mpi_wrapper.hpp
  array.hpp taskmanager.hpp concurrentqueue.h localheap.hpp python_ngcore.hpp flags.hpp
  xbool.hpp signal.hpp bitarray.hpp table.hpp hashtable.hpp ranges.hpp ngstream.hpp
  simd.hpp simd_avx.hpp simd_avx512.hpp simd_generic.hpp simd_sse.hpp simd_arm64.hpp
  register_archive.hpp
  DESTINATION ${NG_INSTALL_DIR_INCLUDE}/core)


# add_dependencies(ngcore ng_generate_version_file)

