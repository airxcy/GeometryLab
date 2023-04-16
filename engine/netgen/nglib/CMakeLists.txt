target_sources(nglib PRIVATE nglib.cpp)

if(USE_OCC)
    target_sources(nglib PRIVATE nglib_occ.cpp)
    install(FILES nglib_occ.h DESTINATION ${NG_INSTALL_DIR}/include)
endif(USE_OCC)

target_link_libraries(nglib PUBLIC ngcore)

target_link_libraries( nglib PRIVATE ${MPI_CXX_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${ZLIB_LIBRARIES} ${OCC_LIBRARIES}  )


message("-----------------" ${NG_INSTALL_DIR})
install(TARGETS nglib DESTINATION ${NG_INSTALL_DIR}/lib)
install(FILES nglib.h DESTINATION ${NG_INSTALL_DIR}/include)
