if(WIN32)

  ExternalProject_Add(project_win_zlib
    SOURCE_DIR "D:/projects/GeometryLab/engine/netgen/zlib"
    UPDATE_COMMAND "" # Disable update
    BUILD_IN_SOURCE 1
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
#    INSTALL_COMMAND ${CMAKE_COMMAND} -E copy_directory . ${CMAKE_INSTALL_PREFIX}
    LOG_DOWNLOAD 1
    )


  list(APPEND NETGEN_DEPENDENCIES project_win_zlib)
endif(WIN32)

