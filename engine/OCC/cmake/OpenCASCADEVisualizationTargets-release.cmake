#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "TKService" for configuration "Release"
set_property(TARGET TKService APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(TKService PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/win64/vc14/lib/TKService.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/win64/vc14/bin/TKService.dll"
  )

list(APPEND _cmake_import_check_targets TKService )
list(APPEND _cmake_import_check_files_for_TKService "${_IMPORT_PREFIX}/win64/vc14/lib/TKService.lib" "${_IMPORT_PREFIX}/win64/vc14/bin/TKService.dll" )

# Import target "TKV3d" for configuration "Release"
set_property(TARGET TKV3d APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(TKV3d PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/win64/vc14/lib/TKV3d.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/win64/vc14/bin/TKV3d.dll"
  )

list(APPEND _cmake_import_check_targets TKV3d )
list(APPEND _cmake_import_check_files_for_TKV3d "${_IMPORT_PREFIX}/win64/vc14/lib/TKV3d.lib" "${_IMPORT_PREFIX}/win64/vc14/bin/TKV3d.dll" )

# Import target "TKOpenGl" for configuration "Release"
set_property(TARGET TKOpenGl APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(TKOpenGl PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/win64/vc14/lib/TKOpenGl.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/win64/vc14/bin/TKOpenGl.dll"
  )

list(APPEND _cmake_import_check_targets TKOpenGl )
list(APPEND _cmake_import_check_files_for_TKOpenGl "${_IMPORT_PREFIX}/win64/vc14/lib/TKOpenGl.lib" "${_IMPORT_PREFIX}/win64/vc14/bin/TKOpenGl.dll" )

# Import target "TKMeshVS" for configuration "Release"
set_property(TARGET TKMeshVS APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(TKMeshVS PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/win64/vc14/lib/TKMeshVS.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/win64/vc14/bin/TKMeshVS.dll"
  )

list(APPEND _cmake_import_check_targets TKMeshVS )
list(APPEND _cmake_import_check_files_for_TKMeshVS "${_IMPORT_PREFIX}/win64/vc14/lib/TKMeshVS.lib" "${_IMPORT_PREFIX}/win64/vc14/bin/TKMeshVS.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
