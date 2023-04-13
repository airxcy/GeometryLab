#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ngcore" for configuration "Release"
set_property(TARGET ngcore APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(ngcore PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/ngcore.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/ngcore.dll"
  )

list(APPEND _cmake_import_check_targets ngcore )
list(APPEND _cmake_import_check_files_for_ngcore "${_IMPORT_PREFIX}/lib/ngcore.lib" "${_IMPORT_PREFIX}/bin/ngcore.dll" )

# Import target "nglib" for configuration "Release"
set_property(TARGET nglib APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(nglib PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/nglib.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "TKBO;TKBRep;TKBool;TKCAF;TKCDF;TKFillet;TKG2d;TKG3d;TKGeomAlgo;TKGeomBase;TKHLR;TKIGES;TKLCAF;TKMath;TKMesh;TKOffset;TKPrim;TKSTEP;TKSTEP209;TKSTEPAttr;TKSTEPBase;TKSTL;TKService;TKShHealing;TKTopAlgo;TKV3d;TKVCAF;TKXCAF;TKXDEIGES;TKXDESTEP;TKXSBase;TKernel"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/nglib.dll"
  )

list(APPEND _cmake_import_check_targets nglib )
list(APPEND _cmake_import_check_files_for_nglib "${_IMPORT_PREFIX}/lib/nglib.lib" "${_IMPORT_PREFIX}/bin/nglib.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
