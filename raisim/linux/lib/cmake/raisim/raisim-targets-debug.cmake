#----------------------------------------------------------------
# Generated CMake target import file for configuration "DEBUG".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "raisim::raisimZ" for configuration "DEBUG"
set_property(TARGET raisim::raisimZ APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(raisim::raisimZ PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libraisimZd.so"
  IMPORTED_SONAME_DEBUG "libraisimZd.so"
  )

list(APPEND _cmake_import_check_targets raisim::raisimZ )
list(APPEND _cmake_import_check_files_for_raisim::raisimZ "${_IMPORT_PREFIX}/lib/libraisimZd.so" )

# Import target "raisim::raisimPng" for configuration "DEBUG"
set_property(TARGET raisim::raisimPng APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(raisim::raisimPng PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libraisimPngd.so"
  IMPORTED_SONAME_DEBUG "libraisimPngd.so"
  )

list(APPEND _cmake_import_check_targets raisim::raisimPng )
list(APPEND _cmake_import_check_files_for_raisim::raisimPng "${_IMPORT_PREFIX}/lib/libraisimPngd.so" )

# Import target "raisim::raisimMine" for configuration "DEBUG"
set_property(TARGET raisim::raisimMine APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(raisim::raisimMine PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libraisimMined.so"
  IMPORTED_SONAME_DEBUG "libraisimMined.so"
  )

list(APPEND _cmake_import_check_targets raisim::raisimMine )
list(APPEND _cmake_import_check_files_for_raisim::raisimMine "${_IMPORT_PREFIX}/lib/libraisimMined.so" )

# Import target "raisim::raisimODE" for configuration "DEBUG"
set_property(TARGET raisim::raisimODE APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(raisim::raisimODE PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libraisimODEd.so.1.1.7"
  IMPORTED_SONAME_DEBUG "libraisimODEd.so.1.1.7"
  )

list(APPEND _cmake_import_check_targets raisim::raisimODE )
list(APPEND _cmake_import_check_files_for_raisim::raisimODE "${_IMPORT_PREFIX}/lib/libraisimODEd.so.1.1.7" )

# Import target "raisim::raisim" for configuration "DEBUG"
set_property(TARGET raisim::raisim APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(raisim::raisim PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libraisimd.so.1.1.7"
  IMPORTED_SONAME_DEBUG "libraisimd.so.1.1.7"
  )

list(APPEND _cmake_import_check_targets raisim::raisim )
list(APPEND _cmake_import_check_files_for_raisim::raisim "${_IMPORT_PREFIX}/lib/libraisimd.so.1.1.7" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
