#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rs_a3_hardware::rs_a3_hardware" for configuration ""
set_property(TARGET rs_a3_hardware::rs_a3_hardware APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(rs_a3_hardware::rs_a3_hardware PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/librs_a3_hardware.so"
  IMPORTED_SONAME_NOCONFIG "librs_a3_hardware.so"
  )

list(APPEND _cmake_import_check_targets rs_a3_hardware::rs_a3_hardware )
list(APPEND _cmake_import_check_files_for_rs_a3_hardware::rs_a3_hardware "${_IMPORT_PREFIX}/lib/librs_a3_hardware.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
