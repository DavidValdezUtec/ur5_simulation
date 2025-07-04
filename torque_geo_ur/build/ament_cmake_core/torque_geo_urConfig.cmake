# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_torque_geo_ur_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED torque_geo_ur_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(torque_geo_ur_FOUND FALSE)
  elseif(NOT torque_geo_ur_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(torque_geo_ur_FOUND FALSE)
  endif()
  return()
endif()
set(_torque_geo_ur_CONFIG_INCLUDED TRUE)

# output package information
if(NOT torque_geo_ur_FIND_QUIETLY)
  message(STATUS "Found torque_geo_ur: 0.0.0 (${torque_geo_ur_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'torque_geo_ur' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${torque_geo_ur_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(torque_geo_ur_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${torque_geo_ur_DIR}/${_extra}")
endforeach()
