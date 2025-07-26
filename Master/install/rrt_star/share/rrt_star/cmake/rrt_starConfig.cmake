# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_rrt_star_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED rrt_star_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(rrt_star_FOUND FALSE)
  elseif(NOT rrt_star_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(rrt_star_FOUND FALSE)
  endif()
  return()
endif()
set(_rrt_star_CONFIG_INCLUDED TRUE)

# output package information
if(NOT rrt_star_FIND_QUIETLY)
  message(STATUS "Found rrt_star: 0.0.0 (${rrt_star_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'rrt_star' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${rrt_star_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(rrt_star_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${rrt_star_DIR}/${_extra}")
endforeach()
