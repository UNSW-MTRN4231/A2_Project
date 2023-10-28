# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_pizza_pos_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED pizza_pos_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(pizza_pos_FOUND FALSE)
  elseif(NOT pizza_pos_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(pizza_pos_FOUND FALSE)
  endif()
  return()
endif()
set(_pizza_pos_CONFIG_INCLUDED TRUE)

# output package information
if(NOT pizza_pos_FIND_QUIETLY)
  message(STATUS "Found pizza_pos: 0.0.0 (${pizza_pos_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'pizza_pos' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${pizza_pos_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(pizza_pos_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${pizza_pos_DIR}/${_extra}")
endforeach()
