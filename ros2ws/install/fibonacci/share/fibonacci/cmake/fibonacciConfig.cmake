# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_fibonacci_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED fibonacci_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(fibonacci_FOUND FALSE)
  elseif(NOT fibonacci_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(fibonacci_FOUND FALSE)
  endif()
  return()
endif()
set(_fibonacci_CONFIG_INCLUDED TRUE)

# output package information
if(NOT fibonacci_FIND_QUIETLY)
  message(STATUS "Found fibonacci: 0.0.0 (${fibonacci_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'fibonacci' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${fibonacci_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(fibonacci_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${fibonacci_DIR}/${_extra}")
endforeach()
