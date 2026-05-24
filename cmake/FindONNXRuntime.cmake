# Copyright 2026 px4_ros2_ctrl contributors

set(_ONNXRUNTIME_ROOT_HINTS
  "${CMAKE_CURRENT_LIST_DIR}/../third_party/onnxruntime"
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/onnxruntime"
  "$ENV{ONNXRUNTIME_ROOT}"
)

find_path(ONNXRuntime_INCLUDE_DIR
  NAMES onnxruntime_cxx_api.h
  PATHS ${_ONNXRUNTIME_ROOT_HINTS}
  PATH_SUFFIXES include
)

find_library(ONNXRuntime_LIBRARY
  NAMES onnxruntime
  PATHS ${_ONNXRUNTIME_ROOT_HINTS}
  PATH_SUFFIXES lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
  ONNXRuntime
  REQUIRED_VARS ONNXRuntime_INCLUDE_DIR ONNXRuntime_LIBRARY
)

if(ONNXRuntime_FOUND AND NOT TARGET ONNXRuntime::onnxruntime)
  add_library(ONNXRuntime::onnxruntime UNKNOWN IMPORTED)
  set_target_properties(ONNXRuntime::onnxruntime PROPERTIES
    IMPORTED_LOCATION "${ONNXRuntime_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${ONNXRuntime_INCLUDE_DIR}"
  )
endif()

mark_as_advanced(ONNXRuntime_INCLUDE_DIR ONNXRuntime_LIBRARY)
