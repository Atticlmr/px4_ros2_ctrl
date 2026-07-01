# Copyright 2026 px4_ros2_ctrl contributors

set(ONNXRUNTIME_VERSION "1.26.0" CACHE STRING "ONNX Runtime prebuilt package version")
set(ONNXRUNTIME_PLATFORM "linux-x64" CACHE STRING "ONNX Runtime prebuilt package platform")
set(ONNXRUNTIME_URL
  "https://github.com/microsoft/onnxruntime/releases/download/v${ONNXRUNTIME_VERSION}/onnxruntime-${ONNXRUNTIME_PLATFORM}-${ONNXRUNTIME_VERSION}.tgz"
  CACHE STRING "ONNX Runtime prebuilt package URL"
)

set(_ONNXRUNTIME_VENDOR_ROOT "${CMAKE_CURRENT_LIST_DIR}/../third_party/onnxruntime")
set(_ONNXRUNTIME_ROOT_HINTS
  "${_ONNXRUNTIME_VENDOR_ROOT}"
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

if(NOT ONNXRuntime_INCLUDE_DIR OR NOT ONNXRuntime_LIBRARY)
  if(NOT UNIX OR APPLE)
    message(FATAL_ERROR
      "ONNX Runtime was not found in third_party/onnxruntime and automatic download is only "
      "configured for Linux. Set ONNXRUNTIME_ROOT to a local ONNX Runtime installation."
    )
  endif()

  set(_ONNXRUNTIME_DOWNLOAD_DIR "${CMAKE_CURRENT_LIST_DIR}/../third_party/_downloads")
  set(_ONNXRUNTIME_ARCHIVE
    "${_ONNXRUNTIME_DOWNLOAD_DIR}/onnxruntime-${ONNXRUNTIME_PLATFORM}-${ONNXRUNTIME_VERSION}.tgz")
  set(_ONNXRUNTIME_EXTRACTED
    "${_ONNXRUNTIME_DOWNLOAD_DIR}/onnxruntime-${ONNXRUNTIME_PLATFORM}-${ONNXRUNTIME_VERSION}")

  file(MAKE_DIRECTORY "${_ONNXRUNTIME_DOWNLOAD_DIR}")
  file(REMOVE_RECURSE "${_ONNXRUNTIME_EXTRACTED}")
  message(STATUS "ONNX Runtime not found in third_party/onnxruntime; downloading ${ONNXRUNTIME_URL}")
  file(DOWNLOAD
    "${ONNXRUNTIME_URL}"
    "${_ONNXRUNTIME_ARCHIVE}"
    SHOW_PROGRESS
    STATUS _ONNXRUNTIME_DOWNLOAD_STATUS
    TLS_VERIFY ON
  )
  list(GET _ONNXRUNTIME_DOWNLOAD_STATUS 0 _ONNXRUNTIME_DOWNLOAD_CODE)
  list(GET _ONNXRUNTIME_DOWNLOAD_STATUS 1 _ONNXRUNTIME_DOWNLOAD_MESSAGE)
  if(NOT _ONNXRUNTIME_DOWNLOAD_CODE EQUAL 0)
    message(FATAL_ERROR
      "Failed to download ONNX Runtime: ${_ONNXRUNTIME_DOWNLOAD_MESSAGE}. "
      "Install it under third_party/onnxruntime or set ONNXRUNTIME_ROOT."
    )
  endif()

  file(ARCHIVE_EXTRACT INPUT "${_ONNXRUNTIME_ARCHIVE}" DESTINATION "${_ONNXRUNTIME_DOWNLOAD_DIR}")
  if(NOT EXISTS "${_ONNXRUNTIME_EXTRACTED}/include/onnxruntime_cxx_api.h")
    message(FATAL_ERROR "Downloaded ONNX Runtime archive does not contain the expected include directory")
  endif()

  file(REMOVE_RECURSE "${_ONNXRUNTIME_VENDOR_ROOT}")
  file(RENAME "${_ONNXRUNTIME_EXTRACTED}" "${_ONNXRUNTIME_VENDOR_ROOT}")

  unset(ONNXRuntime_INCLUDE_DIR CACHE)
  unset(ONNXRuntime_LIBRARY CACHE)

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
endif()

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
