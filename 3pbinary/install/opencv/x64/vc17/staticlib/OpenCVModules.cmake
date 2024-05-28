# Generated by CMake

if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 2.8)
   message(FATAL_ERROR "CMake >= 2.8.0 required")
endif()
if(CMAKE_VERSION VERSION_LESS "2.8.3")
   message(FATAL_ERROR "CMake >= 2.8.3 required")
endif()
cmake_policy(PUSH)
cmake_policy(VERSION 2.8.3...3.24)
#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Protect against multiple inclusion, which would fail when already imported targets are added once more.
set(_cmake_targets_defined "")
set(_cmake_targets_not_defined "")
set(_cmake_expected_targets "")
foreach(_cmake_expected_target IN ITEMS zlib libjpeg-turbo libtiff libwebp libopenjp2 libpng IlmImf ippiw libprotobuf ittnotify ade opencv_core opencv_flann opencv_imgproc opencv_ml opencv_photo ocv.3rdparty.flatbuffers opencv_dnn opencv_features2d opencv_imgcodecs ocv.3rdparty.dshow ocv.3rdparty.msmf ocv.3rdparty.ffmpeg ocv.3rdparty.obsensor opencv_videoio opencv_calib3d ocv.3rdparty.win32ui opencv_highgui opencv_objdetect opencv_stitching opencv_video opencv_gapi)
  list(APPEND _cmake_expected_targets "${_cmake_expected_target}")
  if(TARGET "${_cmake_expected_target}")
    list(APPEND _cmake_targets_defined "${_cmake_expected_target}")
  else()
    list(APPEND _cmake_targets_not_defined "${_cmake_expected_target}")
  endif()
endforeach()
unset(_cmake_expected_target)
if(_cmake_targets_defined STREQUAL _cmake_expected_targets)
  unset(_cmake_targets_defined)
  unset(_cmake_targets_not_defined)
  unset(_cmake_expected_targets)
  unset(CMAKE_IMPORT_FILE_VERSION)
  cmake_policy(POP)
  return()
endif()
if(NOT _cmake_targets_defined STREQUAL "")
  string(REPLACE ";" ", " _cmake_targets_defined_text "${_cmake_targets_defined}")
  string(REPLACE ";" ", " _cmake_targets_not_defined_text "${_cmake_targets_not_defined}")
  message(FATAL_ERROR "Some (but not all) targets in this export set were already defined.\nTargets Defined: ${_cmake_targets_defined_text}\nTargets not yet defined: ${_cmake_targets_not_defined_text}\n")
endif()
unset(_cmake_targets_defined)
unset(_cmake_targets_not_defined)
unset(_cmake_expected_targets)


# Compute the installation prefix relative to this file.
get_filename_component(_IMPORT_PREFIX "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component(_IMPORT_PREFIX "${_IMPORT_PREFIX}" PATH)
get_filename_component(_IMPORT_PREFIX "${_IMPORT_PREFIX}" PATH)
get_filename_component(_IMPORT_PREFIX "${_IMPORT_PREFIX}" PATH)
if(_IMPORT_PREFIX STREQUAL "/")
  set(_IMPORT_PREFIX "")
endif()

# Create imported target zlib
add_library(zlib STATIC IMPORTED)

# Create imported target libjpeg-turbo
add_library(libjpeg-turbo STATIC IMPORTED)

# Create imported target libtiff
add_library(libtiff STATIC IMPORTED)

set_target_properties(libtiff PROPERTIES
  INTERFACE_LINK_LIBRARIES "zlib"
)

# Create imported target libwebp
add_library(libwebp STATIC IMPORTED)

# Create imported target libopenjp2
add_library(libopenjp2 STATIC IMPORTED)

set_target_properties(libopenjp2 PROPERTIES
  INTERFACE_COMPILE_DEFINITIONS "OPJ_STATIC"
)

# Create imported target libpng
add_library(libpng STATIC IMPORTED)

set_target_properties(libpng PROPERTIES
  INTERFACE_LINK_LIBRARIES "zlib"
)

# Create imported target IlmImf
add_library(IlmImf STATIC IMPORTED)

set_target_properties(IlmImf PROPERTIES
  INTERFACE_LINK_LIBRARIES "zlib"
)

# Create imported target ippiw
add_library(ippiw STATIC IMPORTED)

# Create imported target libprotobuf
add_library(libprotobuf STATIC IMPORTED)

# Create imported target ittnotify
add_library(ittnotify STATIC IMPORTED)

# Create imported target ade
add_library(ade STATIC IMPORTED)

# Create imported target opencv_core
add_library(opencv_core STATIC IMPORTED)

set_target_properties(opencv_core PROPERTIES
  INTERFACE_LINK_LIBRARIES "\$<LINK_ONLY:ippiw>;\$<LINK_ONLY:ippicv>;\$<LINK_ONLY:zlib>;\$<LINK_ONLY:ittnotify>"
)

# Create imported target opencv_flann
add_library(opencv_flann STATIC IMPORTED)

set_target_properties(opencv_flann PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_core;opencv_core;\$<LINK_ONLY:ippiw>;\$<LINK_ONLY:ippicv>"
)

# Create imported target opencv_imgproc
add_library(opencv_imgproc STATIC IMPORTED)

set_target_properties(opencv_imgproc PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_core;opencv_core;\$<LINK_ONLY:ippiw>;\$<LINK_ONLY:ippicv>"
)

# Create imported target opencv_ml
add_library(opencv_ml STATIC IMPORTED)

set_target_properties(opencv_ml PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_core;opencv_core;\$<LINK_ONLY:ippiw>;\$<LINK_ONLY:ippicv>"
)

# Create imported target opencv_photo
add_library(opencv_photo STATIC IMPORTED)

set_target_properties(opencv_photo PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_core;opencv_imgproc;opencv_core;opencv_imgproc;\$<LINK_ONLY:ippiw>;\$<LINK_ONLY:ippicv>"
)

# Create imported target ocv.3rdparty.flatbuffers
add_library(ocv.3rdparty.flatbuffers INTERFACE IMPORTED)

set_target_properties(ocv.3rdparty.flatbuffers PROPERTIES
  INTERFACE_COMPILE_DEFINITIONS "HAVE_FLATBUFFERS=1"
)

# Create imported target opencv_dnn
add_library(opencv_dnn STATIC IMPORTED)

set_target_properties(opencv_dnn PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_core;opencv_imgproc;opencv_core;opencv_imgproc;\$<LINK_ONLY:ippiw>;\$<LINK_ONLY:ippicv>;\$<LINK_ONLY:ocv.3rdparty.flatbuffers>;\$<LINK_ONLY:libprotobuf>"
)

# Create imported target opencv_features2d
add_library(opencv_features2d STATIC IMPORTED)

set_target_properties(opencv_features2d PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_core;opencv_flann;opencv_imgproc;\$<LINK_ONLY:ippiw>;\$<LINK_ONLY:ippicv>"
)

# Create imported target opencv_imgcodecs
add_library(opencv_imgcodecs STATIC IMPORTED)

set_target_properties(opencv_imgcodecs PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_core;opencv_imgproc;opencv_core;opencv_imgproc;\$<LINK_ONLY:ippiw>;\$<LINK_ONLY:ippicv>;\$<LINK_ONLY:libjpeg-turbo>;\$<LINK_ONLY:libwebp>;\$<LINK_ONLY:libpng>;\$<LINK_ONLY:libtiff>;\$<LINK_ONLY:libopenjp2>;\$<LINK_ONLY:IlmImf>;\$<LINK_ONLY:zlib>"
)

# Create imported target ocv.3rdparty.dshow
add_library(ocv.3rdparty.dshow INTERFACE IMPORTED)

set_target_properties(ocv.3rdparty.dshow PROPERTIES
  INTERFACE_COMPILE_DEFINITIONS "HAVE_DSHOW"
)

# Create imported target ocv.3rdparty.msmf
add_library(ocv.3rdparty.msmf INTERFACE IMPORTED)

set_target_properties(ocv.3rdparty.msmf PROPERTIES
  INTERFACE_COMPILE_DEFINITIONS "HAVE_MSMF;HAVE_MSMF_DXVA"
)

# Create imported target ocv.3rdparty.ffmpeg
add_library(ocv.3rdparty.ffmpeg INTERFACE IMPORTED)

set_target_properties(ocv.3rdparty.ffmpeg PROPERTIES
  INTERFACE_COMPILE_DEFINITIONS "HAVE_FFMPEG_WRAPPER"
)

# Create imported target ocv.3rdparty.obsensor
add_library(ocv.3rdparty.obsensor INTERFACE IMPORTED)

set_target_properties(ocv.3rdparty.obsensor PROPERTIES
  INTERFACE_COMPILE_DEFINITIONS "HAVE_OBSENSOR;HAVE_OBSENSOR_MSMF"
)

# Create imported target opencv_videoio
add_library(opencv_videoio STATIC IMPORTED)

set_target_properties(opencv_videoio PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_core;opencv_imgproc;opencv_imgcodecs;opencv_core;opencv_imgproc;opencv_imgcodecs;\$<LINK_ONLY:ippiw>;\$<LINK_ONLY:ippicv>;\$<LINK_ONLY:ocv.3rdparty.dshow>;\$<LINK_ONLY:ocv.3rdparty.msmf>;\$<LINK_ONLY:ocv.3rdparty.ffmpeg>;\$<LINK_ONLY:ocv.3rdparty.obsensor>"
)

# Create imported target opencv_calib3d
add_library(opencv_calib3d STATIC IMPORTED)

set_target_properties(opencv_calib3d PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;\$<LINK_ONLY:ippiw>;\$<LINK_ONLY:ippicv>"
)

# Create imported target ocv.3rdparty.win32ui
add_library(ocv.3rdparty.win32ui INTERFACE IMPORTED)

set_target_properties(ocv.3rdparty.win32ui PROPERTIES
  INTERFACE_COMPILE_DEFINITIONS "HAVE_WIN32UI"
  INTERFACE_LINK_LIBRARIES "user32;gdi32"
)

# Create imported target opencv_highgui
add_library(opencv_highgui STATIC IMPORTED)

set_target_properties(opencv_highgui PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_core;opencv_imgproc;opencv_imgcodecs;opencv_videoio;opencv_core;opencv_imgproc;opencv_imgcodecs;opencv_videoio;\$<LINK_ONLY:ippiw>;\$<LINK_ONLY:ippicv>;\$<LINK_ONLY:comctl32>;\$<LINK_ONLY:gdi32>;\$<LINK_ONLY:ole32>;\$<LINK_ONLY:setupapi>;\$<LINK_ONLY:ws2_32>;\$<LINK_ONLY:ocv.3rdparty.win32ui>"
)

# Create imported target opencv_objdetect
add_library(opencv_objdetect STATIC IMPORTED)

set_target_properties(opencv_objdetect PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_dnn;opencv_features2d;opencv_calib3d;opencv_core;opencv_flann;opencv_imgproc;opencv_dnn;opencv_features2d;opencv_calib3d;\$<LINK_ONLY:ippiw>;\$<LINK_ONLY:ippicv>"
)

# Create imported target opencv_stitching
add_library(opencv_stitching STATIC IMPORTED)

set_target_properties(opencv_stitching PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_calib3d;opencv_core;opencv_flann;opencv_imgproc;opencv_features2d;opencv_calib3d;\$<LINK_ONLY:ippiw>;\$<LINK_ONLY:ippicv>"
)

# Create imported target opencv_video
add_library(opencv_video STATIC IMPORTED)

set_target_properties(opencv_video PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_dnn;opencv_features2d;opencv_calib3d;opencv_core;opencv_flann;opencv_imgproc;opencv_dnn;opencv_features2d;opencv_calib3d;\$<LINK_ONLY:ippiw>;\$<LINK_ONLY:ippicv>"
)

# Create imported target opencv_gapi
add_library(opencv_gapi STATIC IMPORTED)

set_target_properties(opencv_gapi PROPERTIES
  INTERFACE_LINK_LIBRARIES "opencv_core;opencv_flann;opencv_imgproc;opencv_dnn;opencv_features2d;opencv_calib3d;opencv_video;opencv_core;opencv_flann;opencv_imgproc;opencv_dnn;opencv_features2d;opencv_calib3d;opencv_video;\$<LINK_ONLY:ippiw>;\$<LINK_ONLY:ippicv>;\$<LINK_ONLY:ade>;\$<LINK_ONLY:ittnotify>;\$<LINK_ONLY:wsock32>;\$<LINK_ONLY:ws2_32>"
)

if(CMAKE_VERSION VERSION_LESS 3.0.0)
  message(FATAL_ERROR "This file relies on consumers using CMake 3.0.0 or greater.")
endif()

# Load information for each installed configuration.
file(GLOB _cmake_config_files "${CMAKE_CURRENT_LIST_DIR}/OpenCVModules-*.cmake")
foreach(_cmake_config_file IN LISTS _cmake_config_files)
  include("${_cmake_config_file}")
endforeach()
unset(_cmake_config_file)
unset(_cmake_config_files)

# Cleanup temporary variables.
set(_IMPORT_PREFIX)

# Loop over all imported files and verify that they actually exist
foreach(_cmake_target IN LISTS _cmake_import_check_targets)
  foreach(_cmake_file IN LISTS "_cmake_import_check_files_for_${_cmake_target}")
    if(NOT EXISTS "${_cmake_file}")
      message(FATAL_ERROR "The imported target \"${_cmake_target}\" references the file
   \"${_cmake_file}\"
but this file does not exist.  Possible reasons include:
* The file was deleted, renamed, or moved to another location.
* An install or uninstall procedure did not complete successfully.
* The installation package was faulty and contained
   \"${CMAKE_CURRENT_LIST_FILE}\"
but not all the files it references.
")
    endif()
  endforeach()
  unset(_cmake_file)
  unset("_cmake_import_check_files_for_${_cmake_target}")
endforeach()
unset(_cmake_target)
unset(_cmake_import_check_targets)

# This file does not depend on other imported targets which have
# been exported from the same project but in a separate export set.

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
cmake_policy(POP)
