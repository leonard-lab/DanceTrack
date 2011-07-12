##################################################
#
# CMakeLists.txt - Root cmake file for DanceTracker
#
# History:
#   * Original file - Dan Swain, 7/6/10
#   * Simplied significantly - DTS 7/14/10
#   * Modified from SimpleBWTracker - DTS 8/26/10
#   * Modified from KilliTracker - DTS 6/20/11
#   
##################################################

# this is the most recent version of CMake
cmake_minimum_required (VERSION 2.8)

# name of the project - this has to come first
project(DanceTracker)

# MADTRaC information
set(MT_ROOT "../MADTraC/build" CACHE PATH "MADTraC root directory")
# sets variables related to MT
include(${MT_ROOT}/cmake/MT_Config.cmake)

# set the name of the built target
set(APP_NAME DanceTracker)

# set this to a list of source and header files to be compiled
set(APP_SRC
  src/DanceTrackerGUI.cpp
  src/DanceTrackerGUI.h
  src/DanceTracker.cpp
  src/DanceTracker.h
  src/DSGYBlobber.cpp
  src/DSGYBlobber.h
  test/DSGYA_Segmenter.cpp
  test/DSGYA_Segmenter.h
  test/BiCC.cpp
  test/BiCC.h
  )

# adds src to the header search path
include_directories(src)
include_directories(test)

# sets up the build target
add_executable(${APP_NAME} WIN32 MACOSX_BUNDLE ${APP_SRC})

# sets up the app to build correctly using the right libraries, etc
MT_tracking_app(${APP_NAME})
