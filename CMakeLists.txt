##
## Name        :  CMakeLists.txt
## Author      :  Aaron
## Purpose     :  An input file to CMake used to build all of the executables
##                in this project.
##
##                For this to work and be portable across different people
##                You need to have the environment variable 'FOURXX' defined.
##                It should be set to the location where you checked out the top-
##                level folder in the git repository. For example, the following
##                was taken from my ~/.bash_profile:
##
##                export FOURXX="/Users/amnicholas/Documents/ELEC490/adaptive_grip_recent"

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "$ENV{FOURXX}/cmake/")

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(fourxx)

# Locate the PCL and Readling Packages - we need them
find_package(PCL 1.2 REQUIRED)
find_package(Readline REQUIRED)

# Set include directories
include_directories (headers
  /Users/amnicholas/Documents/ELEC490/Group3/original/headers)
include_directories(${INCLUDE_DIRECTORIES} ${PCL_INCLUDE_DIRS} ${Readline_INCLUDE_DIR})
include_directories(${INCLUDE_DIRECTORIES} "$ENV{FOURXX}/headers")

# Subdirectory containing source files
add_subdirectory(src)

# Link Directories
link_directories(${PCL_LIBRARY_DIRS})

# Definitions
add_definitions(${PCL_DEFINITIONS})

# Executables
add_executable(test_two_plane programs/test_two_plane_filter.cpp)

target_link_libraries(test_two_plane ${PCL_LIBRARIES} ${Readline_LIBRARY} Utils)

