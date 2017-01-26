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

# set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH} $ENV{FOURXX}/cmake/")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/Users/amnicholas/Documents/ELEC490/adaptive_grip_recent/cmake/")
MESSAGE(STATUS "CMAKE_MODULE_PATH : " ${CMAKE_MODULE_PATH})


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

# Original "Robot" include directory..
include_directories(${INCLUDE_DIRECTORIES} "/Users/amnicholas/Documents/ELEC490/Group3/original/headers/")
# add_subdirectory("../Group3/original/src")


# Suppress some warnings that we aren't going to fix
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -Wno-deprecated-register -Wno-inconsistent-missing-override -std=c++11")


# Subdirectory containing source files
add_subdirectory(src)



# Link Directories
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${LINK_DIRECTORIES} "/Users/amnicholas/Documents/ELEC490/adaptive_grip_recent/libraries/GripperBase")
# Definitions
add_definitions(${PCL_DEFINITIONS})

# Executables
#add_executable(test_two_plane programs/test_two_plane_filter.cpp)
#add_executable(gantry_test programs/gantry_test.cpp)
add_executable(test_vector_two programs/test_vector_two.cpp)
add_executable(test_vector_three programs/test_vector_three.cpp)
add_executable(test_engine programs/test_engine.cpp)

#add_executable(basic_shell programs/basic_shell.cpp)
#target_link_libraries(basic_shell GripperBase ${Readline_LIBRARY} Utils)

#target_link_libraries(gantry_test ${PCL_LIBRARIES} ${Readline_LIBRARY} Utils GripperBase)
target_link_libraries(test_vector_two ${PCL_LIBRARIES} ${Readline_LIBRARY} Utils GripperBase)
target_link_libraries(test_vector_three ${PCL_LIBRARIES} ${Readline_LIBRARY} Utils GripperBase)
target_link_libraries(test_engine ${PCL_LIBRARIES} ${Readline_LIBRARY} Utils GripperBase)
#target_link_libraries(test_two_plane ${PCL_LIBRARIES} ${Readline_LIBRARY} Utils)

