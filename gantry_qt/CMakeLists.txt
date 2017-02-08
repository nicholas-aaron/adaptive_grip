set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/Users/amnicholas/Documents/ELEC490/adaptive_grip_recent/cmake/")
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pcl_qt_cmake)

set(PCL_DIR "~/Documents/ELEC490/libraries/pcl/build")
#find_package(Qt5Widgets REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package(VTK REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# This doesn't work
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-inconsistent-missing-override -g")
#Touch
set(FOURXX_DIR "/Users/amnicholas/Documents/ELEC490/adaptive_grip_recent")
set(VTK_LIBRARIES vtkRenderingQt vtkViewsQt vtkGUISupportQt)

include_directories(${INCLUDE_DIRECTORIES} /Users/amnicholas/Documents/ELEC490/Group3/original/headers)
include_directories(${INCLUDE_DIRECTORIES} ${PCL_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRECTORIES} ${Readline_INCLUDE_DIR})
include_directories(${INCLUDE_DIRECTORIES} ${FOURXX_DIR}/headers)


# GripperBase Library
#include_directories(${INCLUDE_DIRECTORIES} "/Users/amnicholas/Documents/ELEC490/adaptive_grip_recent/libraries/GantryLib/")

add_subdirectory(${FOURXX_DIR}/src ${CMAKE_CURRENT_BINARY_DIR}fourxx)

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${LINK_DIRECTORIES} "/Users/amnicholas/Documents/ELEC490/adaptive_grip_recent/libraries/GripperBase")

add_definitions(${PCL_DEFINTIONS})

qt5_wrap_ui(UIS_HDRS
    pclviewer2.ui
    gantrywindow.ui)

aux_source_directory(. SRC_LIST)

set (SOURCES main.cpp mainwindow.cpp itemdisplaywindow.cpp gantrywindow.cpp engthread.cpp liveviewer.cpp)
add_executable(${PROJECT_NAME} ${SOURCES} ${UIS_HDRS} mainwindow.cpp)
target_link_libraries(pcl_qt_cmake ${PCL_LIBRARIES} ${Readline_LIBRARY} Utils GripperBase)

# qt5_use_modules(${PROJECT_NAME} Widgets)
target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Widgets)
