## Name        :  src/CMakeLists.txt
## Author      :  Aaron
## Purpose     :  CMake input file.

# Tell CMake to compile these files into a library called "Utils"
#find_package(Qt5Widgets REQUIRED)

find_package(PCL 1.2 REQUIRED)
#find_package(Readline REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${Qt5Widgets_INCLUDE_DIRS})
link_directories("/usr/local/lib")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")

# Removed Shell.cpp
add_library(Utils 
   RobotExt.cpp stringmanip.cpp Logger.cpp  Engine.cpp PlaneFilter.cpp TwoPlaneFilter.cpp Camera.cpp 
   CameraFile.cpp PCLUtils.cpp ClusterFinder.cpp WSObject.cpp plane_clipper_3D.cpp pcl_visualizer_instantiations.cpp
   Calibration.cpp Engine2.cpp QLogger.cpp Clawduino.cpp)

target_link_libraries(Utils)
