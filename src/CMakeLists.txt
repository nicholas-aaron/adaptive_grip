## Name        :  src/CMakeLists.txt
## Author      :  Aaron
## Purpose     :  CMake input file.

# Tell CMake to compile these files into a library called "Utils"
find_package(Qt5Widgets REQUIRED)
find_package(PCL 1.2 REQUIRED)
include_directories({${PCL_INCLUDE_DIRS})
include_directories(${Qt5Widgets_INCLUDE_DIRS})
add_library(Utils 
   RobotExt.cpp Shell.cpp stringmanip.cpp Logger.cpp QLogger.cpp Engine.cpp PlaneFilter.cpp TwoPlaneFilter.cpp Camera.cpp 
   CameraFile.cpp PCLUtils.cpp ClusterFinder.cpp WSObject.cpp plane_clipper_3D.cpp pcl_visualizer_instantiations.cpp
   Calibration.cpp Engine2.cpp)

qt5_use_modules(Utils Widgets)




