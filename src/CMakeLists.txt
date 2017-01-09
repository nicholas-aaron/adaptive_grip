## Name        :  src/CMakeLists.txt
## Author      :  Aaron
## Purpose     :  CMake input file.

# Tell CMake to compile these files into a library called "Utils"
add_library(Utils Shell.cpp stringmanip.cpp Logger.cpp Engine.cpp)

