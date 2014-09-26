cmake_minimum_required(VERSION 2.8)
project( main )
find_package( OpenCV REQUIRED )
ADD_DEFINITIONS(
  -g
  -O0
  -Wall
  -std=c++11
)
add_executable( main src/main.cpp src/LaplacianPotential.cpp)
target_link_libraries( main ${OpenCV_LIBS} )
