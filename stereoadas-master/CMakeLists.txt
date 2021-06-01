cmake_minimum_required(VERSION 3.1)
# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
SET(OpenCV_DIR /home/burak/installation/OpenCV-3.4.4/share/OpenCV/)
project( DisplayImage )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable( DisplayImage main.cpp StereoMatching.cpp StereoVisionForADAS.cpp StixelEstimation.cpp StixelSegmentation.cpp )
target_link_libraries( DisplayImage ${OpenCV_LIBS} )

