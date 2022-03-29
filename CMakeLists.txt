cmake_minimum_required(VERSION 3.20)
project(findLines)

set(CMAKE_CXX_STANDARD 14)

set(OpenCV_DIR /home/chan/chan/3rdparty/opencv454/opencv-4.5.5/build)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(findLines main.cpp template_match.cpp)

target_link_libraries(findLines ${OpenCV_LIBS})