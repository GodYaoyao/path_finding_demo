cmake_minimum_required(VERSION 3.9)
project(PathFinding)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)

add_executable(PathFinding main.cpp include/Astar.h include/RRT.h include/Dijkstra.h include/BiRRT.h include/RRTStar.h)
target_link_libraries(PathFinding ${OpenCV_LIBS})