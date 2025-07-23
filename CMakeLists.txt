cmake_minimum_required(VERSION 3.10)

project(AGPD VERSION 1.0)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include ${PCL_INCLUDE_DIRS})

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(dvmm_demo src/dvmm.h src/dvmm.cpp src/demo.cpp)

target_link_libraries(dvmm_demo ${PCL_LIBRARIES} ${OpenCV_LIBS} yaml-cpp)
