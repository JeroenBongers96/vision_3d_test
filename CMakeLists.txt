cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(visionTest)

find_package(PCL 1.8 REQUIRED)
find_package(realsense2 REQUIRED)
find_package(OpenCV 3.2.0 REQUIRED)
find_package(ZED 3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

FILE(GLOB_RECURSE SRC_FILES src/*.cpp)
FILE(GLOB_RECURSE HDR_FILES include/*.hpp)

add_definitions(${PCL_DEFINITIONS})
add_definitions(-std=c++14 -O3)

add_executable (${PROJECT_NAME} ${HDR_FILES} ${SRC_FILES})

target_link_libraries (visionTest ${OpenCV_LIBS} ${PCL_LIBRARIES} ${realsense2_LIBRARY})