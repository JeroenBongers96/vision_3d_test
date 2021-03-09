cmake_minimum_required(VERSION 3.5)
project(vision_3d_test)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
#find_package(roscpp REQUIRED)
#find_package(rospy REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)

# Include Cpp "include" directory
include_directories(include)

FILE(GLOB_RECURSE SRC_FILES src/*.cpp)   

# Create Cpp executable
add_executable(vision_run ${SRC_FILES})
target_link_libraries(vision_run ${catkin_LIBRARIES} ${realsense2_LIBRARY} ${OpenCV_LIBS} ${PCL_LIBRARIES})
ament_target_dependencies(vision_run rclcpp)

# Install Cpp executables
install(TARGETS
  vision_run
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
#ament_python_install_package(${PROJECT_NAME})

ament_package()