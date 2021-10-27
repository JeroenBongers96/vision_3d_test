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
find_package(PCL 1.8 REQUIRED)
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(suii_communication REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# REMOVE AFTER TESTING!
find_package(example_interfaces REQUIRED)

# Find python before pybind11
find_package(python_cmake_module REQUIRED)
find_package(PythonExtra REQUIRED)

# Include Cpp "include" directory
include_directories(include)

FILE(GLOB_RECURSE SRC_FILES src/*.cpp)   

# Create Cpp executable
add_executable(vision_server ${SRC_FILES})
target_link_libraries(vision_server ${catkin_LIBRARIES} ${realsense2_LIBRARY} ${OpenCV_LIBS} ${PCL_LIBRARIES})
ament_target_dependencies(vision_server rclcpp suii_communication tf2 tf2_ros tf2_msgs geometry_msgs cv_bridge sensor_msgs)

add_executable(yolo_client_test scritps/yolo_client_test.cpp)
target_link_libraries(yolo_client_test ${OpenCV_LIBS})
ament_target_dependencies(yolo_client_test rclcpp std_msgs sensor_msgs suii_communication cv_bridge)

# Install Cpp executables
install(TARGETS
  vision_server
  yolo_client_test
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  vision_3d_test/yolo_server_test.py
  vision_3d_test/yolov5_server.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()