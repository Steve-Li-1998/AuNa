cmake_minimum_required(VERSION 3.5)
project(auna_gapfollowing)

# Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20 REQUIRED)
# endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
foreach(
        PACKAGE
        ament_cmake
        ament_cmake_python
        rclcpp
        rclpy
        geometry_msgs
        nav_msgs
        ackermann_msgs
        sensor_msgs
        std_msgs)

  find_package(${PACKAGE} REQUIRED)

endforeach()

# Include Cpp "include" directory
include_directories(include)

# Add libraries
add_library(sample_node_lib SHARED src/gapfollowing.cpp)

# Create Cpp executable
add_executable(sample_node src/gapfollowing_main.cpp)

# Target dependencies
set(sample_node_dependencies rclcpp std_msgs sensor_msgs geometry_msgs)
ament_target_dependencies(sample_node_lib ${sample_node_dependencies})

# Link libraries
target_link_libraries(sample_node sample_node_lib)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# Install Cpp executables
install(TARGETS sample_node DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS sample_node_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
