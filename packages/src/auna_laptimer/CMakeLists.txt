cmake_minimum_required(VERSION 3.5)
project(auna_laptimer)

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
        auna_laptimer_interfaces
        std_msgs)

  find_package(${PACKAGE} REQUIRED)

endforeach()

# Include Cpp "include" directory
include_directories(include)

# Target dependencies
set(laptimer_dependencies rclcpp std_msgs sensor_msgs geometry_msgs auna_laptimer_interfaces)

# Add libraries
add_library(${PROJECT_NAME}_lib SHARED src/laptimer.cpp)
target_include_directories(${PROJECT_NAME}_lib PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}_lib ${laptimer_dependencies})

# Create Cpp executable
add_executable(laptimer src/laptimer_main.cpp)

# Link libraries
target_link_libraries(laptimer ${PROJECT_NAME}_lib)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# Install Cpp executables
install(TARGETS laptimer DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME}_lib
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
