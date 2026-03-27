cmake_minimum_required(VERSION 3.8)
project(tangent_bug_pmr)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)      # LaserScan
find_package(geometry_msgs REQUIRED)    # Twist, Pose
find_package(nav_msgs REQUIRED)         # Odometry 

# Executable
include_directories(include)
add_executable(tangent_bug
  src/tangent_bug.cpp
)

ament_target_dependencies(tangent_bug
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
)

# Install
install(TARGETS tangent_bug
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch
  description
  config
  worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
