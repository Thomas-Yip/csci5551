cmake_minimum_required(VERSION 3.12.2)
project(drone_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options( -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

ament_auto_find_build_dependencies()

include_directories(include)

# ament_auto_add_executable(
#   auv_test
#   src/auv_test.cpp
# )

ament_auto_add_executable(
  drone_control_demo
  src/drone.cpp
)

# Load & run linters listed in package.xml
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install scripts
install(
  PROGRAMS  # PROGRAMS sets execute bits, FILES clears them
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package()