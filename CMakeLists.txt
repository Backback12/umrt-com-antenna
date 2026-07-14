cmake_minimum_required(VERSION 3.8)
project(umrt_com_antenna)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# libgpiod does not ship a CMake config, so locate it manually.
# Requires: sudo apt install libgpiod-dev
find_path(GPIOD_INCLUDE_DIR gpiod.hpp)
find_library(GPIOD_LIB gpiod)
find_library(GPIODCXX_LIB gpiodcxx)

if(NOT GPIOD_INCLUDE_DIR OR NOT GPIOD_LIB OR NOT GPIODCXX_LIB)
  message(FATAL_ERROR "libgpiod (with C++ bindings) not found. Install with: sudo apt install libgpiod-dev")
endif()

include_directories(include ${GPIOD_INCLUDE_DIR})

# --- Switch publisher node (SW1/SW2 GPIO) ---
add_executable(switch_publisher_node
  src/switch_publisher_node.cpp
  src/switch_publisher_main.cpp
)
ament_target_dependencies(switch_publisher_node rclcpp std_msgs)
target_link_libraries(switch_publisher_node ${GPIODCXX_LIB} ${GPIOD_LIB})

# --- hmc5883l Magnetometer node (I2C) ---
add_executable(qmc5883p_node
  src/qmc5883p_node.cpp
  src/qmc5883p_main.cpp
)
ament_target_dependencies(qmc5883p_node rclcpp std_msgs sensor_msgs)

install(TARGETS
  switch_publisher_node
  qmc5883p_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()