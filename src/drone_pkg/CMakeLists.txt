cmake_minimum_required(VERSION 3.8)
project(drone_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)

# ── C++ controller node ───────────────────────────────────────────────────────
add_executable(drone_controller src/drone_controller.cpp)

ament_target_dependencies(drone_controller
  rclcpp
  std_msgs
  geometry_msgs
  gazebo_msgs
)

install(TARGETS drone_controller
  DESTINATION lib/${PROJECT_NAME}
)

# ── Python teleop node ────────────────────────────────────────────────────────
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/drone_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
