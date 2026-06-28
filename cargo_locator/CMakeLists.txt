cmake_minimum_required(VERSION 3.8)
project(cargo_locator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Find required packages ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(aruco_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

# --- Create the executable ---
# 添加 Kalman_filter 源文件到 executable
add_executable(${PROJECT_NAME}_node 
  src/cargo_locator_node.cpp
  src/Kalman_filter.cpp
)

target_link_libraries(${PROJECT_NAME}_node Eigen3::Eigen)

# 添加 include 目录
target_include_directories(${PROJECT_NAME}_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  aruco_interfaces
)

# --- Install the executable ---
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# --- Install launch and config files ---
install(FILES
  launch/locator.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# --- Install header files ---
install(
  DIRECTORY include/
  DESTINATION include
)

# --- Setup testing (if applicable) ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Exclude copyright and cpplint checks if desired
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()