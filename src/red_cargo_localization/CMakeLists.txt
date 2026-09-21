cmake_minimum_required(VERSION 3.8)
project(red_cargo_localization)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(red_cargo_detector src/red_cargo_detector.cpp)
target_include_directories(red_cargo_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(red_cargo_localizer_node src/red_cargo_localizer_node.cpp)
target_link_libraries(red_cargo_localizer_node red_cargo_detector)
ament_target_dependencies(red_cargo_localizer_node
  geometry_msgs rclcpp sensor_msgs std_msgs)

install(TARGETS red_cargo_detector red_cargo_localizer_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_red_cargo_detector test/test_red_cargo_detector.cpp)
  target_link_libraries(test_red_cargo_detector red_cargo_detector)
endif()

ament_export_include_directories(include)
ament_export_libraries(red_cargo_detector)
ament_package()
