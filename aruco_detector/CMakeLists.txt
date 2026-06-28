cmake_minimum_required(VERSION 3.8)
project(aruco_detector)

# 编译器选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找 ROS2 依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED) # 确保包含 tf2_ros
find_package(aruco_interfaces REQUIRED)

# --- 查找 OpenCV ---
find_package(OpenCV REQUIRED)

# 调试信息：打印找到的 OpenCV 信息
message(STATUS "Found OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV libraries (from OpenCV_LIBS): ${OpenCV_LIBS}")

# 确定用于链接的 OpenCV 库
if(OpenCV_LIBS)
    set(USED_OPENCV_LIBS ${OpenCV_LIBS})
else()
    message(WARNING "OpenCV_LIBS not found by find_package. Falling back to manual list.")
    set(USED_OPENCV_LIBS
        opencv_core
        opencv_imgproc
        opencv_calib3d
        opencv_aruco
    )
endif()
message(STATUS "Will link against OpenCV libraries: ${USED_OPENCV_LIBS}")

# 定义可执行文件
add_executable(aruco_detector_node src/aruco_detector_node.cpp)

# 设置依赖（包含路径、库等由 ament 处理）
ament_target_dependencies(aruco_detector_node
  rclcpp
  image_transport
  cv_bridge
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros # 链接 tf2_ros
  aruco_interfaces
)

# 添加 OpenCV 的包含目录
if(OpenCV_INCLUDE_DIRS)
    target_include_directories(aruco_detector_node PRIVATE ${OpenCV_INCLUDE_DIRS})
else()
    message(FATAL_ERROR "OpenCV_INCLUDE_DIRS not set. Please check your OpenCV installation.")
endif()

# 链接 OpenCV 库
target_link_libraries(aruco_detector_node ${USED_OPENCV_LIBS})

# 安装规则
install(TARGETS
  aruco_detector_node
  DESTINATION lib/${PROJECT_NAME})

# 测试（如果启用）
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install specific launch files.
install(FILES
  launch/detect_aruco.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)
# 包导出
ament_package()



