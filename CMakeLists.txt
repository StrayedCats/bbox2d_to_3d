cmake_minimum_required(VERSION 3.8)
project(bbox2d_to_3d_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(OpenCV REQUIRED)

# bbox2d_to_3d_sync --
set(TARGET bbox2d_to_3d_sync)
ament_auto_add_library(${TARGET} SHARED
  src/${TARGET}_node.cpp
)
ament_target_dependencies(${TARGET} OpenCV)

rclcpp_components_register_node(
  ${TARGET}
  PLUGIN "${PROJECT_NAME}::BBox2DTo3DSyncNode"
  EXECUTABLE ${TARGET}_node_exec)

# bbox2d_to_3d_quick --
set(TARGET bbox2d_to_3d_quick)
ament_auto_add_library(${TARGET} SHARED
  src/${TARGET}_node.cpp
)
ament_target_dependencies(${TARGET} OpenCV)

rclcpp_components_register_node(
  ${TARGET}
  PLUGIN "${PROJECT_NAME}::BBox2DTo3DQuickNode"
  EXECUTABLE ${TARGET}_node_exec)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)
ament_auto_package()