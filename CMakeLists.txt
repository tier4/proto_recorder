cmake_minimum_required(VERSION 3.8)
project(proto_recorder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(include)

# Build the proto_recorder library
add_library(proto_recorder SHARED
  src/proto_recorder.cpp
)

target_compile_definitions(proto_recorder
  PRIVATE "PROTO_RECORDER_BUILDING_DLL")

ament_target_dependencies(proto_recorder
  rclcpp
  rclcpp_components
  rosbag2_cpp
  rosbag2_storage
  rosbag2_transport
  diagnostic_updater
  yaml-cpp
)

# Register the component
rclcpp_components_register_node(proto_recorder
  PLUGIN "proto_recorder::ProtoRecorder"
  EXECUTABLE proto_recorder_node
)

# Install include directories
install(
  DIRECTORY include/
  DESTINATION include
)

# Install libraries
install(
  TARGETS proto_recorder
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install executables
install(
  TARGETS proto_recorder_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(proto_recorder)
ament_export_targets(export_${PROJECT_NAME})
ament_package()
