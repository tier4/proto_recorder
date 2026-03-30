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
  proto_recorder_msgs
  std_msgs
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
  find_package(ament_cmake_gtest REQUIRED)

  set(TEST_FIXTURES_DIR "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
  set(TEST_DEPS rclcpp proto_recorder_msgs rosbag2_cpp rosbag2_storage rosbag2_transport std_msgs yaml-cpp)

  # Helper macro to reduce repetition
  macro(add_proto_recorder_test TEST_NAME)
    ament_add_gtest(${TEST_NAME} test/${TEST_NAME}.cpp)
    target_link_libraries(${TEST_NAME} proto_recorder)
    target_compile_definitions(${TEST_NAME} PRIVATE
      PROTO_RECORDER_TESTING
      TEST_FIXTURES_DIR="${TEST_FIXTURES_DIR}")
    ament_target_dependencies(${TEST_NAME} ${TEST_DEPS})
  endmacro()

  add_proto_recorder_test(test_load_topics_from_file)
  add_proto_recorder_test(test_generate_timestamped_uri)
  add_proto_recorder_test(test_topic_rate_logic)
  add_proto_recorder_test(test_state_transitions)
  add_proto_recorder_test(test_node_integration)
endif()

ament_export_include_directories(include)
ament_export_libraries(proto_recorder)
ament_export_targets(export_${PROJECT_NAME})
ament_package()
