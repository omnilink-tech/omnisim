cmake_minimum_required(VERSION 3.16)
project(omnisim_ros2_control LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  # std::from_chars / std::to_chars for double (json_lite, http_client) are
  # C++17 library features; GCC has them from 11.
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(omnisim_ros2_control SHARED
  src/omnisim_system.cpp
  src/http_client.cpp
  src/json_lite.cpp
)
target_include_directories(omnisim_ros2_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(omnisim_ros2_control
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface omnisim_ros2_control_plugin.xml)

install(
  TARGETS omnisim_ros2_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config description launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Unit tests for the pieces that can be wrong SILENTLY: the JSON reader
  # (a mis-parse looks like a stationary robot), the differential-drive fold
  # (a wrong sign looks like a robot that turns instead of driving), and the
  # joint-command verb selection (both verbs accept the FIRST command, so a
  # wrong pick only surfaces as intermittent 409s under a stream).
  ament_add_gtest(test_omnisim_ros2_control
    test/test_json_lite.cpp
    test/test_diff_drive_fold.cpp
    test/test_servo_verb.cpp
    src/json_lite.cpp
    src/omnisim_system.cpp
    src/http_client.cpp
  )
  target_include_directories(test_omnisim_ros2_control PRIVATE include)
  ament_target_dependencies(test_omnisim_ros2_control
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(omnisim_ros2_control)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
