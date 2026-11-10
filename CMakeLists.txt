cmake_minimum_required(VERSION 3.8)
project(sbus_serial)

# C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# Find and dependencies for all node executables
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
set(dependencies "rclcpp" "std_msgs" "geometry_msgs" "std_srvs")

# Generate the interfaces
find_package(rosidl_default_generators REQUIRED)
set(msg_files
  "msg/Sbus.msg"
  "msg/SbusCommands.msg"
  DEPENDENCIES std_msgs
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
ament_export_dependencies(rosidl_default_runtime)


# sbus_serial_node: include the generated interfaces
add_executable(sbus_serial_node src/sbus_serial_node.cpp src/sbus_serial_driver.cpp)
ament_target_dependencies(sbus_serial_node ${dependencies})
target_link_libraries(sbus_serial_node "${cpp_typesupport_target}")

# sbus_cmd_vel_node: include the generated interfaces
add_executable(sbus_commands src/sbus_commands.cpp)
ament_target_dependencies(sbus_commands ${dependencies})
target_link_libraries(sbus_commands "${cpp_typesupport_target}")

# Separate executable for the calibration
add_executable(sbus_calibrate src/sbus_calibrate.cpp src/sbus_serial_driver.cpp)

# Install executables in lib
install(
	TARGETS sbus_serial_node sbus_commands
	DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}/
)

# Install calibration binary in share
install(
	TARGETS sbus_calibrate
	DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_winch_hold_to_run test/test_winch_hold_to_run.cpp)
  target_include_directories(test_winch_hold_to_run PRIVATE include)

  ament_add_gtest(test_sbus_commands_mock test/test_sbus_commands_mock.cpp)
  target_include_directories(test_sbus_commands_mock PRIVATE include)
  ament_target_dependencies(test_sbus_commands_mock ${dependencies})
  target_link_libraries(test_sbus_commands_mock "${cpp_typesupport_target}")
endif()

ament_package()
