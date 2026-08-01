cmake_minimum_required(VERSION 3.8)
project(roar_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Compile the hardware interface library
add_library(roar_hardware SHARED src/roar_hardware_interface.cpp)
target_include_directories(roar_hardware PRIVATE include)
ament_target_dependencies(roar_hardware rclcpp hardware_interface pluginlib sensor_msgs std_msgs)

# Export the plugin description
pluginlib_export_plugin_description_file(hardware_interface roar_hardware.xml)

# Install the targets
install(
  TARGETS roar_hardware
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Install the plugin XML file so the controller manager can find it
install(
  FILES roar_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
