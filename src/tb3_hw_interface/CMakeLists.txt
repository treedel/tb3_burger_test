cmake_minimum_required(VERSION 3.8)
project(tb3_hw_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  tb3_hw_interface
  SHARED
  hardware/tb3_system.cpp
)
target_compile_features(tb3_hw_interface PUBLIC cxx_std_17)
target_include_directories(tb3_hw_interface PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/tb3_hw_interface>
)
ament_target_dependencies(
  tb3_hw_interface PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(tb3_hw_interface PUBLIC serial)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface tb3_hw_interface.xml)

# INSTALL
install(
  DIRECTORY hardware/include
  DESTINATION include/tb3_hw_interface
)
install(
  DIRECTORY urdf
  DESTINATION share/tb3_hw_interface
)
install(
  DIRECTORY launch config
  DESTINATION share/tb3_hw_interface
)
install(TARGETS tb3_hw_interface
  EXPORT export_tb3_hw_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_tb3_hw_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
