cmake_minimum_required(VERSION 3.16)
project(rpi_diff_hw_control LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# Find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Create executable
add_executable(twist_to_twist_stamped hardware/twist_to_twist_stamped.cpp)

# Link dependencies
ament_target_dependencies(twist_to_twist_stamped
  rclcpp
  geometry_msgs
)

# Install
install(TARGETS
  twist_to_twist_stamped
  DESTINATION lib/${PROJECT_NAME}
)

# Create library
add_library(
  rpi_diff_hw_control
  SHARED
  hardware/rpi_ros2_control.cpp
)
target_compile_features(rpi_diff_hw_control PUBLIC cxx_std_17)
target_include_directories(rpi_diff_hw_control PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(
  rpi_diff_hw_control PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Link pigpiod_if2 and pigpio libraries
target_link_libraries(rpi_diff_hw_control PUBLIC pigpiod_if2)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface rpi_diff_hw_control_interface.xml)

# Install header files
install(
  DIRECTORY hardware/include/
  DESTINATION include/${PROJECT_NAME}
)

# Install launch, configuration, and description files
install(
  DIRECTORY description/ros2_control description/urdf description/rviz
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/${PROJECT_NAME}
)

# Install library
install(TARGETS rpi_diff_hw_control
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export targets and dependencies
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()

