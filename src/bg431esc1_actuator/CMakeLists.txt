cmake_minimum_required(VERSION 3.8)
project(bg431esc1_actuator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable
    -Werror=return-type -Werror=shadow -Werror=format -Werror=range-loop-construct
    -Werror=missing-braces)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  tf2
  tf2_msgs
)

# Tell CMake to get the deps
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_compile_definitions(RCPPUTILS_VERSION_MAJOR=${rcpputils_VERSION_MAJOR})
add_compile_definitions(RCPPUTILS_VERSION_MINOR=${rcpputils_VERSION_MINOR})
generate_parameter_library(bg431esc1_actuator_parameters
  src/bg431esc1_actuator_parameters.yaml
)

add_library(bg431esc1_actuator SHARED
  src/bg431esc1_actuator.cpp
)
target_compile_features(bg431esc1_actuator PUBLIC cxx_std_23)
target_include_directories(bg431esc1_actuator PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/bg431esc1_actuator>
)

target_link_libraries(bg431esc1_actuator PUBLIC bg431esc1_actuator_parameters)
target_link_libraries(bg431esc1_actuator PUBLIC Eigen3::Eigen)
ament_target_dependencies(bg431esc1_actuator PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
pluginlib_export_plugin_description_file(controller_interface bg431esc1_plugin.xml)

# if(BUILD_TESTING)
#   find_package(ament_cmake_gmock REQUIRED)
#   find_package(controller_manager REQUIRED)
#   find_package(ros2_control_test_assets REQUIRED)

#   # ament_add_gmock(test_bg431esc1_actuator
#   #   test/test_bg431esc1_actuator.cpp)
#   # target_link_libraries(test_bg431esc1_actuator bg431esc1_actuator)
#   # ament_target_dependencies(test_bg431esc1_actuator
#   #   geometry_msgs
#   #   hardware_interface
#   #   nav_msgs
#   #   rclcpp
#   #   rclcpp_lifecycle
#   #   realtime_tools
#   #   tf2
#   #   tf2_msgs
#   # )

#   add_definitions(-DTEST_FILES_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/test")
#   ament_add_gmock(test_load_bg431esc1_actuator test/test_load_bg431esc1_actuator.cpp)
#   ament_target_dependencies(test_load_bg431esc1_actuator
#     controller_manager
#     ros2_control_test_assets
#   )

#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# Install the header files
install(
  DIRECTORY include/
  DESTINATION include/bg431esc1_actuator
)

install(TARGETS bg431esc1_actuator bg431esc1_actuator_parameters
  EXPORT export_bg431esc1_actuator
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_bg431esc1_actuator HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()