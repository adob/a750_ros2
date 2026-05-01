cmake_minimum_required(VERSION 3.10)
project(a750_hardware)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wconversion
            -Wno-sign-compare -Wno-deprecated -Wno-sign-conversion
            -Wno-missing-field-initializers
            -Werror=shift-count-overflow
            -Werror=return-type)
endif()

include(cmake/CPM.cmake)

CPMAddPackage(
  NAME a750_control
  GITHUB_REPOSITORY adob/a750_control
  VERSION 0.0.0
  GIT_TAG ad10c3dda79716600c088393482130ed1f40af8e
)

CPMAddPackage(
  NAME baselib
  GITHUB_REPOSITORY adob/baselib
  VERSION 0.0.0
  GIT_TAG 36f6c17ea5fcf481d9d6849566a980ca82b7d6e3
  EXCLUDE_FROM_ALL ${A750_CONTROL_EXCLUDE_FROM_ALL}
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/a750_system.cpp
)

target_include_directories(a750_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(a750_hardware
  rclcpp rclcpp_lifecycle hardware_interface pluginlib
)

target_link_libraries(${PROJECT_NAME} 
  baselib::baselib
  a750_control::a750_control
)

pluginlib_export_plugin_description_file(hardware_interface a750_hardware_plugins.xml)

install(
  TARGETS a750_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(FILES a750_hardware_plugins.xml DESTINATION share/${PROJECT_NAME})

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

ament_package()
