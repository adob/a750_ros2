cmake_minimum_required(VERSION 3.10)
project(a750_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wall -Wextra -Wconversion
            -Wno-sign-compare -Wno-deprecated -Wno-sign-conversion
            -Wno-missing-field-initializers
            -Werror=shift-count-overflow
            -Werror=return-type)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(${PROJECT_NAME} STATIC
  src/a750_system.cpp
)

add_subdirectory(third_party/baselib EXCLUDE_FROM_ALL)
add_subdirectory(third_party/serialrpc EXCLUDE_FROM_ALL)
add_subdirectory(third_party/a750_control EXCLUDE_FROM_ALL)

 
target_include_directories(a750_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(a750_hardware
  rclcpp rclcpp_lifecycle hardware_interface pluginlib
)

#target_link_libraries(${PROJECT_NAME} l)

#target_link_libraries(${PROJECT_NAME} PUBLIC baselib::baselib)
#target_link_libraries(${PROJECT_NAME} a750_control::a750_control)
#target_link_libraries(${PROJECT_NAME} serialrpc::serialrpc)
target_link_libraries(${PROJECT_NAME} baselib::baselib)
target_link_libraries(${PROJECT_NAME} a750_control::a750_control)

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
