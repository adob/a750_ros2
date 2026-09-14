cmake_minimum_required(VERSION 3.30)
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
  GIT_TAG 69f9284db4267da1eea2d51b19beb61e0a57e56b
)

CPMAddPackage(
  NAME baselib
  GITHUB_REPOSITORY adob/baselib
  VERSION 0.0.0
  GIT_TAG aac03db111a5c89ff95a9f0a8b2512f6112b4248
  EXCLUDE_FROM_ALL ${A750_CONTROL_EXCLUDE_FROM_ALL}
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# realtime_tools exports a bare "cap" dependency. Resolve it through CMake's
# prefixes so linking also works when libcap lives outside the system directories.
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND NOT TARGET cap)
  find_library(LIBCAP_LIBRARY NAMES cap REQUIRED)
  add_library(cap UNKNOWN IMPORTED)
  set_target_properties(cap PROPERTIES IMPORTED_LOCATION "${LIBCAP_LIBRARY}")
endif()

buildtool_register_project(a750_hardware_sources SOURCE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}")

target_include_directories(a750_hardware_sources PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(a750_hardware_sources PUBLIC
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  a750_control_sources
)

buildtool_add_library(a750_hardware_objects
  LIBRARY a750_hardware_sources SOURCES src/a750_system.cpp)
add_dependencies(a750_hardware_objects_build
  a750_control_generate_protocol serialrpc_generate_protocol)

# Buildtool compiles the plugin and its imports; CMake links and installs the DSO.
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/hardware_link.cc"
  CONTENT "// Plugin sources are compiled by buildtool.\n")
add_library(${PROJECT_NAME} SHARED "${CMAKE_CURRENT_BINARY_DIR}/hardware_link.cc")
# Keep plugin registration and exported entry points even without direct callers.
target_link_libraries(${PROJECT_NAME} PRIVATE
  "$<LINK_LIBRARY:WHOLE_ARCHIVE,a750_hardware_objects>")

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
