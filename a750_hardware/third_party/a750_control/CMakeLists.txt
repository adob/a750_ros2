cmake_minimum_required(VERSION 3.16)

include(FetchContent)

get_filename_component(
  CURRENT_DIRNAME
  ${CMAKE_CURRENT_LIST_DIR}
  NAME
)

project(${CURRENT_DIRNAME} VERSION 0.0.0 LANGUAGES CXX)

if(NOT TARGET baselib::baselib)
  FetchContent_Declare(
    baselib
    GIT_REPOSITORY https://github.com/adob/baselib.git
    GIT_TAG 21c73f31b752670c5afac3339c0085b9d189049d
  )
  FetchContent_MakeAvailable(baselib)
endif()

if(NOT TARGET serialrpc::serialrpc)
  FetchContent_Declare(
    serialrpc
    GIT_REPOSITORY https://github.com/adob/serialrpc.git
    GIT_TAG b04c4cfbdb757cfcea88eed43f074e9650c166d2
  )
  FetchContent_MakeAvailable(serialrpc)
endif()

file(GLOB SOURCES
    ${CMAKE_CURRENT_LIST_DIR}/*.cc
)

foreach(src_file IN LISTS SOURCES)
  if(src_file MATCHES ".*_(test|posix|experimental|benchmark|arduino)\\.cc$")
    list(REMOVE_ITEM SOURCES ${src_file})
  endif()

  if(src_file MATCHES "(^|[^A-Za-z0-9_])example([^A-Za-z0-9_]|$)")
    list(REMOVE_ITEM SOURCES ${src_file})
  endif()
endforeach()

add_library(${PROJECT_NAME} STATIC
    ${SOURCES}
)

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_23)
#set_target_properties(baselib PROPERTIES
  #CXX_STANDARD 26
  #CXX_STANDARD_REQUIRED YES
  #CXX_EXTENSIONS NO
#)

get_filename_component(_parent_dir
  "${CMAKE_CURRENT_LIST_DIR}/.."
  REALPATH
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${_parent_dir}>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC baselib::baselib)
target_link_libraries(${PROJECT_NAME} PUBLIC serialrpc::serialrpc)
