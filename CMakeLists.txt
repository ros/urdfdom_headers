cmake_minimum_required( VERSION 3.8 FATAL_ERROR )
project (urdfdom_headers)

include(GNUInstallDirs)

option(APPEND_PROJECT_NAME_TO_INCLUDEDIR
  "When ON headers are installed to a folder ending with an extra ${PROJECT_NAME}. \
  This avoids include directory search order issues when overriding this package
  from a merged catkin, ament, or colcon workspace." ON)

if(APPEND_PROJECT_NAME_TO_INCLUDEDIR)
  set(CMAKE_INSTALL_INCLUDEDIR "${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}")
endif()

set (URDF_MAJOR_VERSION 1)
set (URDF_MINOR_VERSION 1)
set (URDF_PATCH_VERSION 2)

set (URDF_VERSION ${URDF_MAJOR_VERSION}.${URDF_MINOR_VERSION}.${URDF_PATCH_VERSION})

message (STATUS "${PROJECT_NAME} version ${URDF_VERSION}")

# This shouldn't be necessary, but there has been trouble
# with MSVC being set off, but MSVCXX ON.
if(MSVC OR MSVC90 OR MSVC10)
  set(MSVC ON)
endif (MSVC OR MSVC90 OR MSVC10)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

if(WIN32 AND NOT CYGWIN)
  set(CMAKE_CONFIG_INSTALL_DIR CMake)
else()
  set(CMAKE_CONFIG_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}/cmake)
endif()
string(REGEX REPLACE "[^/]+" ".." RELATIVE_PATH_CMAKE_DIR_TO_PREFIX "${CMAKE_CONFIG_INSTALL_DIR}")
string(REGEX REPLACE "[^/]+" ".." RELATIVE_PATH_LIBDIR_TO_PREFIX "${CMAKE_INSTALL_LIBDIR}")

set(PACKAGE_NAME ${PROJECT_NAME})
set(cmake_conf_file "${PROJECT_NAME}-config.cmake")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake/${cmake_conf_file}.in" "${CMAKE_BINARY_DIR}/${cmake_conf_file}" @ONLY)
set(cmake_conf_version_file "${PROJECT_NAME}-config-version.cmake")
# Use write_basic_package_version_file to generate a ConfigVersion file that
# allow users of gazebo to specify the API or version to depend on
# TODO: keep this instruction until deprecate Ubuntu/Precise and update with
# https://github.com/Kitware/CMake/blob/v2.8.8/Modules/CMakePackageConfigHelpers.cmake
include(WriteBasicConfigVersionFile)
write_basic_config_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${cmake_conf_version_file}
  VERSION "${URDF_VERSION}"
  COMPATIBILITY SameMajorVersion)
install(FILES
  "${CMAKE_BINARY_DIR}/${cmake_conf_file}"
  "${CMAKE_BINARY_DIR}/${cmake_conf_version_file}"
  DESTINATION ${CMAKE_CONFIG_INSTALL_DIR} COMPONENT cmake)

# Make the package config file
set(PACKAGE_DESC "Unified Robot Description Format")
set(pkg_conf_file "urdfdom_headers.pc")
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake/pkgconfig/${pkg_conf_file}.in" "${CMAKE_BINARY_DIR}/${pkg_conf_file}" @ONLY)
install(FILES "${CMAKE_BINARY_DIR}/${pkg_conf_file}" DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig/ COMPONENT pkgconfig)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME})
install(
  EXPORT ${PROJECT_NAME}
  DESTINATION ${CMAKE_CONFIG_INSTALL_DIR}
  NAMESPACE "${PROJECT_NAME}::"
  FILE "${PROJECT_NAME}Export.cmake"
)

# Add uninstall target
# Ref: http://www.cmake.org/Wiki/CMake_FAQ#Can_I_do_.22make_uninstall.22_with_CMake.3F
configure_file("${PROJECT_SOURCE_DIR}/cmake/uninstall.cmake.in" "${PROJECT_BINARY_DIR}/uninstall.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall "${CMAKE_COMMAND}" -P "${PROJECT_BINARY_DIR}/uninstall.cmake")

message(STATUS "Configuration successful. Type make install to install urdfdom_headers.")
