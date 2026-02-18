cmake_minimum_required(VERSION 3.8)
project(gnss_mgrs_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(llh_converter REQUIRED)
find_package(eagleye_coordinate REQUIRED)

# Find Eigen3 (this should still work as Eigen usually provides a config file)
find_package(Eigen3 REQUIRED CONFIG)

# --- MANUAL FIND FOR GEOGRAPHICLIB ---
# This explicitly searches for the GeographicLib header and library files.
# It assumes GeographicLib is installed as a system library (e.g., via apt-get).
find_path(GEOGRAPHICLIB_INCLUDE_DIR GeographicLib/Config.h
  # Common include paths where GeographicLib headers might be found
  PATHS /usr/include /usr/local/include
  PATH_SUFFIXES GeographicLib
  REQUIRED # Make this a hard requirement
)

find_library(GEOGRAPHICLIB_LIBRARY
  # Common names for the GeographicLib library
  NAMES Geographic GeographicLib
  # Common library paths
  PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu
  REQUIRED # Make this a hard requirement
)

if(NOT GEOGRAPHICLIB_INCLUDE_DIR OR NOT GEOGRAPHICLIB_LIBRARY)
  message(FATAL_ERROR "Could not find GeographicLib. Please ensure 'libgeographic-dev' is installed.")
endif()
# -------------------------------------

add_executable(gnss_mgrs_converter_node
  src/gnss_mgrs_converter_node.cpp
)

# Add GeographicLib include directory to your executable's include paths
target_include_directories(gnss_mgrs_converter_node PUBLIC
  ${GEOGRAPHICLIB_INCLUDE_DIR}
)

ament_target_dependencies(gnss_mgrs_converter_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  ament_index_cpp
  llh_converter
  eagleye_coordinate
  # Eigen3 is linked via target_link_libraries, not ament_target_dependencies for direct use
)

# Link GeographicLib and Eigen3 to your executable
target_link_libraries(gnss_mgrs_converter_node
  ${GEOGRAPHICLIB_LIBRARY} # Link the found GeographicLib library
  Eigen3::Eigen            # Link Eigen3 via its CMake target
)

target_compile_features(gnss_mgrs_converter_node PUBLIC cxx_std_17)

install(TARGETS
  gnss_mgrs_converter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies for other packages that might depend on yours.
# GeographicLib will be implicitly handled if this package is used as a library
# by others, but explicitly listing it helps clarify dependencies.
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  ament_index_cpp
  llh_converter
  # Export GeographicLib as a dependency, though it's not an ament package itself
  # This serves as documentation for downstream packages.
  # The actual linking is handled by target_link_libraries.
  GeographicLib
  Eigen3
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()