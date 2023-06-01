cmake_minimum_required(VERSION 3.5.6)
project(ros2_ipcamera)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV 4 REQUIRED)

include_directories(include)

set(node_plugins "")

add_library(ipcamera_component SHARED
  src/ipcamera_component.cpp)
target_compile_definitions(ipcamera_component
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(ipcamera_component
  "rclcpp"
  "rclcpp_components"
  "cv_bridge"
  "OpenCV"
  "sensor_msgs"
  "camera_info_manager"
  "camera_calibration_parsers"
  "image_transport")

rclcpp_components_register_nodes(ipcamera_component "ros2_ipcamera::IpCamera")
set(node_plugins "${node_plugins}ros2_ipcamera::IpCamera;$<TARGET_FILE:ipcamera_component>\n")

# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

add_executable(composition
  src/composition.cpp)
target_link_libraries(composition
  ipcamera_component)
ament_target_dependencies(composition
  "rclcpp")

install(TARGETS
  ipcamera_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  composition
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
