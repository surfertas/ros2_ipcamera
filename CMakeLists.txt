cmake_minimum_required(VERSION 3.16)
project(ros2_ipcamera)

# Default to C++17, the minimum supported by ROS 2 Lyrical.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rmw REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV 4 REQUIRED)

add_library(ipcamera_component SHARED
  src/ipcamera_component.cpp)
target_compile_definitions(ipcamera_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
target_include_directories(ipcamera_component
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
    ${OpenCV_INCLUDE_DIRS})

ament_target_dependencies(ipcamera_component
  "rclcpp"
  "rclcpp_components"
  "rcl_interfaces"
  "rmw"
  "cv_bridge"
  "sensor_msgs"
  "camera_info_manager"
  "camera_calibration_parsers"
  "image_transport")
target_link_libraries(ipcamera_component
  ${OpenCV_LIBS})

rclcpp_components_register_nodes(ipcamera_component "ros2_ipcamera::IpCamera")

add_executable(composition
  src/composition.cpp)
target_link_libraries(composition
  ipcamera_component)
ament_target_dependencies(composition
  "rclcpp")

install(TARGETS
  ipcamera_component
  EXPORT export_ros2_ipcamera
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

install(TARGETS
  composition
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  add_launch_test(test/test_rtsp_stream.py TIMEOUT 60)
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

install(FILES LICENSE
  DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_ros2_ipcamera HAS_LIBRARY_TARGET)
ament_export_dependencies(
  OpenCV
  camera_info_manager
  image_transport
  rcl_interfaces
  rclcpp
  rmw
  sensor_msgs)

ament_package()
