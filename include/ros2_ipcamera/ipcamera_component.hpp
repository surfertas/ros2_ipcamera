// Copyright (c) 2019 Tasuku Miura
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef ROS2_IPCAMERA__IPCAMERA_COMPONENT_HPP_
#define ROS2_IPCAMERA__IPCAMERA_COMPONENT_HPP_

#include <chrono>
#include <cstddef>
#include <memory>
#include <string>

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "ros2_ipcamera/visibility_control.hpp"

namespace ros2_ipcamera
{
class IpCamera : public rclcpp::Node
{
public:
  /**
   * Instantiates the IpCamera Node.
   */
  COMPOSITION_PUBLIC
  explicit IpCamera(const std::string & node_name, const rclcpp::NodeOptions & options);

  /**
   * Delegates construction.
   */
  COMPOSITION_PUBLIC
  explicit IpCamera(const rclcpp::NodeOptions & options);

  /**
   * Configures the video capture and camera calibration.
   */
  COMPOSITION_PUBLIC
  void configure();

  /**
   * Declares node parameters.
   */
  COMPOSITION_PUBLIC
  void initialize_parameters();

  /**
   * Captures and publishes one frame.
   */
  COMPOSITION_PUBLIC
  void execute();

private:
  std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_;
  std::string camera_calibration_file_param_;

  image_transport::CameraPublisher pub_;
  rclcpp::QoS qos_;
  rclcpp::TimerBase::SharedPtr capture_timer_;
  std::chrono::milliseconds frame_period_{30};

  cv::VideoCapture cap_;
  std::string source_;
  int width_;
  int height_;
  std::size_t frame_id_{0};

  std::string mat_type2encoding(int mat_type);

  void convert_frame_to_message(
    const cv::Mat & frame,
    std::size_t frame_id,
    sensor_msgs::msg::Image & msg,
    sensor_msgs::msg::CameraInfo & camera_info_msg);
};
}  // namespace ros2_ipcamera

#endif  // ROS2_IPCAMERA__IPCAMERA_COMPONENT_HPP_
