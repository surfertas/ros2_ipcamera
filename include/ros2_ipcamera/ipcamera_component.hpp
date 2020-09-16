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
#ifndef IPCAMERA_COMPONENT_H
#define IPCAMERA_COMPONENT_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "ros2_ipcamera/visibility_control.hpp"
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <chrono>


using namespace std::chrono_literals;

namespace ros2_ipcamera
{
  class IpCamera : public rclcpp::Node
  {
  public:
    /**
     * Instantiates the IpCamera Node.
     */
    COMPOSITION_PUBLIC
    explicit IpCamera(const std::string& node_name, const rclcpp::NodeOptions & options);

    /**
     * Delegates construction.
     */
    COMPOSITION_PUBLIC
    explicit IpCamera(const rclcpp::NodeOptions & options);

    /**;
     * Configures component.
     *
     * Declares parameters and configures video capture.
     */
    COMPOSITION_PUBLIC
    void
    configure();

    /**;
     * Declares the parameter using rcl_interfaces.
     */
    COMPOSITION_PUBLIC
    void
    initialize_parameters();

    /**;
     * Captures frame and converts frame to message.
     */
    COMPOSITION_PUBLIC
    void
    execute();

  private:
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_manager_;
    std::string camera_calibration_file_param_;

    image_transport::CameraPublisher pub_;
    rclcpp::QoS qos_;
    std::chrono::milliseconds freq_ = 30ms;

    cv::VideoCapture cap_;
    std::string source_;
    int width_;
    int height_;

    std::string
    mat_type2encoding(int mat_type);

    void
    convert_frame_to_message(
      const cv::Mat & frame,
      size_t frame_id,
      sensor_msgs::msg::Image & msg,
      sensor_msgs::msg::CameraInfo & camera_info_msg);
  };
}  // namespace ros2_ipcamera

#endif // IPCAMERA_COMPONENT_H
