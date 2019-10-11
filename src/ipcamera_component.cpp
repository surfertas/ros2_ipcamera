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
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include "ros2_ipcamera/ipcamera_component.hpp"
#include "rclcpp/rclcpp.hpp"

//TODO: Investigate why the camera info URL is not valid.

namespace ros2_ipcamera
{
IpCamera::IpCamera(const rclcpp::NodeOptions & options)
: Node("ipcamera", options),
  qos_(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data))
{
  // Declare parameters.
  this->declare_parameter<std::string>("rtsp_uri", "");
  this->declare_parameter<std::string>("image_topic", "");
  this->declare_parameter("camera_calibration_file", "");
  this->declare_parameter<int>("image_width", 0);
  this->declare_parameter<int>("image_height", 0);

  execute();
}

void IpCamera::execute()
{
  std::string source;
  std::string topic;
  std::string camera_calibration_file_param;

  int width;
  int height;

  // Get URI to pass as source to cap.
  this->get_parameter<std::string>("rtsp_uri", source);
  this->get_parameter<std::string>("image_topic", topic);
  this->get_parameter<std::string>("camera_calibration_file", camera_calibration_file_param);
  this->get_parameter<int>("image_width", width);
  this->get_parameter<int>("image_height", height);

  rclcpp::Logger node_logger = this->get_logger();

  RCLCPP_INFO(node_logger, "Publishing data on topic '%s'", topic.c_str());
  pub_ = image_transport::create_camera_publisher(this, topic, rmw_qos_profile_sensor_data);


  // https://docs.ros.org/api/camera_info_manager/html/classcamera__info__manager_1_1CameraInfoManager.html#_details
  // Make sure that cname is equal to camera_name in camera_info.yaml file. Default is set to "camera".
  cinfo_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);

  if (cinfo_manager_->validateURL(camera_calibration_file_param)) {
    cinfo_manager_->loadCameraInfo(camera_calibration_file_param);
  } else {
    RCLCPP_WARN(node_logger, "CameraInfo URL not valid.");
    RCLCPP_WARN(node_logger, "URL IS %s", camera_calibration_file_param.c_str());
  }

  rclcpp::WallRate loop_rate(freq_);

  // Initialize OpenCV video capture stream.
  cv::VideoCapture cap;
  cap.open(source);

  // Set the width and height based on command line arguments.
  cap.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width));
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height));
  if (!cap.isOpened()) {
    RCLCPP_ERROR(node_logger, "Could not open video stream");
    throw std::runtime_error("Could not open video stream");
  }

  // Initialize OpenCV image matrices.
  cv::Mat frame;

  size_t frame_id = 0;
  // Our main event loop will spin until the user presses CTRL-C to exit.
  while (rclcpp::ok()) {
    // Initialize a shared pointer to an Image message.
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->is_bigendian = false;

    auto camera_info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>(cinfo_manager_->getCameraInfo());

    // Get the frame from the video capture.
    cap >> frame;

    // Check if the frame was grabbed correctly
    if (!frame.empty()) {

      cv::resize(frame, frame, cv::Size(width, height), 0, 0, CV_INTER_AREA);
      // Convert to a ROS image
      convert_frame_to_message(frame, frame_id, *msg, *camera_info_msg);

      // Publish the image message and increment the frame_id.
      pub_.publish(std::move(msg), camera_info_msg);
      ++frame_id;
    }
    loop_rate.sleep();
  }
}

std::string IpCamera::mat_type2encoding(int mat_type)
{
  switch (mat_type) {
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "bgr8";
    case CV_16SC1:
      return "mono16";
    case CV_8UC4:
      return "rgba8";
    default:
      throw std::runtime_error("Unsupported encoding type");
  }
}

void IpCamera::convert_frame_to_message(
  const cv::Mat & frame, size_t frame_id, sensor_msgs::msg::Image & msg, sensor_msgs::msg::CameraInfo & camera_info_msg)
{
  // copy cv information into ros message
  msg.height = frame.rows;
  msg.width = frame.cols;
  msg.encoding = mat_type2encoding(frame.type());
  msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
  size_t size = frame.step * frame.rows;
  msg.data.resize(size);
  memcpy(&msg.data[0], frame.data, size);

  rclcpp::Time timestamp = this->get_clock()->now();

  msg.header.frame_id = std::to_string(frame_id);
  msg.header.stamp = timestamp;

  camera_info_msg.header.frame_id = std::to_string(frame_id);
  camera_info_msg.header.stamp = timestamp;
}
}
#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_ipcamera::IpCamera)
