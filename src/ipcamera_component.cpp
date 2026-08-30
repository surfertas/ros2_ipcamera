// Copyright (c) 2019 Tasuku
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
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>
#include <rmw/rmw.h>

#include "ros2_ipcamera/ipcamera_component.hpp"

namespace ros2_ipcamera
{
IpCamera::IpCamera(const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options),
  qos_(rclcpp::QoS(rclcpp::KeepLast(1)).best_effort())
{
  RCLCPP_INFO(this->get_logger(), "namespace: %s", this->get_namespace());
  RCLCPP_INFO(this->get_logger(), "name: %s", this->get_name());
  RCLCPP_INFO(
    this->get_logger(), "middleware: %s", rmw_get_implementation_identifier());

  initialize_parameters();
  configure();

  pub_ = image_transport::create_camera_publisher(
    image_transport::RequiredInterfaces(*this), "~/image_raw", qos_);
  capture_timer_ = create_wall_timer(
    frame_period_, std::bind(&IpCamera::execute, this));
}

IpCamera::IpCamera(const rclcpp::NodeOptions & options)
: IpCamera("ipcamera", options)
{}

void IpCamera::configure()
{
  const rclcpp::Logger node_logger = get_logger();

  get_parameter("rtsp_uri", source_);
  RCLCPP_INFO(
    node_logger, "RTSP URI is %s", source_.empty() ? "not configured" : "configured");

  get_parameter("frame_id", frame_id_);
  RCLCPP_INFO(node_logger, "frame_id: %s", frame_id_.c_str());

  get_parameter("camera_calibration_file", camera_calibration_file_param_);
  RCLCPP_INFO(
    node_logger, "camera_calibration_file: %s",
    camera_calibration_file_param_.c_str());

  get_parameter("image_width", width_);
  RCLCPP_INFO(node_logger, "image_width: %d", width_);

  get_parameter("image_height", height_);
  RCLCPP_INFO(node_logger, "image_height: %d", height_);

  cap_.open(source_);
  cap_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width_));
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height_));
  if (!cap_.isOpened()) {
    RCLCPP_ERROR(node_logger, "Could not open video stream");
    throw std::runtime_error("Could not open video stream");
  }

  cinfo_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
    get_node_base_interface(),
    get_node_services_interface(),
    get_node_logging_interface(),
    "camera",
    "",
    rclcpp::SystemDefaultsQoS());
  if (cinfo_manager_->validateURL(camera_calibration_file_param_)) {
    cinfo_manager_->loadCameraInfo(camera_calibration_file_param_);
  } else {
    RCLCPP_WARN(
      node_logger, "CameraInfo URL is invalid: %s",
      camera_calibration_file_param_.c_str());
  }
}

void IpCamera::initialize_parameters()
{
  rcl_interfaces::msg::ParameterDescriptor rtsp_uri_descriptor;
  rtsp_uri_descriptor.name = "rtsp_uri";
  rtsp_uri_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  rtsp_uri_descriptor.description = "RTSP URI of the IP camera.";
  rtsp_uri_descriptor.additional_constraints = "Should start with 'rtsp://'";
  declare_parameter("rtsp_uri", "", rtsp_uri_descriptor);

  rcl_interfaces::msg::ParameterDescriptor frame_id_descriptor;
  frame_id_descriptor.name = "frame_id";
  frame_id_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  frame_id_descriptor.description = "Coordinate frame associated with the camera image.";
  declare_parameter("frame_id", "camera_optical_frame", frame_id_descriptor);

  rcl_interfaces::msg::ParameterDescriptor camera_calibration_file_descriptor;
  camera_calibration_file_descriptor.name = "camera_calibration_file";
  camera_calibration_file_descriptor.type =
    rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
  declare_parameter(
    "camera_calibration_file", "", camera_calibration_file_descriptor);

  rcl_interfaces::msg::ParameterDescriptor image_width_descriptor;
  image_width_descriptor.name = "image_width";
  image_width_descriptor.type =
    rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  declare_parameter("image_width", 640, image_width_descriptor);

  rcl_interfaces::msg::ParameterDescriptor image_height_descriptor;
  image_height_descriptor.name = "image_height";
  image_height_descriptor.type =
    rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  declare_parameter("image_height", 480, image_height_descriptor);
}

void IpCamera::execute()
{
  cv::Mat frame;
  cap_ >> frame;
  if (frame.empty()) {
    return;
  }

  auto image_msg = std::make_unique<sensor_msgs::msg::Image>();
  auto camera_info_msg =
    std::make_unique<sensor_msgs::msg::CameraInfo>(cinfo_manager_->getCameraInfo());
  image_msg->is_bigendian = false;

  convert_frame_to_message(frame, *image_msg, *camera_info_msg);
  pub_.publish(std::move(image_msg), std::move(camera_info_msg));
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
  const cv::Mat & frame,
  sensor_msgs::msg::Image & msg,
  sensor_msgs::msg::CameraInfo & camera_info_msg)
{
  msg.height = frame.rows;
  msg.width = frame.cols;
  msg.encoding = mat_type2encoding(frame.type());
  msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
  const std::size_t size = frame.step * frame.rows;
  msg.data.resize(size);
  std::memcpy(msg.data.data(), frame.data, size);

  const rclcpp::Time timestamp = get_clock()->now();

  msg.header.frame_id = frame_id_;
  msg.header.stamp = timestamp;
  camera_info_msg.header.frame_id = frame_id_;
  camera_info_msg.header.stamp = timestamp;
}
}  // namespace ros2_ipcamera

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_ipcamera::IpCamera)
