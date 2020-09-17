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
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>
#include "ros2_ipcamera/ipcamera_component.hpp"


namespace ros2_ipcamera
{
  IpCamera::IpCamera(const std::string & node_name, const rclcpp::NodeOptions & options)
  : Node(node_name, options),
    qos_(rclcpp::QoS(rclcpp::KeepLast(1)).best_effort())
  {
    RCLCPP_INFO(this->get_logger(), "namespace: %s", this->get_namespace());
    RCLCPP_INFO(this->get_logger(), "name: %s", this->get_name());
    RCLCPP_INFO(this->get_logger(),
                "middleware: %s", rmw_get_implementation_identifier());

    // Declare parameters.
    this->initialize_parameters();

    this->configure();

    //TODO(Tasuku): add call back to handle parameter events.
    // Set up publishers.
    this->pub_ = image_transport::create_camera_publisher(
      this, "~/image_raw", qos_.get_rmw_qos_profile());

    this->execute();
  }

  IpCamera::IpCamera(const rclcpp::NodeOptions & options)
  : IpCamera::IpCamera("ipcamera", options)
  {}

  void
  IpCamera::configure()
  {
    rclcpp::Logger node_logger = this->get_logger();

    // TODO(Tasuku): move to on_configure() when rclcpp_lifecycle available.
    this->get_parameter<std::string>("rtsp_uri", source_);
    RCLCPP_INFO(node_logger, "rtsp_uri: %s", source_.c_str());

    this->get_parameter<std::string>("camera_calibration_file", camera_calibration_file_param_);
    RCLCPP_INFO(node_logger, "camera_calibration_file: %s",
                camera_calibration_file_param_.c_str());

    this->get_parameter<int>("image_width", width_);
    RCLCPP_INFO(node_logger, "image_width: %d", width_);

    this->get_parameter<int>("image_height", height_);
    RCLCPP_INFO(node_logger, "image_height: %d", height_);

    // TODO(Tasuku): move to on_configure() when rclcpp_lifecycle available.
    this->cap_.open(source_);

    // Set the width and height based on command line arguments.
    // The width, height has to match the available resolutions of the IP camera.
    this->cap_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width_));
    this->cap_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height_));
    if (!this->cap_.isOpened()) {
      RCLCPP_ERROR(node_logger, "Could not open video stream");
      throw std::runtime_error("Could not open video stream");
    }

    // TODO(Tasuku): move to on_configure() when rclcpp_lifecycle available.
    // https://docs.ros.org/api/camera_info_manager/html/classcamera__info__manager_1_1CameraInfoManager.html#_details
    // Make sure that cname is equal to camera_name in camera_info.yaml file. Default cname is set to "camera".
    cinfo_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);
    if (cinfo_manager_->validateURL(camera_calibration_file_param_)) {
      cinfo_manager_->loadCameraInfo(camera_calibration_file_param_);
    } else {
      RCLCPP_WARN(node_logger, "CameraInfo URL not valid.");
      RCLCPP_WARN(node_logger, "URL IS %s", camera_calibration_file_param_.c_str());
    }
  }

  void
  IpCamera::initialize_parameters()
  {
    rcl_interfaces::msg::ParameterDescriptor rtsp_uri_descriptor;
    rtsp_uri_descriptor.name = "rtsp_uri";
    rtsp_uri_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    rtsp_uri_descriptor.description = "RTSP URI of the IP camera.";
    rtsp_uri_descriptor.additional_constraints = "Should be of the form 'rtsp://";
    this->declare_parameter("rtsp_uri", "", rtsp_uri_descriptor);

    rcl_interfaces::msg::ParameterDescriptor camera_calibration_file_descriptor;
    camera_calibration_file_descriptor.name = "camera_calibration_file";
    camera_calibration_file_descriptor.type =
      rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    this->declare_parameter(
      "camera_calibration_file", "", camera_calibration_file_descriptor);

    rcl_interfaces::msg::ParameterDescriptor image_width_descriptor;
    image_width_descriptor.name = "image_width";
    image_width_descriptor.type =
      rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    this->declare_parameter("image_width", 640, image_width_descriptor);

    rcl_interfaces::msg::ParameterDescriptor image_height_descriptor;
    image_height_descriptor.name = "image_height";
    image_height_descriptor.type =
      rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    this->declare_parameter("image_height", 480, image_height_descriptor);
  }

  void
  IpCamera::execute()
  {
    rclcpp::Rate loop_rate(freq_);

    auto camera_info_msg = std::make_shared<sensor_msgs::msg::CameraInfo>(cinfo_manager_->getCameraInfo());

    // Initialize OpenCV image matrices.
    cv::Mat frame;

    size_t frame_id = 0;
    // Our main event loop will spin until the user presses CTRL-C to exit.
    while (rclcpp::ok()) {
      // Initialize a shared pointer to an Image message.
      auto msg = std::make_unique<sensor_msgs::msg::Image>();
      msg->is_bigendian = false;

      // Get the frame from the video capture.
      this->cap_ >> frame;
      // Check if the frame was grabbed correctly
      if (!frame.empty()) {
        // Convert to a ROS image
        convert_frame_to_message(frame, frame_id, *msg, *camera_info_msg);
        // Publish the image message and increment the frame_id.
        this->pub_.publish(std::move(msg), camera_info_msg);
        ++frame_id;
      }
      loop_rate.sleep();
    }
  }

  std::string
  IpCamera::mat_type2encoding(int mat_type)
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

  void
  IpCamera::convert_frame_to_message(
    const cv::Mat & frame,
    size_t frame_id,
    sensor_msgs::msg::Image & msg,
    sensor_msgs::msg::CameraInfo & camera_info_msg)
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
