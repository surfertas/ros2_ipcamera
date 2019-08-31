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
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "ros2_ipcamera/visibility_control.h"
#include <chrono>

using namespace std::chrono_literals;

namespace ros2_ipcamera
{
class IpCamera : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit IpCamera(const rclcpp::NodeOptions & options);

  void execute();

private:
  std::string mat_type2encoding(int mat_type);
  void convert_frame_to_message(const cv::Mat & frame, size_t frame_id, sensor_msgs::msg::Image & msg);

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::QoS qos_;
  std::chrono::milliseconds freq_ = 30ms;
};

}  // namespace ros2_ipcamera

#endif // IPCAMERA_COMPONENT_H
