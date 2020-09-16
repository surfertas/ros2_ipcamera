#ifndef CUSTOM_QOS_HPP_
#define CUSTOM_QOS_HPP_

#include <rmw/rmw.h>
#include <rclcpp/qos.hpp>
#include <ros2_ipcamera/visibility_control.hpp>

namespace ros2_ipcamera
{
  /**
   * Customize the 'SensorDataQoS'
   */
   class CustomSensorDataQoS : public rclcpp::SensorDataQoS
   {
   public:
     COMPOSITION_PUBLIC
     explicit CustomSensorDataQoS() : rclcpp::SensorDataQoS()
     {
       this->keep_last(1);
     }
   };

} // namespace ros2_ipcamera

#endif // CUSTOM_QOS_HPP_
