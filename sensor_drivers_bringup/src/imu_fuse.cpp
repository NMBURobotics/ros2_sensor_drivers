// Copyright (c) 2020 Fetullah Atas, Norwegian University of Life Sciences
//
// Licensed under the Apache License, Version 2.0 (the "License"};
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

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class ImuFuse : public rclcpp::Node
{
public:
  ImuFuse(const rclcpp::NodeOptions & node_options);

private:
  void onXsensImuMsg(const sensor_msgs::msg::Imu::SharedPtr msg);
  void onGPSImuMsg(const sensor_msgs::msg::Imu::SharedPtr msg);

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr fused_imu_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr xsens_imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr gps_heading_imu_sub_;

  sensor_msgs::msg::Imu::SharedPtr latest_gps_msg_;
};

ImuFuse::ImuFuse(const rclcpp::NodeOptions & node_options)
: Node("imu_fuse", node_options)
{
  fused_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/absolute", rclcpp::SensorDataQoS());
  
  latest_gps_msg_ = std::make_shared<sensor_msgs::msg::Imu>();

  rclcpp::QoS qos = rclcpp::SensorDataQoS(rclcpp::KeepLast(1));

  gps_heading_imu_sub_ =
    this->create_subscription<sensor_msgs::msg::Imu>(
    "heading/imu", qos, std::bind(&ImuFuse::onGPSImuMsg, this, std::placeholders::_1));

      xsens_imu_sub_ =
    this->create_subscription<sensor_msgs::msg::Imu>(
    "imu/data", qos, std::bind(&ImuFuse::onXsensImuMsg, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),"Creating...");
}

void ImuFuse::onXsensImuMsg(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  auto fused = std::make_shared<sensor_msgs::msg::Imu>(*msg);
  fused->orientation = latest_gps_msg_->orientation;
  fused_imu_pub_->publish(*fused);
}

void ImuFuse::onGPSImuMsg(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  latest_gps_msg_ = msg;
}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(ImuFuse)
