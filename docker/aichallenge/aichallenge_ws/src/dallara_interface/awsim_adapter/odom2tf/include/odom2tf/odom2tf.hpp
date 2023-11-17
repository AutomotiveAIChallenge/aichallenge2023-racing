// Copyright 2023 Tier IV, Inc. All rights reserved.
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

#include <chrono>
#include <functional>
#include <memory>
#include "nav_msgs/msg/odometry.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

using nav_msgs::msg::Odometry;
using tf2_msgs::msg::TFMessage;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::TwistWithCovarianceStamped;

class Odom2tf : public rclcpp::Node {
public:
  Odom2tf();

private:
  void odomCallback(const Odometry::SharedPtr odometry);


  rclcpp::Subscription<Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<TFMessage>::SharedPtr pub_tf_;
  rclcpp::Publisher<TwistWithCovarianceStamped>::SharedPtr pub_twist_;

};
