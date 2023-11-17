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
#include "odom2tf/odom2tf.hpp"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

Odom2tf::Odom2tf() : Node("odom2tf_node") {
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  pub_tf_ = this->create_publisher<TFMessage>("/tf", 1);
  pub_twist_ = this->create_publisher<TwistWithCovarianceStamped>("out_twist", 1);
  sub_odom_ = this->create_subscription<Odometry>(
      "in_odom", qos, std::bind(&Odom2tf::odomCallback, this, _1));
}

void Odom2tf::odomCallback(const Odometry::SharedPtr odometry)
{
  TransformStamped tf;
  tf.header = odometry->header;
  tf.child_frame_id = odometry->child_frame_id;
  tf.transform.translation.x = odometry->pose.pose.position.x;
  tf.transform.translation.y = odometry->pose.pose.position.y;
  tf.transform.translation.z = odometry->pose.pose.position.z;
  tf.transform.rotation = odometry->pose.pose.orientation;
  tf2_msgs::msg::TFMessage tf_msg{};
  tf_msg.transforms.emplace_back(std::move(tf));
  pub_tf_->publish(tf_msg);

  TwistWithCovarianceStamped twist;
  twist.header = odometry->header;
  twist.twist = odometry->twist;
  pub_twist_->publish(twist);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odom2tf>());
  rclcpp::shutdown();
  return 0;
}
