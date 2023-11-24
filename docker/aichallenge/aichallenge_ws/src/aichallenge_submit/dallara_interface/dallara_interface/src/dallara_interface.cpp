// Copyright 2023 TIER IV, Inc. All rights reserved.
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
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "dallara_interface.hpp"

namespace dallara_interface {

  using std::placeholders::_1;

  DallaraInterface::DallaraInterface() : Node("dallara_interface_node") {
    // Publishers

    vehicle_inputs_pub_ = this->create_publisher<VehicleInputs>("/vehicle_inputs", 1);

    velocity_report_pub_ = this->create_publisher<VelocityReport>("/vehicle/status/velocity_status", 1);
    steering_report_pub_ = this->create_publisher<SteeringReport>("/vehicle/status/steering_status", 1);
    gear_report_pub_ = this->create_publisher<GearReport>("/vehicle/status/gear_status", 1);

    gear_cmd_dallara_ = 0;
    current_speed_ = 0.0;
    // Subscribers

    actuation_cmd_sub_ = this->create_subscription<ActuationCommandStamped>(
      "/control/command/actuation_cmd", 1, 
      std::bind(&DallaraInterface::actuation_callback, this, std::placeholders::_1)
    );

    gear_cmd_sub_ = this->create_subscription<GearCommand>(
      "/control/command/gear_cmd", 1, 
      std::bind(&DallaraInterface::gear_cmd_callback, this, std::placeholders::_1)
    );

    powertrain_data_sub_ = this->create_subscription<PowertrainData>(
      "/powertrain_data", 1, 
      std::bind(&DallaraInterface::powertrain_data_callback, this, std::placeholders::_1)
    );

    vehicle_data_sub_ = this->create_subscription<VehicleData>(
      "/vehicle_data", 1, 
      std::bind(&DallaraInterface::vehicle_data_callback, this, std::placeholders::_1)
    );

  }

  void DallaraInterface::actuation_callback(const ActuationCommandStamped::SharedPtr msg) {
    // Convert accel_cmd, brake_cmd, steer_cmd from ActuationCommandStamped
    // to autonoma_msgs/VehicleInputs for racing car control.
    VehicleInputs vehicle_inputs_msg;
    vehicle_inputs_msg.header.stamp = get_clock()->now();
    vehicle_inputs_msg.throttle_cmd = msg->actuation.accel_cmd * 100.0; // to 0~100%
    vehicle_inputs_msg.brake_cmd = msg->actuation.brake_cmd * 1000; // to Pascal
    vehicle_inputs_msg.steering_cmd = msg->actuation.steer_cmd * 180.0 / M_PI * 19.5;
    const double speed_kmph = current_speed_ * 3.6;
    const double gear_thresholds[] = { -1.0, 30.0, 60.0, 90.0, 120.0, 150.0 };
    const int num_gears = sizeof(gear_thresholds) / sizeof(gear_thresholds[0]);
    int gear_cmd_dallara_ = 1;
    for (int i = 1; i < num_gears; ++i) {
        if (speed_kmph < gear_thresholds[i]) {
            break;
        }
        gear_cmd_dallara_ = i + 1;
    }
    vehicle_inputs_msg.gear_cmd = gear_cmd_dallara_;
    vehicle_inputs_pub_->publish(vehicle_inputs_msg);
  }

  void DallaraInterface::gear_cmd_callback(GearCommand::SharedPtr msg) {
    int command = msg->command;
  }

  void DallaraInterface::powertrain_data_callback(PowertrainData::SharedPtr msg) {
    VelocityReport velocity_report_msg;
    velocity_report_msg.header.stamp = get_clock()->now();
    velocity_report_msg.lateral_velocity = 0.0;
    current_speed_ = msg->vehicle_speed_kmph / 3.6; //km/h to m/s
    velocity_report_msg.longitudinal_velocity = current_speed_;
    velocity_report_pub_->publish(velocity_report_msg);

    GearReport gear_report_msg;
    gear_report_msg.stamp = get_clock()->now();
    gear_report_msg.report = msg->current_gear + 1;
    gear_report_pub_->publish(gear_report_msg);
  }

  void DallaraInterface::vehicle_data_callback(VehicleData::SharedPtr msg) {
    SteeringReport steering_report_msg;
    steering_report_msg.stamp = get_clock()->now();
    steering_report_msg.steering_tire_angle = msg->steering_wheel_angle * M_PI / 180.0;
    steering_report_pub_->publish(steering_report_msg);
  }

}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dallara_interface::DallaraInterface>());
  rclcpp::shutdown();
  return 0;
}
