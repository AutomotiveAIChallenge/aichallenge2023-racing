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
#include "rclcpp/rclcpp.hpp"
#include "tier4_vehicle_msgs/msg/actuation_command_stamped.hpp"
#include "autoware_auto_vehicle_msgs/msg/velocity_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_report.hpp"
#include "autoware_auto_vehicle_msgs/msg/gear_command.hpp"
#include "autonoma_msgs/msg/vehicle_inputs.hpp"
#include "autonoma_msgs/msg/vehicle_data.hpp"
#include "autonoma_msgs/msg/powertrain_data.hpp"

namespace dallara_interface {

  using ActuationCommandStamped = tier4_vehicle_msgs::msg::ActuationCommandStamped;
  using VelocityReport = autoware_auto_vehicle_msgs::msg::VelocityReport;
  using SteeringReport = autoware_auto_vehicle_msgs::msg::SteeringReport;
  using GearReport = autoware_auto_vehicle_msgs::msg::GearReport;
  using GearCommand = autoware_auto_vehicle_msgs::msg::GearCommand;

  using VehicleInputs = autonoma_msgs::msg::VehicleInputs;
  using PowertrainData = autonoma_msgs::msg::PowertrainData;
  using VehicleData = autonoma_msgs::msg::VehicleData;

  class DallaraInterface : public rclcpp::Node {
  public:
    DallaraInterface();

    void actuation_callback(ActuationCommandStamped::SharedPtr msg);
    void gear_cmd_callback(GearCommand::SharedPtr msg);
    void powertrain_data_callback(PowertrainData::SharedPtr msg);
    void vehicle_data_callback(VehicleData::SharedPtr msg);

  private:
    int gear_cmd_dallara_;
    double current_speed_;

    // Publishers
    rclcpp::Publisher<VehicleInputs>::SharedPtr vehicle_inputs_pub_;
    rclcpp::Publisher<VelocityReport>::SharedPtr velocity_report_pub_;
    rclcpp::Publisher<SteeringReport>::SharedPtr steering_report_pub_;
    rclcpp::Publisher<GearReport>::SharedPtr gear_report_pub_;
    
    // Subscribers
    rclcpp::Subscription<ActuationCommandStamped>::SharedPtr actuation_cmd_sub_;
    rclcpp::Subscription<GearCommand>::SharedPtr gear_cmd_sub_;
    rclcpp::Subscription<PowertrainData>::SharedPtr powertrain_data_sub_;
    rclcpp::Subscription<VehicleData>::SharedPtr vehicle_data_sub_;
  };

}
