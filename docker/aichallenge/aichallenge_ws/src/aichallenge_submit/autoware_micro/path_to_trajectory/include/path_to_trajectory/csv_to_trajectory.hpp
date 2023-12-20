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

#ifndef PATH_TO_TRAJECTORY__CSV_TO_TRAJECTORY_HPP_
#define PATH_TO_TRAJECTORY__CSV_TO_TRAJECTORY_HPP_

#include "autoware_auto_planning_msgs/msg/trajectory.hpp"
#include "autoware_auto_planning_msgs/msg/trajectory_point.hpp"
#include "rclcpp/rclcpp.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

class CsvToTrajectory : public rclcpp::Node {
 public:
  using Trajectory = autoware_auto_planning_msgs::msg::Trajectory;
  using TrajectoryPoint = autoware_auto_planning_msgs::msg::TrajectoryPoint;

 public:
  CsvToTrajectory();

 private:
  rclcpp::Publisher<Trajectory>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<TrajectoryPoint> trajectory_points_;

  size_t current_point_index_ = 0;
  float velocity_rate = 1.0f;

private:
  void timerCallback();
  void readCsv(const std::string& csv_file_path);
};

#endif  // PATH_TO_TRAJECTORY__CSV_TO_TRAJECTORY_HPP_